#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define TYPE_VIRTUAL_CAN_CONTROLLER "virtual-can-controller"
OBJECT_DECLARE_SIMPLE_TYPE(VirtualCANControllerState, VIRTUAL_CAN_CONTROLLER)

#define MAX_CAN_BITS 135;

typedef enum {
    ERROR_ACTIVE, 
    ERROR_PASSIVE,
    BUS_OFF
} CANBusState;

typedef struct {
    uint32_t id;
    bool id_ready;
    uint8_t rtr;
    bool rtr_ready;
    uint8_t dlc;
    bool dlc_ready;
    uint8_t data[8];
    bool data_low_ready;
    bool data_high_ready;
} RoughCANFrame;

typedef struct {
    uint8_t bits[MAX_CAN_BITS];
    uint16_t length;
} CANFrame;

typedef struct VirtualCANControllerState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    uint64_t base_addr;
    QemuMutex lock;
    QEMUTimer *tx_timer;

    // CAN BUS STATE
    CANBusState can_bus_state;
    uint16_t tx_error_count;
    uint8_t rx_error_count;
    RoughCANFrame pending_rough_can_frame;

    // TX
    CANFrame tx_queue[10];
    uint8_t tx_queue_length;
    bool tx_in_progress;
    uint8_t tx_bit_cursor;
    uint8_t transmitted_bit;

    // RX
    CANFrame rx_queue[10];
    uint8_t rx_queue_length;
    bool rx_in_progress;
    uint8_t consecutive_bit_count; 
    uint8_t rx_bit_cursor;
    qemu_irq rx_irq; 

    // TCP CONNECTION
    int socket_fd;
    char server_address[256];
    uint16_t server_port;
} VirtualCANControllerState;

static uint64_t virtual_can_controller_read(void *opaque, hwaddr offset, unsigned size)
{
    VirtualCANControllerState *state = VIRTUAL_CAN_CONTROLLER(opaque);

    qemu_irq_lower(state->rx_irq);

    uint8_t req[6] = { 'R' }; // 'R' + addr(4B) + size(1B)
    uint64_t value = 0;
    uint8_t buf[8] = {0};
    int ret;

    qemu_mutex_lock(&state->lock);

    // Build request packet
    stl_le_p(req + 1, (uint32_t)offset);
    req[5] = (uint8_t)size;
    
    // Send request
    printf("Sending read request: addr=0x%08x, size=%u\n", (uint32_t)offset, size);
    ret = sizeof(req);
    if (ret != sizeof(req)) {
        error_report("virtual-can-controller: failed to send read request");
        goto unlock;
    }

    // Read response
    ret = size;
    if (ret != size) {
        error_report("virtual-can-controller: failed to read response");
        goto unlock;
    }

    switch (size) {
    case 1:
        value = buf[0];
        break;
    case 2:
        value = lduw_le_p(buf);
        break;
    case 4:
        value = ldl_le_p(buf);
        break;
    default:
        error_report("mmio-sockdev: invalid read size %u", size);
        break;
    }

    unlock:
    qemu_mutex_unlock(&state->lock);
    return value;
}

static void virtual_can_controller_write(void *opaque, hwaddr offset, uint64_t value, unsigned size) 
{
    VirtualCANControllerState *state = VIRTUAL_CAN_CONTROLLER(opaque);

    qemu_mutex_lock(&state->lock);

    switch(offset) {
        case 0x00: // ID
            if (value > 0x7FF) {
                error_report("virtual-can-controller: invalid 11-bit CAN ID");
                state->pending_rough_can_frame.id_ready = false;
                break;
            }
            state->pending_rough_can_frame.id = (uint32_t)value;
            state->pending_rough_can_frame.id_ready = true;
            break;
        case 0x04: // RTR
            if (value != 0) {   
                state->pending_rough_can_frame.rtr = 1;
            } else {
                state->pending_rough_can_frame.rtr = 0;
            }
            state->pending_rough_can_frame.rtr_ready = true;
            break;
        case 0x08: // DLC
            state->pending_rough_can_frame.dlc = (uint8_t)value;
            state->pending_rough_can_frame.dlc_ready = true;
            break;
        case 0x0c: // Data-low
            state->pending_rough_can_frame.data[0] = (uint8_t)(value & 0xFF); 
            state->pending_rough_can_frame.data[1] = (uint8_t)((value >> 8) & 0xFF);
            state->pending_rough_can_frame.data[2] = (uint8_t)((value >> 16) & 0xFF);
            state->pending_rough_can_frame.data[3] = (uint8_t)((value >> 24) & 0xFF);
            state->pending_rough_can_frame.data_low_ready = true;
            break;
        case 0x10: // Data-high
            state->pending_rough_can_frame.data[4] = (uint8_t)(value & 0xFF);
            state->pending_rough_can_frame.data[5] = (uint8_t)((value >> 8) & 0xFF);
            state->pending_rough_can_frame.data[6] = (uint8_t)((value >> 16) & 0xFF);
            state->pending_rough_can_frame.data[7] = (uint8_t)((value >> 24) & 0xFF);
            state->pending_rough_can_frame.data_high_ready = true;
            break;
        case 0x14: // Command
            switch(value) {
                case 0: // Abort frame
                    clear_rough_can_frame(state);
                    break;
                case 1: // Send frame
                    send_rough_can_frame(state);
            }
    }

    qemu_mutex_unlock(&state->lock);
}

static const MemoryRegionOps virtual_can_controller_ops = {
    .read = virtual_can_controller_read,
    .write = virtual_can_controller_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void virtual_can_controller_realize(DeviceState *dev, Error **errp)
{
    VirtualCANControllerState *state = VIRTUAL_CAN_CONTROLLER(dev);

    state->base_addr = 0x40006400; // Default CAN1 base address

    qemu_mutex_init(&state->lock);

    memory_region_init_io(
        &state->mmio,
        OBJECT(state),
        &virtual_can_controller_ops,
        state,
        TYPE_VIRTUAL_CAN_CONTROLLER,
        0x1000 // MMIO region size (TODO: resize it as needed).
    );

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, state->base_addr);

    // Init CAN bus state
    state->can_bus_state = ERROR_ACTIVE;
    state->tx_error_count = 0;
    state->rx_error_count = 0;
    state->tx_queue_length = 0;
    state->rx_queue_length = 0;
    state->tx_bit_cursor = 0;
    state->rx_bit_cursor = 0;
    state->consecutive_bit_count = 0;
    state->tx_in_progress = false;
    state->rx_in_progress = false;

    // Connect to our virtual CAN bus
    connect_to_virtual_can_bus(state);

    // Init TX Timer
    state->tx_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *)tx_timer_callback, state);
    timer_mod(state->tx_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);

    // Init RX IRQ
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &state->rx_irq);
}

static void virtual_can_controller_instance_init(Object *obj)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);
    VirtualCANControllerState *state = VIRTUAL_CAN_CONTROLLER(obj);

    sysbus_init_mmio(
        dev, 
        &state->mmio
    );

    sysbus_init_irq(dev, &state->rx_ir);
}

static void virtual_can_controller_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(klass);

    dev_class->realize = virtual_can_controller_realize;

    dev_class->user_creatable = true;
}

static const TypeInfo virtual_can_controller_info = {
    .name          = TYPE_VIRTUAL_CAN_CONTROLLER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .class_init    = virtual_can_controller_class_init,
    .instance_init = virtual_can_controller_instance_init,
    .instance_size = sizeof(VirtualCANControllerState),
};

static void virtual_can_controller_register_types(void)
{
    type_register_static(&virtual_can_controller_info);
}

type_init(virtual_can_controller_register_types);

///////////////////////////////////////////////////////////////////////////////
// CAN BUS UTILITIES
// Note: these functions are called with the state lock held, so we can safely
// access and modify the CAN bus state and the pending rough CAN frame without
// additional synchronization mechanism required. 
///////////////////////////////////////////////////////////////////////////////

static void clear_rough_can_frame(VirtualCANControllerState *state) {
    state->pending_rough_can_frame.id_ready = false;
    state->pending_rough_can_frame.rtr_ready = false;
    state->pending_rough_can_frame.dlc_ready = false;
    state->pending_rough_can_frame.data_low_ready = false;
    state->pending_rough_can_frame.data_high_ready = false;
}

static void send_rough_can_frame(VirtualCANControllerState *state) {
    if (state->pending_rough_can_frame.id_ready &&
        state->pending_rough_can_frame.rtr_ready &&
        state->pending_rough_can_frame.dlc_ready &&
        state->pending_rough_can_frame.data_low_ready &&
        state->pending_rough_can_frame.data_high_ready) {
            build_can_frame(state);
        }
    // clear pending rough CAN frame regardless of whether it was valid or not,
    // to avoid sending stale data in the next transmissions
    clear_rough_can_frame(state);
}

static void build_can_frame(VirtualCANControllerState *state) {
    CANFrame frame;

    // SoF
    frame.bits[0] = 0; 
    // ID
    for (int i = 0; i < 11; i++) {
        frame.bits[1 +i] = (state->pending_rough_can_frame.id >> (10 - i)) & 1;
    }
    // RTR
    frame.bits[12] = state->pending_rough_can_frame.rtr;
    // Control 
    frame.bits[13] = 0; // IDE (ignored)
    frame.bits[14] = 0; // r0 (ignored)
    // DLC
    for (int i = 0; i < 4; i++) {
        frame.bits[15 +i] = (state->pending_rough_can_frame.dlc >> (3 - i)) & 1;
    }
    // Data
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            frame.bits[19 + i*8 + j] = (state->pending_rough_can_frame.data[i] >> (7 - j)) & 1;
        }
    }
    // CRC (ignored)
    for (int i = 0; i < 15; i++) {
        frame.bits[83 + i] = 0;
    }
    // ACK (ignored)
    frame.bits[98] = 0; // ACK slot
    frame.bits[99] = 1; // ACK delimiter

    // Apply bit stuffing (frame length may change)
    apply_bit_stuffing(&frame);

    // EoF
    for (int i = 0; i < 7; i++) {
        frame.bits[frame.length +i] = 1;
    }
    frame.length += 7;
    append_can_frame_to_tx_queue(state, &frame);
}

static void apply_bit_stuffing(CANFrame *frame) {
    uint8_t stuffed_bits[MAX_CAN_BITS];
    stuffed_bits[0] = frame->bits[0];
    int8_t consecutive_count = 1;
    int16_t current_stuffed_bits = 1;

    for (int i = 1; i < frame->length; i++) {
        uint8_t current_bit = frame->bits[i];
        stuffed_bits[current_stuffed_bits] = current_bit;
        current_stuffed_bits++;

        if (current_bit == stuffed_bits[current_stuffed_bits - 2]) {
            // stuffed_bits[current_stuffed_bits -1] = current_bit;
            // stuffed_bits[current_stuffed_bits -2] = previous_bit;
            consecutive_count++;
        } else {
            consecutive_count = 1;
        }

        if (consecutive_count == 5) {
            // insert opposite bit
            stuffed_bits[current_stuffed_bits] = !current_bit;
            current_stuffed_bits++;
            consecutive_count = 1;
        }
    }
    // replace with stuffed bits
    memcpy(frame->bits, stuffed_bits, current_stuffed_bits);
    frame->length = current_stuffed_bits;
}

static int8_t apply_bit_unstaffing(uint8_t rx_bit, VirtualCANControllerState* state) {
    /* 
        returns:
        -1 -> error in bit unstaffing
         0 -> correct bit unstaffing
         1 -> correct bit unstaffing: rx stuffed bit (to be removed from CANFrame)
    */
    if (state->rx_in_progress) {
        CANFrame rx_frame = state->rx_queue[state->rx_queue_length];
        // SoF
        if (rx_frame.length == 0) {
            state->consecutive_bit_count++;
            return 0;
        }
        // check if it is a stuffed bit and if it is right
        if (state->consecutive_bit_count == 5) {
            if (rx_frame.bits[rx_frame.length -1] != rx_bit) {
                return 1;
            } else {
                apply_rx_error(state);
                return -1;
            }
        } 

        if (rx_frame.bits[rx_frame.length] == rx_bit) {
            state->consecutive_bit_count++;
        }
    }
    return 0;
}

static void apply_IFS(VirtualCANControllerState *state) {
    // only who transmitted a frame transmit IFS, others just observe them
    // before transmitting the next frame
    state->IFS_counter = 3;
    if (state->can_bus_state == ERROR_PASSIVE) {
        state->IFS_counter += 8
    }
}

static void append_can_frame_to_tx_queue(VirtualCANControllerState *state, CANFrame *frame) 
{
    if (state->tx_queue_length >= 10) {
        error_report("virtual-can-controller: TX queue overflow, dropping CAN frame.");
        return;
    }
    state->tx_queue[state->tx_queue_length] = *frame;
    state->tx_queue_length++;
}

static void check_transmission(uint8_t bit_received, VirtualCANControllerState* state) 
{
    if (state->tx_in_progress && state->rx_in_progress) {
        if (bit_received == state->transmitted_bit) {
            return;
        } else {
            if (state->tx_bit_cursor < 1 || state->tx_bit_cursor > 11) {
                // not arbitration mode: apply tx error
                apply_tx_error(state);
            }
            // reset can frame transmission
            state->tx_in_progress = false;
            state->tx_bit_cursor = 0;
        }
    }
}

static bool check_end_of_can_frame(VirtualCANControllerState* state) {
    static const uint8_t eof_pattern[7] = {1, 1, 1, 1, 1, 1, 1};
    if (state->rx_queue[state->rx_queue_length].length >= 7) {
        uint8_t *last7 = &state->rx_queue[state->rx_queue_length].bits[state->rx_bit_cursor -7];
        return memcmp(last7, eof_pattern, 7) == 0;
    }
    return false;
}

static void append_rx_bit_to_can_frame(uint8_t bit_received, VirtualCANController* state)
{
    if (state->rx_in_progress) {
        state->rx_queue[state->rx_queue_length].bits[state->rx_bit_cursor] = bit_received;
        state->rx_bit_cursor++;
        if (check_end_of_can_frame(state)) {
            // received a complete CAN frame
            state->rx_queue_length++;
            state->rx_bit_cursor = 0;
            state->rx_in_progress = false;
            apply_successfull_rx(state);
            if (state->tx_in_progress) {
                apply_IFS(state);
                apply_successfull_tx(state);
                state->tx_in_progress = false;
                // remove transmitted frame from tx queue
                state->tx_bit_cursor = 0;
                uint8_t frames_to_shift = state->tx_queue_length -1;
                if (frames_to_shift > 0) {
                    memmove(&state->tx_queue[0],
                            &state->tx_queue[1],
                            sizeof(CANFrame) * frames_to_shift);
                }
                state->tx_queue_length--;
            }
            // rise interrupt to notify the ECU
            qemu_irq_raise(state->rx_irq);
        }
    }
}

static void apply_tx_error(VirtualCANControllerState *state) 
{
    if (state->can_bus_state == BUS_OFF) {
        // once in BUS_OFF, do not allow state changes
        return;
    }
    state->tx_error_count += 8;
    update_can_bus_state(state);
}

static void apply_rx_error(VirtualCANControllerState *state) 
{
    if (state->can_bus_state == BUS_OFF) {
        // once in BUS_OFF, do not allow state changes
        return;
    } 
    if (state->rx_error_count < UINT8_MAX) {
        state->rx_error_count++;
        update_can_bus_state(state);
    }
}

static void apply_successfull_tx(VirtualCANControllerState *state) 
{
    if (state->can_bus_state == BUS_OFF) {
        // once in BUS_OFF, do not allow state changes
        return;
    }

    if (state->tx_error_count > 0) {
        state->tx_error_count--;
        update_can_bus_state(state);
    }
}

static void apply_successfull_rx(VirtualCANControllerState *state) {
    if (state->can_bus_state == BUS_OFF) {
        // once in BUS_OFF, do not allow state changes
        return;
    }

    if (state->rx_error_count > 0) {
        state->rx_error_count--;
        update_can_bus_state(state);
    }
}

static void update_can_bus_state(VirtualCANControllerState *state) 
{
    if (state->can_bus_state == BUS_OFF) {    
        // once in BUS_OFF, do not allow state changes
        return;
    }
    if (state->tx_error_count > 255) {
        state->can_bus_state = BUS_OFF;
    } else if (state->tx_error_count > 127 || state->rx_error_count > 127) {
        state->can_bus_state = ERROR_PASSIVE;
    } else {
        state->can_bus_state = ERROR_ACTIVE;
    }
}

///////////////////////////////////////////////////////////////////////////////
// CAN BUS INTERACTION
// tx and rx callbacks & tcp connection init with virtual can bus
///////////////////////////////////////////////////////////////////////////////
static uint8_t connect_to_virtual_can_bus(VirtualCANControllerState *state) 
{   
    struct sockaddr_in serv_addr;

    state->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (state->socket_fd < 0) {
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(state->server_port);
    if (inet_pton(AF_INET, state->server_adress, &serv_addr.sin_addr) <= 0) {
        return -1;
    }

    if (connect(state->socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        close(state->socket_fd);
        state->socket_fd = -1;
        return -1;
    }

    qemu_set_fd_handler(state->socket_fd, (IOHandler *)rx_callback, NULL, state);

    return 0;
}

static void tx_timer_callback(VirtualCANControllerState *state) {
    qemu_mutex_lock(&state->lock);
    // set as soon as possible next timer to avoid time drifts
    timer_mod(state->tx_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1); 

    // Check if we are still in the IFS of the previous tx
    if (state->IFS_counter > 0) {
        if (send(state->socket_fd, &(uint8_t){1}, 1, MSG_NOSIGNAL) < 0) {
            error_report("virtual-can-controller emulation error: failed to send IFS bit");
        }
        state->IFS_counter--;
        goto unlock;
    }

    // Check if we have a CAN frame to transmit
    if (state->tx_queue_length > 0) {
        // Check if we are at the end of the current frame 
        if (state->tx_in_progress && state->tx_bit_cursor == (state->tx_queue[0]).length -1) {
            goto unlock;
        }
        // Transmit the next bit of the current CAN frame in the queue
        state->tx_in_progress = true;
        uint8_t bit_to_send = (state->tx_queue[0]).bits[state->tx_bit_cursor];
        if (send(state->socket_fd, &bit_to_send, 1, MSG_NOSIGNAL) < 0) {
            error_report("virtual-can-controller emulation error: failed to send CAN bit");
        }
        state->tx_bit_cursor++;
        state->transmitted_bit = bit_to_send;             
    }

    unlock:
    qemu_mutex_unlock(&state->lock);
}

static void rx_callback(VirtualCANControllerState *state) {
    uint8_t bit_received;
    
    qemu_mutex_lock(&state->lock);

    ssize_t ret = recv(state->socket_fd, &bit_received, 1, 0);
    if (ret <= 0) {
        qemu_set_fd_handler(state->socket_fd, NULL, NULL, NULL);
        close(state->socket_fd);
        state->socket_fd = -1;
        goto unlock;
    }

    // check if a rx is in progress
    if (state->rx_in_progress == false && bit_received == 0) {
        state->rx_in_progress = true;
    }

    check_transmission(bit_received, state);
    if (apply_bit_unstaffing(bit_received, state) == 0) {
        append_rx_bit_to_can_frame(bit_received, state);
    }

    unlock:
    qemu_mutex_unlock(state->lock);
}