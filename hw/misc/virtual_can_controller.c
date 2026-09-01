#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/irq.h" 
#include "qemu/main-loop.h" 
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#define TYPE_VIRTUAL_CAN_CONTROLLER "virtual-can-controller"
#define VCAN_SERVER_HOST "virtual-can-bus"
#define VCAN_SERVER_PORT "8080"

OBJECT_DECLARE_SIMPLE_TYPE(VirtualCANControllerState, VIRTUAL_CAN_CONTROLLER)

#define MAX_CAN_BITS 135

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
    uint8_t arbitration_end;
    bool complete;
} CANFrame;

typedef struct VirtualCANControllerState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    uint64_t base_addr;
    QemuMutex lock;

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
    uint8_t IFS_counter;
    qemu_irq tx_error_irq;

    // RX
    CANFrame rx_queue[10];
    uint8_t rx_queue_length;
    bool rx_in_progress;
    uint8_t consecutive_bit_count; 
    uint8_t raw_bit_history;
    qemu_irq rx_irq; 

    // RX READ
    bool rx_id_read;
    bool rx_rtr_read;
    bool rx_dlc_read;
    bool rx_data_low_read;
    bool rx_data_high_read;

    // ERROR HANDLING
    bool error_to_transmit;
    uint8_t error_bit;
    uint8_t error_cursor;
    bool active_error_flag_echo;
    uint8_t active_error_flag_echo_counter;
    bool count_error_delimiter;
    uint8_t error_delimiter_counter;

    // TCP CONNECTION
    int socket_fd;
    char server_address[256];
    uint16_t server_port;
} VirtualCANControllerState;

///////////////////////////////////////////////////////////////////////////////
// CAN BUS UTILITIES
// Note: these functions are called with the state lock held, so we can safely
// access and modify the CAN bus state and the pending rough CAN frame without
// additional synchronization mechanism required. 
///////////////////////////////////////////////////////////////////////////////

static void update_can_bus_state(VirtualCANControllerState *state) 
{
    if (state->can_bus_state == BUS_OFF) {    
        // once in BUS_OFF, do not allow state changes
        return;
    }
    if (state->tx_error_count > 255) {
        printf("[DEBUG] BUS OFF MODE: TEC=%d, REC=%d\n", state->tx_error_count, state->rx_error_count);
        fflush(stdout);

        state->can_bus_state = BUS_OFF;
    } else if (state->tx_error_count > 127 || state->rx_error_count > 127) {
        printf("[DEBUG] ERROR PASSIVE MODE: TEC=%d, REC=%d\n", state->tx_error_count, state->rx_error_count);
        fflush(stdout);

        state->can_bus_state = ERROR_PASSIVE;
    } else {
        printf("[DEBUG] ERROR ACTIVE MODE: TEC=%d, REC=%d\n", state->tx_error_count, state->rx_error_count);           
        fflush(stdout);
        state->can_bus_state = ERROR_ACTIVE;
    }
}

static void apply_tx_error(VirtualCANControllerState *state) 
{
    if (state->can_bus_state == BUS_OFF) {
        // once in BUS_OFF, do not allow state changes
        return;
    }

    switch(state->can_bus_state) {
        case ERROR_ACTIVE:
            state->error_bit = 0; 
            state->active_error_flag_echo = true;
            break;
        case ERROR_PASSIVE:
            state->error_bit = 1;
            break;
        case BUS_OFF: 
            return;
    }
    state->tx_error_count += 8;
    update_can_bus_state(state);
    state->error_to_transmit = true;
    qemu_irq_raise(state->tx_error_irq);
}

static void apply_rx_error(VirtualCANControllerState *state) 
{
    if (state->can_bus_state == BUS_OFF) {
        // once in BUS_OFF, do not allow state changes
        return;
    } 

    switch(state->can_bus_state) {
        case ERROR_ACTIVE:
            state->error_bit = 0; 
            state->active_error_flag_echo = true;
            break;
        case ERROR_PASSIVE:
            state->error_bit = 1;
            break;
        case BUS_OFF: 
            return;
    }

    if (state->rx_error_count < UINT8_MAX) {
        state->rx_error_count++;
        update_can_bus_state(state);
    }
    state->error_to_transmit = true;
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

static void clear_rough_can_frame(VirtualCANControllerState *state) {
    state->pending_rough_can_frame.id_ready = false;
    state->pending_rough_can_frame.rtr_ready = false;
    state->pending_rough_can_frame.dlc_ready = false;
    state->pending_rough_can_frame.data_low_ready = false;
    state->pending_rough_can_frame.data_high_ready = false;
}

static void append_can_frame_to_tx_queue(VirtualCANControllerState *state, CANFrame *frame) 
{
    if (state->tx_queue_length >= 10) {
        error_report("virtual-can-controller: TX queue overflow, dropping CAN frame.");
        return;
    }
    state->tx_queue[state->tx_queue_length] = *frame;
    
    CANFrame *f = &state->tx_queue[0];
    printf("tx_queue: stuffed bits=");
    for (int i = 0; i < f->length; i++) {
        printf("%d", f->bits[i]);
    }
    printf("\n");
    fflush(stdout);

    state->tx_queue_length++;
}

static void apply_bit_stuffing(CANFrame *frame, uint8_t stuffing_end) {
    uint8_t stuffed_bits[MAX_CAN_BITS];
    stuffed_bits[0] = frame->bits[0];
    int8_t consecutive_count = 1;
    int16_t current_stuffed_bits = 1;

    for (int i = 1; i < stuffing_end; i++) {
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

        if (i == 11) {
            frame->arbitration_end = current_stuffed_bits - 1;
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
        CANFrame *rx_frame = &state->rx_queue[state->rx_queue_length -1];
        if (rx_frame->length < 98) {
            // SoF
            if (rx_frame->length == 0) {
                state->consecutive_bit_count = 1;
                return 0;
            }
            // check if it is a stuffed bit and if it is right
            if (state->consecutive_bit_count == 5) {
                if (rx_frame->bits[rx_frame->length -1] != rx_bit) {
                    // reset consecutive bit count
                    state->consecutive_bit_count = 0;
                    return 1;
                } else {
                    printf("[DEBUG] error in bit unstaffing\n");            
                    fflush(stdout);

                    apply_rx_error(state);
                    // reset can frame reception
                    state->rx_in_progress = false;
                    state->rx_queue_length -= 1;
                    return -1;
                }
            } 

            if (rx_frame->bits[rx_frame->length -1] == rx_bit) {
                state->consecutive_bit_count++;
            } else {
                state->consecutive_bit_count = 1;
            }
        }
    }
    return 0;
}

static void build_can_frame(VirtualCANControllerState *state) {
    CANFrame frame;
    frame.length = 0;
    // SoF
    frame.bits[frame.length++] = 0; 
    
    // ID
    for (int i = 0; i < 11; i++) {
        frame.bits[frame.length++] = (state->pending_rough_can_frame.id >> (10 - i)) & 1;
    }
    // RTR
    frame.bits[frame.length++] = state->pending_rough_can_frame.rtr;
    // Control 
    frame.bits[frame.length++] = 0; // IDE (ignored)
    frame.bits[frame.length++] = 0; // r0 (ignored)
    // DLC
    for (int i = 0; i < 4; i++) {
        frame.bits[frame.length++] = (state->pending_rough_can_frame.dlc >> (3 - i)) & 1;
    }
    // Data
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            frame.bits[frame.length + i*8 + j] = (state->pending_rough_can_frame.data[i] >> (7 - j)) & 1;
        }
    }
    frame.length += 64;

    // CRC (ignored)
    for (int i = 0; i < 15; i++) {
        frame.bits[frame.length++] = 0;
    }
    // Apply bit stuffing (frame length may change)
    apply_bit_stuffing(&frame, frame.length);

    // ACK (ignored)
    frame.bits[frame.length++] = 0; // ACK slot
    frame.bits[frame.length++] = 1; // ACK delimiter

    // EoF
    for (int i = 0; i < 7; i++) {
        frame.bits[frame.length++] = 1;
    }

    append_can_frame_to_tx_queue(state, &frame);
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

static void check_transmission(uint8_t bit_received, VirtualCANControllerState* state) 
{
    if (state->tx_in_progress && state->rx_in_progress) {
        if (bit_received == state->transmitted_bit) {
            state->tx_bit_cursor++;
            return;
        } else {
            if (state->tx_bit_cursor < 1 || state->tx_bit_cursor > state->tx_queue[0].arbitration_end) {
                // not arbitration mode: apply tx error
                printf("[DEBUG] tx error\n");            
                fflush(stdout);

                apply_tx_error(state);

                // reset can frame transmission
                state->tx_in_progress = false;
                state->tx_bit_cursor = 0;
                // reset can frame reception
                state->rx_in_progress = false;
                state->rx_queue_length -= 1;
            } else {
                // arbitration mode: keep rx can frame
                // reset can frame transmission
                printf("[DEBUG] lost arbitration\n");            
                fflush(stdout);

                state->tx_in_progress = false;
                state->tx_bit_cursor = 0;
            }   
        }
        return;
    }

    if (state->error_to_transmit) {
        if (bit_received == state->transmitted_bit) {
            state->error_cursor += 1;
            if (state->error_cursor == 6) {
                // tx of error flag finished
                state->error_to_transmit = false;
                state->error_cursor = 0;
                // set error delimiter 
                state->count_error_delimiter = true;
                state->error_delimiter_counter = 8;
            }
        } else {
            // ERROR_PASSIVE: not able to tx error flag (passive), retry to retransmit 
            state->error_cursor = 0;
        }
        return;
    }

    if (state->count_error_delimiter) {
        if (bit_received == 0 && state->active_error_flag_echo) {
            state->active_error_flag_echo_counter += 1;
            if (state->active_error_flag_echo_counter == 5) {
                state->active_error_flag_echo = false;
                state->active_error_flag_echo_counter = 0;
            }
            return;
        }
        if (bit_received == 1) {
            if (state->active_error_flag_echo) {
                // stop active error echo
                state->active_error_flag_echo = false;
                state->active_error_flag_echo_counter = 0;
            } 

            state->error_delimiter_counter -= 1;
            if (state->error_delimiter_counter == 0) {
                state->count_error_delimiter = false;
                state->IFS_counter = 3;
            }
        } else {
            // error delimiter form error
            printf("[DEBUG] error-delimiter form error\n");            
            fflush(stdout);

            state->count_error_delimiter = false;
            state->error_delimiter_counter = 0;
            apply_tx_error(state);
        }
        return;
    }

    if (state->IFS_counter > 0) {
        if (bit_received == 1) {
            state->IFS_counter -= 1;
        } else {
            // ifs delimiter form error
            printf("[DEBUG] IFS-delimiter form error\n");            
            fflush(stdout);

            apply_tx_error(state);
        }
    }
}

static bool check_end_of_can_frame(VirtualCANControllerState* state) {
    // check if 7 consecutive '1' and rx in progress
    return state->rx_in_progress && state->raw_bit_history == 0x7F;
}

static void append_rx_bit_to_can_frame(uint8_t bit_received, VirtualCANControllerState* state)
{
    if (state->rx_in_progress) {
        CANFrame *can_frame = &state->rx_queue[state->rx_queue_length -1];
        can_frame->bits[can_frame->length] = bit_received;
        can_frame->length++;
        if (check_end_of_can_frame(state)) {
            // received a complete CAN frame
            state->rx_in_progress = false;
            state->IFS_counter = 3;
            apply_successfull_rx(state);
            if (state->tx_in_progress) {
                apply_successfull_tx(state);
                state->tx_in_progress = false;
                if (state->can_bus_state == ERROR_PASSIVE) {
                    // suspend transmission rule
                    state->IFS_counter += 8;
                }
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
            
            CANFrame *f = &state->rx_queue[0];
            printf("rx_queue: bits=");
            for (int i = 0; i < f->length; i++) {
                printf("%d", f->bits[i]);
            }
            printf("\n");
            fflush(stdout);

            can_frame->complete = true;

            // rise interrupt to notify the ECU
            qemu_irq_raise(state->rx_irq);
        }
    }
}

static void handle_error_tx(VirtualCANControllerState *state) {
    printf("[DEBUG] tx (Error Flag): %u\n", state->error_bit);            
    fflush(stdout);

    if (send(state->socket_fd, &(state->error_bit), 1, MSG_NOSIGNAL) < 0) {
        error_report("virtual-can-controller emulation error: failed to send error bit");
    }
    state->transmitted_bit = state->error_bit;
    return;
}

static uint8_t check_complete_can_frame_reading(VirtualCANControllerState *state) {
    uint8_t successful_reading = 0;
    if (state->rx_id_read &&
        state->rx_rtr_read &&
        state->rx_dlc_read &&
        state->rx_data_low_read &&
        state->rx_data_high_read) {
            // all the CAN frame fields have been read
            successful_reading = 1;
            state->rx_queue_length--;
            // pop the head of the rx queue
            if (state->rx_queue_length > 0) {
                uint8_t frames_to_shift = state->rx_queue_length;
                memmove(&state->rx_queue[0],
                        &state->rx_queue[1],
                        sizeof(CANFrame) * frames_to_shift);
            }
            
    }
    // reset pending rx frame reading state for the next frame
    state->rx_id_read = false;
    state->rx_rtr_read = false;
    state->rx_dlc_read = false;
    state->rx_data_low_read = false;
    state->rx_data_high_read = false;
    return successful_reading;
}

///////////////////////////////////////////////////////////////////////////////
// CAN BUS INTERACTION
// tx and rx callbacks & tcp connection init with virtual can bus
///////////////////////////////////////////////////////////////////////////////
static void tx_callback(VirtualCANControllerState *state) {
    // BUS_OFF state: do not tx
    if (state->can_bus_state == BUS_OFF) {
        // idle bus default
        if (send(state->socket_fd, &(uint8_t){1}, 1, MSG_NOSIGNAL) < 0) {
            error_report("virtual-can-controller emulation error: failed to send idle bit");
        }
        return;
    }

    // Check if we have error to tx
    if (state->error_to_transmit) {
        handle_error_tx(state);
        return;
    }

    // Check if we have error delimiter active
    if (state->count_error_delimiter) {
        printf("[DEBUG] tx (Error Delimiter): 1\n");            
        fflush(stdout);

        if (send(state->socket_fd, &(uint8_t){1}, 1, MSG_NOSIGNAL) < 0) {
            error_report("virtual-can-controller emulation error: failed to send error delimiter bit");
        }
        return;
    }

    // Check if we are still in the IFS of the previous tx
    if (state->IFS_counter > 0) {
        printf("[DEBUG] tx (IFS): 1 \n");            
        fflush(stdout);

        if (send(state->socket_fd, &(uint8_t){1}, 1, MSG_NOSIGNAL) < 0) {
            error_report("virtual-can-controller emulation error: failed to send IFS bit");
        }
        return;
    }

    // we have an rx in progress but not a tx in progress
    if (state->rx_in_progress && !state->tx_in_progress) {
        // idle bus default
        if (send(state->socket_fd, &(uint8_t){1}, 1, MSG_NOSIGNAL) < 0) {
            error_report("virtual-can-controller emulation error: failed to send idle bit");
        }
        return;
    }

    // Check if we have a CAN frame to transmit
    if (state->tx_queue_length > 0) {
        // Check if we are at the end of the current frame 
        if (state->tx_in_progress && state->tx_bit_cursor == (state->tx_queue[0]).length) {
            return;
        }
        // Transmit the next bit of the current CAN frame in the queue
        state->tx_in_progress = true;
        uint8_t bit_to_send = (state->tx_queue[0]).bits[state->tx_bit_cursor];

        printf("[DEBUG] tx: %u\n", bit_to_send);            
        fflush(stdout);

        if (send(state->socket_fd, &bit_to_send, 1, MSG_NOSIGNAL) < 0) {
            error_report("virtual-can-controller emulation error: failed to send CAN bit");
        }
        state->transmitted_bit = bit_to_send;    
        return;         
    }

    // nothing to transmit: (idle bus default)
    if (send(state->socket_fd, &(uint8_t){1}, 1, MSG_NOSIGNAL) < 0) {
        error_report("virtual-can-controller emulation error: failed to send idle bit");
    }
    return;
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

    if (state->can_bus_state == BUS_OFF) {
        tx_callback(state);
        goto unlock;
    }

    state->raw_bit_history = ((state->raw_bit_history << 1) | bit_received) & 0x7F;

    // check if a rx is in progress
    if (
        !state->rx_in_progress && 
        !state->error_to_transmit &&
        !state->active_error_flag_echo &&
        bit_received == 0
    ) {
        if (state->rx_queue_length >= 10) {
            // rx queue full, drop the frame and do not start rx
            error_report("virtual-can-controller: RX queue overflow, dropping incoming CAN frame.");
            goto unlock;
        }

        state->rx_in_progress = true;
        state->rx_queue[state->rx_queue_length] = (CANFrame){ .complete = false};
        state->rx_queue_length++;
    }

    check_transmission(bit_received, state);
    switch(apply_bit_unstaffing(bit_received, state)) {
        case -1:
            break;
        case 0:
            append_rx_bit_to_can_frame(bit_received, state);
            break;
        case 1:
            // stuffing bit: skip it
            break;
    }   

    tx_callback(state);
    unlock:
    qemu_mutex_unlock(&state->lock);
}

static int8_t connect_to_virtual_can_bus(VirtualCANControllerState *state) 
{   
    struct addrinfo hints, *res;

    state->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (state->socket_fd < 0) {
        return -1;
    }

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(VCAN_SERVER_HOST, VCAN_SERVER_PORT, &hints, &res) != 0) {
        close(state->socket_fd);
        state->socket_fd = -1;
        return -1;
    }

    if (connect(state->socket_fd, res->ai_addr, res->ai_addrlen) < 0) {
        freeaddrinfo(res);
        close(state->socket_fd);
        state->socket_fd = -1;
        return -1;
    }

    freeaddrinfo(res);
    qemu_set_fd_handler(state->socket_fd, (IOHandler *)rx_callback, NULL, state);

    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// MMIO QEMU DEVICE
// read and write operations, realize and init functions
///////////////////////////////////////////////////////////////////////////////
static uint64_t virtual_can_controller_read(void *opaque, hwaddr offset, unsigned size)
{
    VirtualCANControllerState *state = VIRTUAL_CAN_CONTROLLER(opaque);
    uint64_t value = 0;

    qemu_mutex_lock(&state->lock);
    qemu_irq_lower(state->rx_irq);

    if (state->rx_queue_length == 0 || !state->rx_queue[0].complete) { 
        // rx queue empty or first rx frame still not ready
        goto unlock;
    }

    CANFrame *frame = &state->rx_queue[0];

    switch(offset) {
        case 0x00: { // ID
            uint32_t id = 0;
            for (int i = 0; i < 11; i++) {
                id = (id << 1) | frame->bits[1+i];
            }
            value = id;
            state->rx_id_read = true;
            break;
        }
        case 0x04: { // RTR
            value = frame->bits[12];
            state->rx_rtr_read = true;
            break;
        }
        case 0x08: { // DLC
            uint8_t dlc = 0;
            for (int i = 0; i < 4; i++) {
                dlc = (dlc << 1) | frame->bits[15+i];
            }
            value = dlc;
            state->rx_dlc_read = true;
            break;
        }
        case 0x0c: { // Data-low
            uint32_t data_low = 0;
            for (int i = 0; i < 4; i++) {
                uint8_t byte = 0;
                for (int j = 0; j < 8; j++) {
                    byte = (byte << 1) | frame->bits[19 + i*8 + j];
                }
                data_low |= ((uint32_t)byte << (i * 8));
            }
            value = data_low;
            state->rx_data_low_read = true;
            break;
        }
        case 0x10: { // Data-high
            uint32_t data_high = 0;
            for (int i = 0; i < 4; i++) {
                uint8_t byte = 0;
                for (int j = 0; j < 8; j++) {
                    byte = (byte << 1) | frame->bits[19 + (4 + i)*8 + j];
                }
                data_high |= ((uint32_t)byte << (i * 8));
            }
            value = data_high;
            state->rx_data_high_read = true;
            break;
        }
        case 0x14: { // Command
            value = check_complete_can_frame_reading(state);
            break;
        }
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
                    break;
            }
            break;
        case 0x18: { // Tx Error Ack
            qemu_irq_lower(state->tx_error_irq);
            break;
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
    state->consecutive_bit_count = 0;
    state->tx_in_progress = false;
    state->rx_in_progress = false;
    state->IFS_counter = 0;
    state->error_to_transmit = false;
    state->error_cursor = 0;
    state->active_error_flag_echo = false;
    state->active_error_flag_echo_counter = 0;
    state->count_error_delimiter = false;
    state->error_delimiter_counter = 0;
    state->rx_id_read = false;
    state->rx_rtr_read = false;
    state->rx_dlc_read = false;
    state->rx_data_low_read = false;
    state->rx_data_high_read = false;

    strcpy(state->server_address, "127.0.0.1");
    state->server_port = 8080;

    // Connect to our virtual CAN bus
    if (connect_to_virtual_can_bus(state) != 0) {
        error_setg(errp, "Failed to connect to Virtual CAN Bus.");
        return;
    }

    // Init IRQ
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &state->rx_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &state->tx_error_irq);
}

static void virtual_can_controller_instance_init(Object *obj)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);
    VirtualCANControllerState *state = VIRTUAL_CAN_CONTROLLER(obj);

    sysbus_init_mmio(
        dev, 
        &state->mmio
    );
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