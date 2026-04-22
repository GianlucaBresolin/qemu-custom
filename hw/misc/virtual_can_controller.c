#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define TYPE_VIRTUAL_CAN_CONTROLLER "virtual-can-controller"
OBJECT_DECLARE_SIMPLE_TYPE(VirtualCANControllerState, VIRTUAL_CAN_CONTROLLER)

typedef enum {
    ERROR_ACTIVE, 
    ERROR_PASSIVE,
    BUS_OFF
} CANBusState;

typedef struct VirtualCANControllerState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    uint64_t base_addr;
    QemuMutex lock;

    int socket_fd;
    char server_address[256];
    uint16_t server_port;

    // CAN BUS STATE
    CANBusState can_bus_state;
    uint16_t tx_error_count;
    uint8_t rx_error_count;
} VirtualCANControllerState;

static uint8_t connect_to_virtual_can_bus(VirtualCANControllerState *state) {
    
    struct sockaddr_in serv_addr;

    state->socket_fd = socket(AF_INET, SOCK_STREAM, 0),
    if (state->socket_fd < 0) {
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(state->server_port);
    if (inet_pton(AF_INET, state->server_ip, &serv_addr.sin_addr) <= 0) {
        return -1;
    }

    if (connect(state->sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        close(state->sock_fd);
        state->sock_fd = -1;
        return -1;
    }

    return 0;
}

static uint64_t virtual_can_controller_read(void *opaque, hwaddr offset, unsigned size)
{
    printf("virtual_can_controller_read: offset=0x%08x, size=%u\n", (uint32_t)offset, size);
    VirtualCANControllerState *state = VIRTUAL_CAN_CONTROLLER(opaque);

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

    uint8_t req[14];
    int packet_size = 6 + size;
    int ret;
    
    qemu_mutex_lock(&state->lock);

    // Build request packet: 'W' + addr(4B) + size(1B) + data(8B max)
    req[0] = 'W';
    stl_le_p(req + 1, (uint32_t)offset);
    req[5] = (uint8_t)size;

    // Add data in little-endian format
    switch (size) {
    case 1:
        req[6] = (uint8_t)value;
        break;
    case 2:
        stw_le_p(req + 6, (uint16_t)value);
        break;
    case 4: 
        stl_le_p(req + 6, (uint32_t)value);
        break;
    default:
        error_report("virtual-can-controller: invalid write size %u", size);
        goto unlock;
    }

    // Send request
    printf("Sending write request: addr=0x%08x, size=%u, value=0x%llx\n",
           (uint32_t)offset, size, (unsigned long long)value);
    ret = packet_size;
    if (ret != packet_size) {
        error_report("virtual-can-controller: failed to send write request");
    }

    // Send data to virutal CAN bus
    ssize_t sent = send(state->sock_fd, req, packet_size, 0);
    if (sent != packet_size) {
        error_report("virtual-can-controller: failed to send TCP data");
        close(state->sock_fd);
        state->sock_fd = -1;
    }

    unlock:
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

    /*
    if (!verify_backend_connected()) {
        error_setg(errp, "mmio-sockdev: backennd not connected");
        return;
    }
    */

    state->base_addr = 0x40006400; // Default CAN1 base address

    qemu_mutex_init(&state->lock);

    memory_region_init_io(
        &state->mmio,
        OBJECT(state),
        &virtual_can_controller_ops,
        state,
        TYPE_VIRTUAL_CAN_CONTROLLER,
        0x1000 // MMIO region size
    );

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, state->base_addr);

    // Init CAN bus state
    state->can_bus_state = ERROR_ACTIVE;
    state->tx_error_count = 0;
    state->rx_error_count = 0;

    // Connect to our virtual CAN bus
    connect_to_virtual_can_bus(state);
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

///////////////////////////
// CAN BUS UTILITIES
///////////////////////////
static void apply_tx_errror(VirtualCANControllerState *state) {
    state->tx_error_count += 8;
    update_can_bus_state(state);
}

static void apply_rx_error(VirtualCANControllerState *state) 
{
    if (state->rx_error_count < UINT8_MAX) {
        state->rx_error_count++;
    }
    update_can_bus_state(state);
}

static void apply_successfull_tx(VirtualCANControllerState *state) 
{
    if state->can_bus_state == BUS_OFF {
        // once in BUS_OFF, do not allow state changes
        return;
    }
    if (state->tx_error_count > 0) {
        state->tx_error_count--;
    }
    if (state->rx_error_count > 0) {
        state->rx_error_count--;
    }
    update_can_bus_state(state);
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