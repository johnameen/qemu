#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/misc/msp430_svs.h"

/*******************************************************************
 * Prototypes
 ******************************************************************/
static void memory_write(void* opaque, hwaddr offset, uint64_t val, unsigned size);
static uint64_t memory_read(void *opaque, hwaddr offset, unsigned size);

/*******************************************************************
 * Global Structures
 ******************************************************************/
static const MemoryRegionOps mmio_ops = {
    .read = memory_read,
    .write = memory_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


/*******************************************************************
 * Helper/Misc Functions
 ******************************************************************/
static void handle_svsctrl(msp430_svs_device *device, uint8_t value)
{
    device->vldx = (value & 0xF0) >> 4;
    device->poron = (device->SVSCTL & 0x08) ? true : false;
    if ((value & 0x1) && (device->SVSCTL & 0x1))
        device->SVSCTL &= ~(0x1);
    device->SVSCTL &= (0x7);
    device->SVSCTL |= (value & 0xF8);
    return;
}

static void memory_write(void* opaque, hwaddr offset, 
                         uint64_t val, unsigned size)
{
    /* Set the new bits within the SVS control register */
    msp430_svs_device *device = (msp430_svs_device *)opaque;
    if (offset != 0)
        return;
    handle_svsctrl(device, val);
    return;
}

static uint64_t memory_read(void *opaque, hwaddr offset, unsigned size)
{
    msp430_svs_device *device = MSP430_SVS(opaque);
    if (offset != 0)
        return 0;
    return device->SVSCTL;
}


/*******************************************************************
 * Object Callback Functions
 ******************************************************************/
static void msp430_svs_reset(DeviceState *dev)
{
    msp430_svs_device *device = MSP430_SVS(dev);
    handle_svsctrl(device, 0x00);
}

/**
 * @brief Initialization Function for the SVS Family 1 device.
 * @details Initialization Function for the SVS Family 1 device.
 * This  function is responsible for initializing the object device that 
 * is instantiated as a SVS Family 1 device.
 * 
 * @param dev SysBusDevice that will be initialized as a SVS device
 * Family 1 device.
 * @return If initialization was successful or not (0 for success)
 */
static void msp430_svs_init(Object *obj)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    msp430_svs_device *device = MSP430_SVS(dev);
    memory_region_init_io(&device->region, OBJECT(dev),  &mmio_ops,
                          dev, "flash-cfg", 0x1);
    sysbus_init_mmio(dev, &device->region);

    /* Reset the Device so we are able to have an initial state
     * for this device. */
    msp430_svs_reset(DEVICE(dev));
}

/*******************************************************************
 * Registration and Initialization Functions (Class and Object)
 ******************************************************************/
/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param klass [description]
 * @param data [description]
 */
static void msp430_svs_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = msp430_svs_reset;
}

/**
 * @brief Type registration for MSP430 Supply voltage supervisor hardware for family 1.
 * @details Type registration for MSP430 Supply voltage supervisor hardware for
 * family 1. This device controls the supervisor monitor of the supply voltage.
 */
static const TypeInfo msp430_svs_info = {
    .name          = TYPE_MSP430_SVS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(msp430_svs_device),
    .class_init    = msp430_svs_class_init,
    .instance_init = msp430_svs_init
};

/**
 * @brief Registers types declared within this hardware device.
 * @details Register types declared within this hardware device file.
 */
static void msp430_svs_register_types(void)
{
    type_register_static(&msp430_svs_info);
}

// Define the function that will register new types into 
// QEMU with the hardware declarations within this file.
type_init(msp430_svs_register_types)
