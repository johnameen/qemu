#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"

#define TYPE_MSP430_WATCHDOG "msp430_watchdog"
#define MSP430_WATCHDOG_WORDSIZE (1)
#define MSP430_WATCHDOG(obj) OBJECT_CHECK(msp430_watchdog_device, (obj), TYPE_MSP430_WATCHDOG)

typedef struct 
{
    // Public Fields
    SysBusDevice parent_obj;

    // Private Fields
    MemoryRegion region;
    uint16_t WDTCTL;
    uint8_t wdthold;
    uint8_t wdtssel;
    uint8_t wdtisx;
    qemu_irq pucRequest;
} msp430_watchdog_device;

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
static void handle_wdtctrl(msp430_watchdog_device *device, uint16_t value)
{
    // Check for the writing key before replacing the value with the 
    // read key value.
    if ((value & 0xFF00) != 0x5A)
        return;
    value &= 0x00F7;

    // Store what looks to be important fields so we can implement
    // more logic later on easier.
    device->wdthold = (value & 0x80);
    device->wdtssel = (value & 0x4);
    device->wdtisx = (value & 0x3);

    // Save the register value to the WDTCTL value within
    // the device object.
    device->WDTCTL &= 0xFF08;
    device->WDTCTL |= value ;

    return;
}

static void memory_write(void* opaque, hwaddr offset, 
                         uint64_t val, unsigned size)
{
    /* Set the new bits within the SVS control register */
    msp430_watchdog_device *device = MSP430_WATCHDOG(opaque);
    if (offset)
        return;
    handle_wdtctrl(device, val);
    return;
}

static uint64_t memory_read(void *opaque, hwaddr offset, unsigned size)
{
    msp430_watchdog_device *device = MSP430_WATCHDOG(opaque);
    if (offset)
        return 0;
    return device->WDTCTL;
}


/*******************************************************************
 * Object Callback Functions
 ******************************************************************/
/**
 * @brief Resets the MSP430 device.
 * @details Resets the MSP430 device. Handles resetting the MSP430
 * peripheral into a known good initial state, such as power up.
 * 
 * @param dev MSP430 Device object.
 */
static void msp430_watchdog_reset(DeviceState *dev)
{
    msp430_watchdog_device *device = MSP430_WATCHDOG(dev);
    device->WDTCTL = 0x6900;
    handle_wdtctrl(device, 0x5A00);
}

/**
 * @brief Interrupt handler for input interrupts
 * @details Interrupt handler for input interrupts
 * 
 * @param opaque Data Structure
 * @param irq Interrupt Number
 * @param req Set/Lower IRQ line that is being changed.
 */
static void handle_irq_reset(void *opaque, int irq, int req)
{
    // Handle the case where the device is being reset.
    fprintf(stdout, "Watchdog Timer Interrupt for Reset");
    msp430_watchdog_reset(DEVICE(opaque));
}

/**
 * @brief Initialization Function for the MSP430 device.
 * @details Initialization Function for the MSP430 device.
 * This function is responsible for initializing the object device that 
 * is instantiated as a MSP430 device.
 * 
 * @param dev SysBusDevice that will be initialized as a MSP430 device.
 * @return If initialization was successful or not (0 for success)
 */
static int msp430_watchdog_init(SysBusDevice *dev)
{
    msp430_watchdog_device *device = MSP430_WATCHDOG(dev);
    memory_region_init_io(&device->region, OBJECT(dev),  &mmio_ops,
                          dev, "watchdog", 0x1);
    sysbus_init_mmio(dev, &device->region);
    //sysbus_init_irq(device->pucRequest, 0);
    qdev_init_gpio_in(DEVICE(dev), handle_irq_reset, 1);

    /* Reset the Device so we are able to have an initial state
     * for this device. */
    msp430_watchdog_reset(DEVICE(dev));

    return 0;
}

/*******************************************************************
 * Registration and Initialization Functions (Class and Object)
 ******************************************************************/
/**
 * @brief Initializes class object for MSP430 device.
 * @details Initializes class object for MSP430 device. Specifies the
 * required interface functions for an object to be created.
 * 
 * @param klass Class Object being instantiated
 * @param data Data?
 */
static void msp430_watchdog_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    sdc->init = msp430_watchdog_init;
    dc->reset = msp430_watchdog_reset;
}

/**
 * @brief Type registration for MSP430 hardware for family 1.
 * @details Type registration for MSP430 hardware for family 1.
 */
static const TypeInfo msp430_watchdog_info = {
    .name          = TYPE_MSP430_WATCHDOG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(msp430_watchdog_device),
    .class_init    = msp430_watchdog_class_init,
};

/**
 * @brief Registers types declared within this hardware device.
 * @details Register types declared within this hardware device file.
 */
static void msp430_watchdog_register_types(void)
{
    type_register_static(&msp430_watchdog_info);
}

// Define the function that will register new types into 
// QEMU with the hardware declarations within this file.
type_init(msp430_watchdog_register_types)
