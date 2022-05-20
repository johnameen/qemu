#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"

#define TYPE_MSP430_GPIO "msp430_gpio"
#define MSP430_GPIO_SIZE 32
#define MSP430_GPIO(obj) OBJECT_CHECK(msp430_gpio_device, (obj), TYPE_MSP430_GPIO)

typedef enum
{
    // P3x Registers
    P3IN  = 0x0,
    P3OUT = 0x1,
    P3DIR = 0x2,
    P3SEL = 0x3,

    // P4x Registers
    P4IN  = 0x4,
    P4OUT = 0x5,
    P4DIR = 0x6,
    P4SEL = 0x7,

    // P1x Registers
    P1IN  = 0x8,
    P1OUT = 0x9,
    P1DIR = 0xA,
    P1IFG = 0xB,
    P1IES = 0xC,
    P1IE  = 0xD,
    P1SEL = 0xE,

    // P2x Registers
    P2IN  = 0x10,
    P2OUT = 0x11,
    P2DIR = 0x12,
    P2IFG = 0x13,
    P2IES = 0x14,
    P2IE  = 0x15,
    P2SEL = 0x16,

    // P5x Registers
    P5IN  = 0x18,
    P5OUT = 0x19,
    P5DIR = 0x1A,
    P5SEL = 0x1B,

    // P6x Registers 
    P6IN  = 0x1C,
    P6OUT = 0x1D,
    P6DIR = 0x1E,
    P6SEL = 0x1F,
} MSP430_GPIO_OFFSET;

typedef struct
{
    uint8_t dir;
    uint8_t sel;
    uint8_t bits;
    qemu_irq pin[8];
} gpio_port;

typedef struct 
{
    // Public Fields
    SysBusDevice parent_obj;

    // Private Fields
    MemoryRegion region;
    gpio_port p3;
    gpio_port p4;
    gpio_port p1;
    gpio_port p2;
    gpio_port p5;
    gpio_port p6;
} msp430_gpio_device;

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


/**
 * @brief Handle GPIO output write. 
 * @details Handle GPIO output write. Function will check to ensure
 * the port is able to write before setting a write to that pin.
 * 
 * @param port Port structure holding the port state
 * @param val New value that is being written to PxOUT
 */
static inline void handle_gpio_output(gpio_port *port, uint64_t val)
{
    int i;
    int diff = (port->bits ^ (uint8_t)val);
    if (!diff)
        return;

    // Loop through and check to see if the bits are properly 
    // being set within this 
    for(i=0; i<8; i++)
    {
        int bitMask = (1 << i);
        int res = (val & bitMask);
        int sel = (port->sel & bitMask);
        int dir = (port->dir & bitMask);

        // Check to see if this bit gives us a diff. If not,
        // then continue the loop until we find the bit. Also, check
        // to see if the direction is set to output, if not, then 
        // continue with the loop.
        if((bitMask & diff) == 0 ||   // Check pin difference
           (dir == 0)            ||   // Check pin direction
           (sel == 1))                // Check pin selection
            continue;

        // If res has a value, that means we have a rising edge
        // occuring within this.
        if (res)
            fprintf(stdout, "BIT%d is set\n", i);

        // If res is low, that means we have a falling edge occuring
        // within the GPIO module.
        else
            fprintf(stdout, "BIT%d is cleared\n", i);
    }

}

static uint64_t handle_gpio_port_ext_read(msp430_gpio_device *device,
                                          gpio_port *port, hwaddr offset)
{
    switch(offset)
    {
        case 0:
        case 1:
            return port->bits;
        case 2:
            return port->dir;
        case 6:
            return port->sel;
        default:
            return 0;
    }
}

static uint64_t handle_gpio_port_read(msp430_gpio_device *device,
                                      gpio_port *port, hwaddr offset)
{
    switch(offset)
    {
        case 0:
        case 1:
            return port->bits;
        case 2:
            return port->dir;
        case 3:
            return port->sel;
        default:
            return 0;
    }
}

static void handle_gpio_port_write(msp430_gpio_device *device,
                                   gpio_port *port,
                                   hwaddr offset,
                                   uint64_t val)
{
    offset %= 4;
    switch (offset)
    {
        // Handle Output Write 
        case 1:
            handle_gpio_output(port, val);
            break;

        // Handle Direction Write
        case 2:
            port->dir = (uint8_t)val;
            break;

        // Handle Selection Write
        case 3:
            port->sel = (uint8_t)val;
            break;

        // Handle Input Write - Input write does nothing but increase
        // current consumption. Because of this. Functionality does
        // not exist for this.
        case 0:
        default:
            break;
    }
}

static void handle_gpio_port_ext_write(msp430_gpio_device *device,
                                       gpio_port *port,
                                       hwaddr offset,
                                       uint64_t val)
{
    offset %= 4;
    switch (offset)
    {
        // Handle Output Write 
        case 1:
            handle_gpio_output(port, val);
            break;

        // Handle Direction Write
        case 2:
            port->dir = (uint8_t)val;
            break;

        // Handle Selection Write
        case 6:
            port->sel = (uint8_t)val;
            break;

        // Handle Input Write - Input write does nothing but increase
        // current consumption. Because of this. Functionality does
        // not exist for this.
        case 0: case 3: case 4: case 5:
        default:
            break;
    }
}

/*******************************************************************
 * Helper/Misc Functions
 ******************************************************************/
/**
 * @brief Memory write handler for the MSP430 device's memory mapped IO space.
 * @details Memory write handler for the MSP430 device's memory mapped IO
 * space. This will allow us to do perform actions depending on the type
 * of memory access.
 * 
 * @param opaque MSP430 Device object
 * @param offset Offset into the Device's memory mapped base address.
 * @param val [description]
 * @param size Size of the memory access
 * 
 */
static void memory_write(void* opaque, hwaddr offset, 
                         uint64_t val, unsigned size)
{
    /* Set the new bits within the SVS control register */
    msp430_gpio_device *device = MSP430_GPIO(opaque);
    if(size != 1)
    {
        fprintf(stdout, "Writing to more than two entries. Handle accordingly.");
        return;
    }

    switch(offset)
    {
        case P1IN:  case P1OUT:
        case P1SEL: case P1DIR:
            handle_gpio_port_ext_write(device, &device->p1, offset, val);
            break;

        case P2IN:  case P2OUT:
        case P2SEL: case P2DIR:
            handle_gpio_port_ext_write(device, &device->p2, offset, val);
            break;

        case P3IN:  case P3OUT:
        case P3SEL: case P3DIR:
            handle_gpio_port_write(device, &device->p3, offset, val);
            break;

        case P4IN:  case P4OUT:
        case P4SEL: case P4DIR:
            handle_gpio_port_write(device, &device->p4, offset, val);
            break;

        case P5IN:  case P5OUT:
        case P5SEL: case P5DIR:
            handle_gpio_port_write(device, &device->p5, offset, val);
            break;

        case P6IN:  case P6OUT:
        case P6SEL: case P6DIR:
            handle_gpio_port_write(device, &device->p6, offset, val);
            break;

    }
    return;
}

/**
 * @brief Memory read handler for the MSP430 device's memory mapped IO
 * space.
 * @details Memory read handler for the MSP430 device's memory mapped IO
 * space. This will allow us to do perform actions depending on the type
 * of memory access.
 * 
 * @param opaque MSP430 Device object
 * @param offset Offset into the Device's memory mapped base address.
 * @param size Size of the memory access
 * @return Value that is read.
 */
static uint64_t memory_read(void *opaque, hwaddr offset, unsigned size)
{
/* Set the new bits within the SVS control register */
    msp430_gpio_device *device = MSP430_GPIO(opaque);
    if(size != 1)
    {
        fprintf(stdout, "Writing to more than two entries. Handle accordingly.");
        return 0;
    }

    switch(offset)
    {
        case P1IN:  case P1OUT:
        case P1SEL: case P1DIR:
            return handle_gpio_port_ext_read(device, &device->p1, offset);

        case P2IN:  case P2OUT:
        case P2SEL: case P2DIR:
            return handle_gpio_port_ext_read(device, &device->p2, offset);

        case P3IN:  case P3OUT:
        case P3SEL: case P3DIR:
            return handle_gpio_port_read(device, &device->p3, offset);
           
        case P4IN:  case P4OUT:
        case P4SEL: case P4DIR:
            return handle_gpio_port_read(device, &device->p4, offset);
        
        case P5IN:  case P5OUT:
        case P5SEL: case P5DIR:
            return handle_gpio_port_read(device, &device->p5, offset);
        
        case P6IN:  case P6OUT:
        case P6SEL: case P6DIR:
            return handle_gpio_port_read(device, &device->p6, offset);
        
        default:
            return 0;
    }
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
static void msp430_gpio_reset(DeviceState *dev)
{
    msp430_gpio_device *device = MSP430_GPIO(dev);
    memset(&device->p1, 0x00, sizeof(gpio_port));
    memset(&device->p2, 0x00, sizeof(gpio_port));
    memset(&device->p3, 0x00, sizeof(gpio_port));
    memset(&device->p4, 0x00, sizeof(gpio_port));
    memset(&device->p5, 0x00, sizeof(gpio_port));
    memset(&device->p6, 0x00, sizeof(gpio_port));
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
static int msp430_gpio_init(SysBusDevice *dev)
{
    msp430_gpio_device *device = MSP430_GPIO(dev);
    memory_region_init_io(&device->region, OBJECT(dev),  &mmio_ops,
                          dev, "gpio", MSP430_GPIO_SIZE);
    sysbus_init_mmio(dev, &device->region);
    
    // sysbus_init_irq(device->pucRequest, 0);
    // qdev_init_gpio_in(DEVICE(dev), handle_irq_reset, 1);

    /* Reset the Device so we are able to have an initial state
     * for this device. */
    msp430_gpio_reset(DEVICE(dev));
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
static void msp430_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    sdc->init = msp430_gpio_init;
    dc->reset = msp430_gpio_reset;
}

/**
 * @brief Type registration for MSP430 hardware for family 1.
 * @details Type registration for MSP430 hardware for family 1.
 */
static const TypeInfo msp430_gpio_info = {
    .name          = TYPE_MSP430_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(msp430_gpio_device),
    .class_init    = msp430_gpio_class_init,
};

/**
 * @brief Registers types declared within this hardware device.
 * @details Register types declared within this hardware device file.
 */
static void msp430_gpio_register_types(void)
{
    type_register_static(&msp430_gpio_info);
}

// Define the function that will register new types into 
// QEMU with the hardware declarations within this file.
type_init(msp430_gpio_register_types)
