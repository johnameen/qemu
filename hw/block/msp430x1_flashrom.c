#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/log.h"

#define TYPE_MSP430X1_FLASHROM "msp430x1_flashrom"
#define MSP430X1_FLASHROM(obj) OBJECT_CHECK(MSP430FlashRomState, (obj), TYPE_MSP430X1_FLASHROM)

#define MSP430X1_FLASHROM_CONFIG_BASE  (0x0128)
#define MSP430X1_FLASHROM_CONFIG_WORDSIZE (3)
#define FCTL1 (0)
#define FCTL2 (1)
#define FCTL3 (2)

typedef struct 
{
    MemoryRegion region;
    void *ptr;
    uint32_t base;
    uint32_t size;
    uint32_t segment_size;
    const char *name;
    bool isRom;
} MspMemory;

typedef enum 
{
    ERASE_ILLEGAL   = 0x0,
    ERASE_SEGMENT   = 0x1,
    ERASE_MASS      = 0x2,
    ERASE_ALLFLASH  = 0x3,
} flash_erase_mode;

typedef enum
{
    WRITE_ILLEGAL0  = 0x0,
    WRITE_BYTEWORD  = 0x1,
    WRITE_ILLEGAL1  = 0x2,
    WRITE_BLOCK     = 0x3,

} flash_write_mode;

typedef struct 
{
    // Public Fields
    SysBusDevice parent_obj;

    // Private Fields
    // void *storage;
    // MemoryRegion memory;
    // uint32_t memory_base;
    // uint32_t memory_size;
    // uint32_t memory_segment_size;
    // char *memory_name;
    // bool isRom;

    MspMemory flash;
    MspMemory info;

    MemoryRegion iomem;
    hwaddr config_base;
    int32_t config_size;
    uint16_t r[MSP430X1_FLASHROM_CONFIG_WORDSIZE];

    flash_write_mode writeMode;
    flash_erase_mode eraseMode;
    uint8_t fssel;
    uint8_t fn;

} MSP430FlashRomState;

/*******************************************************************
 * Prototypes
 ******************************************************************/
static void flashrom_config_write(void* opaque, hwaddr offset, uint64_t val, unsigned size);
static uint64_t flashrom_config_read(void *opaque, hwaddr offset, unsigned size);
static void flash_memory_write(void* opaque, hwaddr offset, uint64_t val, unsigned size);
static void info_memory_write(void* opaque, hwaddr offset, uint64_t val, unsigned size);
static uint64_t memory_read(void *opaque, hwaddr offset, unsigned size);

/*******************************************************************
 * Global Structures
 ******************************************************************/
static const MemoryRegionOps mmio_ops = {
    .read = flashrom_config_read,
    .write = flashrom_config_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps flash_ops = {
    .read = memory_read,
    .write = flash_memory_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps info_ops = {
    .read = memory_read,
    .write = info_memory_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};



/*******************************************************************
 * Helper/Misc Functions
 ******************************************************************/
static inline void mspmemory_write(MSP430FlashRomState *device,
                                   MspMemory *mem, 
                                   hwaddr offset,
                                   uint64_t val,
                                   unsigned size)
{
    uint8_t *p = mem->ptr;
    uint8_t *data = (uint8_t *)&val;
    int i = 0;
    
    /* Check to see if the lock bit is set. If it is, then return to signal 
     * that a write is not performed. This bit locks erases/writes. */
    if (device->r[FCTL3] & 0x10)
        return;

    // For writes to flash, emulate the write to flash, which will toggle
    // down the bits. Multiple writes will force the values to eventually
    // be zero if all the bits are written.
    if (device->eraseMode == ERASE_SEGMENT)
    {
        offset &= ~(mem->segment_size-1);
        memset(&p[offset], 0xFF, mem->segment_size);
        // TODO: Put delay for segment 
    }
    
    // If the mode is set to ERASE_MASS, then we erase the entire segment
    else if (device->eraseMode == ERASE_MASS ||
             device->eraseMode == ERASE_ALLFLASH)
    {
        uint8_t *dptr = (uint8_t *)device->flash.ptr;
        uint32_t dataSize = device->flash.size;
        memset(&dptr[0], 0xFF, dataSize);
    
        if (device->eraseMode == ERASE_ALLFLASH)
        {
            dptr = (uint8_t *)device->info.ptr;
            dataSize = device->info.size;
            memset(&dptr[0], 0xFF, dataSize);
        }
    }   

    // If the following is not a write, then handle the case where
    // we are doing a write to a block.
    else if(device->writeMode == WRITE_BLOCK || 
           device->writeMode == WRITE_BYTEWORD)
    {
        for (i=0; i<size; i++)
            p[offset+i] &= data[i];
    }

    // Return
    return;
}

static void info_memory_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    MSP430FlashRomState *device = (MSP430FlashRomState *)opaque;
    fprintf(stdout, "MSP430: INFO Write @ %08x(%d): %08x\n",
            (unsigned int)offset, size, (unsigned int)val);
    mspmemory_write(device, &device->info, offset, val, size);
}

static void flash_memory_write(void* opaque, hwaddr offset, uint64_t val, unsigned size)
{
    MSP430FlashRomState *device = (MSP430FlashRomState *)opaque;
    fprintf(stdout, "MSP430: FLASH Write @ %08x (%d)\n",
            (unsigned int)offset, size);
    mspmemory_write(device, &device->flash, offset, val, size);
}

static uint64_t memory_read(void *opaque, hwaddr offset, unsigned size)
{
    //TODO: Figure out how to handle this. For some reason, have some issues
    // with just copying the data to val.
    // Maybe byte ordering issue?
    uint64_t val = 0;
    //MSP430FlashRomRawState *d = (MSP430FlashRomRawState *)opaque;
    //uint8_t *p = d->storage;
    //memcpy(&val, p+offset, size);
    return val;
}

static void flashrom_handle_fctl1(MSP430FlashRomState *device, uint64_t val)
{
    uint16_t temp;
    // Mask out the reserved/key bits, so we can see what is
    // needed to be analyzed.
    temp = (uint16_t)(val & 0xFFC6);
    device->r[FCTL1] = temp;

    // Find the mode in which we are in for erasures.
    device->eraseMode = (flash_erase_mode)(temp & 0x6) >> 1;
    device->writeMode = (flash_write_mode)(temp & 0xC0) >> 6;
    return;
}

static void flashrom_handle_fctl2(MSP430FlashRomState *device, uint64_t val)
{
    uint16_t temp;
    // Mask out the reserved/key bits, so we can see what is
    // needed to be analyzed.
    temp = (uint16_t)(val & 0xFFFF);
    device->r[FCTL2] = temp;

    // Find the mode in which we are in for erasures.
    device->fssel = (val & 0xC0) >> 6;
    device->fn = (val & 0x3F);
    return;
}

static void flashrom_handle_fctl3(MSP430FlashRomState *device, uint64_t val)
{
    uint16_t temp;
    // Mask out the reserved/key bits, so we can see what is
    // needed to be analyzed.
    temp = (uint16_t)(val & 0xFF37);
    device->r[FCTL3] = temp;
    return;
}


static void flashrom_config_write(void* opaque, hwaddr offset, 
                                  uint64_t val, unsigned size)
{
    MSP430FlashRomState *device = (MSP430FlashRomState *)opaque;
    qemu_log("MSP430X1 FlashRom Config: Handling Memory Write\n");
    offset >>= 1;

    // Reject any writes that are not within the bounds of 
    // two bytes. This is considered illegal.
    // TODO: Handle larger sizes
    if (size != 2)
        return;

    // Switch to handle the proper offset and value
    // that is going to be used. 
    switch (offset)
    {
        case FCTL1:
            flashrom_handle_fctl1(device, val);
            break;

        case FCTL2:
            flashrom_handle_fctl2(device, val);

        case FCTL3:
            flashrom_handle_fctl3(device, val);
            break;
    }
    return;
}

static uint64_t flashrom_config_read(void *opaque, hwaddr offset, unsigned size)
{
    MSP430FlashRomState *d = (MSP430FlashRomState *)opaque;
    offset >>= 1;
    qemu_log("MSP430X1 FlashRom Config: Handling Memory Read\n");
    return d->r[offset];
}

static void msp430_register_memory(MspMemory *memory,
                                   MSP430FlashRomState *s,
                                   const MemoryRegionOps *ops,
                                   int n)
{
    memory_region_init_rom_device(&memory->region, OBJECT(s), 
                                  ops, s,
                                  memory->name,
                                  memory->size, 
                                  NULL);
    vmstate_register_ram(&memory->region, DEVICE(s));
    memory->ptr = memory_region_get_ram_ptr(&memory->region);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &memory->region);
    sysbus_mmio_map(SYS_BUS_DEVICE(s), n, memory->base);
}

/*******************************************************************
 * Object Callback Functions
 ******************************************************************/
static void msp430x1_flashrom_reset(DeviceState *dev)
{
    MSP430FlashRomState *s = MSP430X1_FLASHROM(dev);
    
    /* Set the default for the flash memory module registers.
     * Also set any metadata within the State object */
    s->r[FCTL1] = 0x9600;
    s->r[FCTL2] = 0x9642;
    s->r[FCTL3] = 0x9618;
    flashrom_handle_fctl1(s, 0x9600);
    flashrom_handle_fctl2(s, 0x9642);
    flashrom_handle_fctl2(s, 0x9618);
}

/**
 * @brief Initialization Function for the Flash/ROM Family 1 device.
 * @details Initialization Function for the Flash/ROM Family 1 device.
 * This  function is responsible for initializing the object device that 
 * is instantiated as a FlashROM Family 1 device.
 * 
 * @param dev SysBusDevice that will be initialized as a FlashROM
 * Family 1 device.
 * @return If initialization was successful or not (0 for success)
 */
static int msp430x1_flashrom_init(SysBusDevice *dev)
{
    // Cast the SysBusDevice to be a bcm2835_gpio_t
    // device. This also checks to ensure this is the right
    // object that is being used.
    MSP430FlashRomState *s = MSP430X1_FLASHROM(dev);

    // Register Configuration space that is being used by the 
    // FlashRom device.
    s->config_size = (MSP430X1_FLASHROM_CONFIG_WORDSIZE * 2);
    s->config_base = MSP430X1_FLASHROM_CONFIG_BASE;
    memory_region_init_io(&s->iomem, OBJECT(s), &mmio_ops, 
                          s, "flash-cfg", s->config_size);
    sysbus_init_mmio(dev, &s->iomem);

    // Register the main memory location. This will allow us
    // to register this as a rom device which will give us a toggle on
    // whether we can intercept read/writes or not.
    s->flash.base = 0xFC00;
    s->flash.size = 0x400;
    s->flash.name = "flash";
    s->flash.segment_size = 512;
    msp430_register_memory(&s->flash, s, &flash_ops, 1);
    
    // Informational Segment
    s->info.base = 0x1000;
    s->info.size = 0x100;
    s->info.name = "info";
    s->info.segment_size = 128;
    msp430_register_memory(&s->info, s, &info_ops, 2);



    return 0;
}

/*******************************************************************
 * Registration and Initialization Functions (Class and Object)
 ******************************************************************/
/**
 * @brief Properties for the Flash/ROM Base device
 * @details Properties for the Flash/ROM base device. This allows for
 * the basic information about the Flash/ROM memory.
 */
// static Property msp430_flashrom_props[] = {
//     DEFINE_PROP_UINT32("memory_size", MSP430FlashRomState, memory_size, 0),
//     DEFINE_PROP_UINT32("memory_base", MSP430FlashRomState, memory_base, 0),
//     DEFINE_PROP_STRING("memory_name", MSP430FlashRomState, memory_name),
//     DEFINE_PROP_END_OF_LIST(),
// };

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param klass [description]
 * @param data [description]
 */
static void msp430x1_flashrom_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    sdc->init = msp430x1_flashrom_init;
    dc->reset = msp430x1_flashrom_reset;
    //dc->props = msp430_flashrom_props;
}

/**
 * @brief Type registration for MSP430 ROM/FLASH memory hardware for family 1.
 * @details Type registration for MSP430 ROM/FLASH memory hardware for
 * family 1. This device controls the writes/erases that will happen on the flash
 * part.
 */
static const TypeInfo msp430_flashrom_info = {
    .name          = TYPE_MSP430X1_FLASHROM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MSP430FlashRomState),
    .class_init    = msp430x1_flashrom_class_init,
};

/**
 * @brief Registers types declared within this hardware device.
 * @details Register types declared within this hardware device file.
 */
static void msp430x1_flashrom_register_types(void)
{
    type_register_static(&msp430_flashrom_info);
}

// Define the function that will register new types into 
// QEMU with the hardware declarations within this file.
type_init(msp430x1_flashrom_register_types)
