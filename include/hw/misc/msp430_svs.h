#ifndef HW_MSP430_SVS_H
#define HW_MSP430_SVS_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"

#define MSP430_SVS_WORDSIZE (1)

// #define MSP430_SVS(obj) OBJECT_CHECK(msp430_svs_device, (obj), TYPE_MSP430_SVS)

typedef enum
{
    VLD_OFF   = 0x000,
    VLD_19V   = 0x190,
    VLD_21V   = 0x210,
    VLD_22V   = 0x220,
    VLD_23V   = 0x230,
    VLD_24V   = 0x240,
    VLD_25V   = 0x250,
    VLD_265V  = 0x265,
    VLD_28V   = 0x280,
    VLD_29V   = 0x290,
    VLD_305V  = 0x305,
    VLD_32V   = 0x320,
    VLD_335V  = 0x335,
    VLD_35V   = 0x350,
    VLD_37V   = 0x370,
    VLD_COMP  = 0xFFF,
} VLDValue;

struct msp430_svs_device
{
    // Public Fields
    SysBusDevice parent_obj;

    // Private Fields
    MemoryRegion region;
    uint8_t SVSCTL;
    VLDValue    vldx;
    bool        poron;
};

#define TYPE_MSP430_SVS "msp430_svs"
OBJECT_DECLARE_SIMPLE_TYPE(msp430_svs_device, MSP430_SVS)

#endif //HW_MSP430_SVS_H