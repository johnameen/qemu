#include "qemu/osdep.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"
#include "hw/loader.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qapi/error.h"

#define TYPE_MSP430X1_FLASHROM "msp430x1_flashrom"

#define MSP430_FLASH_SIZE (0x400)     // 1kB Flash
#define MSP430_FLASH_BASE (0xFC00)    // Base Address
#define MSP430_SRAM_SIZE (0x0E00)     // Ram
#define MSP430_SRAM_BASE (0x0200)     // Ram Address
#define MSP430_RESET_ADDR (0xFFFE)    // Reset Address

static void msp430f1x_boot(MachineState *args)
{
    const char *kernel_filename = args->kernel_filename;
    int kernel_size;

    // Check to see if we have a kernel file specified. If we don't, then
    // exit out saying we don't have a kernel file. One is needed to execute.
    if (!kernel_filename)
    {
        fprintf(stderr, "Bare-metal application not specified by the user. Exiting.\n");
        exit(1);
    }

    // Attempt to load kernel file in the appropriate location. This will
    // load the memory at the flash address.
    kernel_size = load_image_targphys(kernel_filename, MSP430_FLASH_BASE, MSP430_FLASH_SIZE);
    if (kernel_size < 0)
    {
        fprintf(stderr, "Unable to load firmware to MSP430. Exiting.\n");
        exit(1);
    }
    qemu_log("MSP430F1x Board Image Loaded...\n");
}

typedef struct 
{
    const char *deviceName;
    uint32_t deviceBaseAddr;
} msp430_peripheral;

msp430_peripheral periphs[] = 
{
    {"msp430_svs",      0x0055},
    {"msp430_watchdog", 0x0120},
    {"msp430_gpio",     0x0018}
};


static void msp430f1x_init(MachineState *args)
{
    const char *cpu_model = "msp430";
    MSP430Cpu *cpu;
    MemoryRegion *sram;
    DeviceState *flashrom;
    DeviceState *svs;
    SysBusDevice *s;


    // Create Flash/Info and Flash Controller
    // flashrom = qdev_create(NULL, "msp430x1_flashrom");
    flashrom = qdev_new(TYPE_MSP430X1_FLASHROM);

    // qdev_init_nofail(flashrom);
    s = SYS_BUS_DEVICE(flashrom);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, 0x0128);

    // Create SRAM Region
    sram = g_new(MemoryRegion, 1);
    memory_region_init_ram(sram, NULL, "msp430.sram", MSP430_SRAM_SIZE, NULL);
    memory_region_add_subregion(get_system_memory(), MSP430_SRAM_BASE, sram);
    vmstate_register_ram_global(sram);

    int entries = sizeof(periphs)/sizeof(msp430_peripheral);
    int i;
    for (i=0; i<entries; i++)
    {
        fprintf(stdout, "Initializing periphs %s @ %08x\n", 
                periphs[i].deviceName, periphs[i].deviceBaseAddr);
        svs = qdev_new(periphs[i].deviceName);
        s = SYS_BUS_DEVICE(svs);
        // svs = qdev_create(NULL, periphs[i].deviceName);
        // qdev_init_nofail(svs);
        sysbus_mmio_map(s, 0, periphs[i].deviceBaseAddr);
    }

    // Attempt to boot the MSP430 with the kernel file. This will allow
    // use to boot the firmware.
    msp430f1x_boot(args);
    cpu = cpu_msp430_init(cpu_model);
    if (!cpu) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }

    return;
}

static void msp430f1x_machine_init(MachineClass *mc)
{
    mc->desc = "MSP430F1x Family Suite";
    mc->init = msp430f1x_init;
    mc->is_default = 1;
}

DEFINE_MACHINE("msp430f1x", msp430f1x_machine_init)
