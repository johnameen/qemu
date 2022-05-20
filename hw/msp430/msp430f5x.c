#include "qemu/osdep.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "migration/vmstate.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"
#include "hw/loader.h"

#define MSP430_RAM_SIZE (16*0x1000)

static void msp430f5x_boot(MSP430Cpu *cpu, MachineState *args)
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
    kernel_size = load_image_targphys(kernel_filename, 0x400, MSP430_RAM_SIZE);
    if (kernel_size < 0)
    {
        fprintf(stderr, "Unable to load firmware to MSP430. Exiting.\n");
        exit(1);
    }
}

static void msp430f5x_init(MachineState *args)
{
    MSP430Cpu *cpu;
    const char *cpu_model = "msp430";
    MemoryRegion *toplevel = get_system_memory();
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    
    cpu = cpu_msp430_init(cpu_model);
    if (!cpu) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }

    memory_region_init_ram(sram, NULL, "msp430.ram", MSP430_RAM_SIZE, NULL);
    vmstate_register_ram_global(sram);
    memory_region_add_subregion(toplevel, 0x400, sram);

    // Attempt to boot the MSP430 with the kernel file. This will allow
    // use to boot the firmware.
    msp430f5x_boot(cpu, args);
    return;
}

static void msp430f5x_machine_init(MachineClass *mc)
{
    mc->desc = "MSP430F5x Family Suite";
    mc->init = msp430f5x_init;
    mc->is_default = 0;
}

DEFINE_MACHINE("msp430f5x", msp430f5x_machine_init)