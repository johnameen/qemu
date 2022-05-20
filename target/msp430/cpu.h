#ifndef _CPU_MSP430_H
#define _CPU_MSP430_H

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qom/cpu.h"

//////////////////////////////////////////////////////////////////////////////////////////////
// Required Implementation for QEMU to Compile
//////////////////////////////////////////////////////////////////////////////////////////////

// REQUIRED: New Architecture needs this macro defined. No where is it specified that
// this needs to be made. But it does. TCG Code related defines
#define TARGET_LONG_BITS 32
#define CPUArchState struct MSP430CpuState
#define TARGET_HAS_ICE 0

#include "exec/cpu-defs.h"
#include "fpu/softfloat.h"

// REQUIRED: New Architecture needs this macro defined. No where is it specified that
// this needs to be made. But it does. TCG Code related defines
#define NB_MMU_MODES 1
#define TARGET_PAGE_BITS 10     /* 1k */
#define TARGET_PHYS_ADDR_SPACE_BITS 32
#define TARGET_VIRT_ADDR_SPACE_BITS 32

#define ENV_GET_CPU(e) CPU(msp430_env_get_cpu(e))
#define ENV_OFFSET offsetof(MSP430Cpu, state)
//////////////////////////////////////////////////////////////////////////////////////////////



// ======== MSP430 Specific Definitions ========
// MSP430 Register Definitions
#define MSP430_NUM_REGISTERS         (16)
#define MSP430_NUM_IRQS              (64)
#define MSP430_PC_REGISTER           (0)
#define MSP430_SP_REGISTER           (1)
#define MSP430_SR_REGISTER           (2)
#define MSP430_CG2_REGISTER          (3)
#define PC_REG                       regs[MSP430_PC_REGISTER]
#define SP_REG                       regs[MSP430_SP_REGISTER]
#define SR_REG                       regs[MSP430_SR_REGISTER]
#define CG2_REG                      regs[MSP430_CG2_REGISTER]
#define MSP430_EXCP_BREAK            (1)
#define MSP430_EXCP_DEBUG            (2)

// Misc QEMU Helper Defines that will help us get CPU class and CPU object
// conversions
#define TYPE_MSP430_CPU "msp430"
#define MSP430_CPU_CLASS(klass)   OBJECT_CLASS_CHECK(MSP430Class, (klass), TYPE_MSP430_CPU)
#define MSP430_CPU(obj)           OBJECT_CHECK(MSP430Cpu, (obj), TYPE_MSP430_CPU)
#define MSP430_CPU_GET_CLASS(obj) OBJECT_GET_CLASS(MSP430Class, (obj), TYPE_MSP430_CPU)

/**
 * Bitmask for each of the bits within this
 */
typedef enum
{
    STATUS_CFLAG  = 0x0001,
    STATUS_ZFLAG  = 0x0002,
    STATUS_NFLAG  = 0x0004,
    STATUS_GIE    = 0x0008,
    STATUS_CPUOFF = 0x0010,
    STATUS_OSCOFF = 0x0020,
    STATUS_SCG0   = 0x0040,
    STATUS_SCG1   = 0x0080,
    STATUS_VFLAG  = 0x0100,
    STATUS_ALL    = 0xFFFF
} msp430_status_mask;

typedef enum 
{
    CHECK_CFLAG = 0x0001,
    CHECK_ZFLAG = 0x0002,
    CHECK_NFLAG = 0x0004,
    CHECK_VFLAG = 0x0100,
    CHECK_CFLAG_NOTZ = 0x0010,
    CHECK_BORROW = 0x0020,

    CHECK_ALL = (CHECK_CFLAG | CHECK_ZFLAG | CHECK_NFLAG | CHECK_VFLAG),
    CHECK_ZCN = (CHECK_ZFLAG | CHECK_CFLAG | CHECK_NFLAG),
    CHECK_ZBN = (CHECK_ZFLAG | CHECK_BORROW | CHECK_NFLAG)

} msp430_flag_check;

typedef enum 
{
    
    NE       = 0, // JNE/JNZ
    NZ       = 0,
    EQ       = 1, // JEQ/JZ
    Z        = 1,
    NC       = 2, // JNC/JLO
    LO       = 2,
    C        = 3, // JC/JHS
    HS       = 3,
    N        = 4, // JN
    GE       = 5, // JGE
    L        = 6, // JL
    ALWAYS   = 7 // JMP

} MSP430_COND;

typedef enum
{
    MSP430_ADDR_INVALID     = 0,
    MSP430_ADDR_REG         = 1,
    MSP430_ADDR_INDEX       = 2,
    MSP430_ADDR_SYM         = 3,
    MSP430_ADDR_ABS         = 4,
    MSP430_ADDR_INDIR_REG   = 5,
    MSP430_ADDR_INDIR_AUTO  = 6,
    MSP430_ADDR_IMM         = 7
} msp430_addr_mode;

/* MSP430 Access Size. This determines the size that will be transferred
 * between one operand to another. */
typedef enum 
{
    ACCESS_INVALID = 0,
    ACCESS_BYTE = 1,
    ACCESS_WORD = 2,
    ACCESS_ADDR_WORD = 4
} msp430_access_size;

/* MSP430 Opcodes: These are all the opcodes for the MSP430. Lucky for us, they advertise 
 * only 27 instructions for the CPU. This low count will make life easier. */
typedef enum 
{
    INVALID = 0,
    MOVA, CMPA, ADDA, SUBA, RRCM, RRAM, RLAM, RRUM,
    RRC, SWPB, RRA, SXT, PUSH, CALL, RETI, CALLA,
    PUSHMA, POPMA, PUSHMW, POPMW, J, JMP, MOV, ADD,
    ADDC, SUBC, SUB, CMP, DADD, BIT, BIC, BIS,
    XOR, AND
} msp430_opcode;

// General State Control of the MSP430 Processor. Contains the state of the
// registers and the IRQ lines that interrupt the CPU for control flow change.
typedef struct MSP430CpuState {
    // MSP430F5x Family has this many registers. This maybe consistent 
    // most of the 16bit 
    uint32_t regs[MSP430_NUM_REGISTERS];

    // MSP430F5x Family Consist of 64 Input IRQ Lines in which they
    // jump to a paricular memory location based on their vector table
    // address.
    void *irqs[64];


    CPU_COMMON
} MSP430CpuState;

// Definition of the MSP430 CPU structure that QEMU will use to define
// the CPU and hold all the metadat about that CPU.
typedef struct {
    // <private>
    CPUState qdev;

    // <public>
    MSP430CpuState state;
} MSP430Cpu;

// Class Structure for the MSP430 CPU. Any particular extra functions
// that maybe needed from this class can be implemented here.
typedef struct {
    /*< private >*/
    CPUClass qdev;

    /*< public >*/
    DeviceRealize parent_realize;
    void (*parent_reset)(CPUState *cpu);
} MSP430Class;


#include "exec/cpu-all.h"
#include "exec/exec-all.h"

//////////////////////////////////////////////////////////////////////////////////////////////
// Required Implementation for QEMU to Compile
//////////////////////////////////////////////////////////////////////////////////////////////

// cpu_get_tb_cpu_state
// cpu_exec

// REQUIRED: Stubbed Functions. This appears to support more the TCG side of 
// QEMU that we don't support currently.
MSP430Cpu *cpu_msp430_init(const char *cpu_model);
int cpu_msp430_exec(CPUState *cpu);
void msp430_translate_init(void);

#define cpu_exec cpu_msp430_exec
//void msp430_cpu_gen_init(void);
//int msp430_cpu_gen_code(CPUArchState *env, struct TranslationBlock *tb, int *gen_code_size_ptr);
//void QEMU_NORETURN msp430_cpu_resume_from_signal(CPUState *cpu, void *puc);
//void QEMU_NORETURN msp430_cpu_io_recompile(CPUState *cpu, uintptr_t retaddr);
//#define cpu_gen_init             msp430_cpu_gen_init
//#define cpu_gen_code             msp430_cpu_gen_code
//#define cpu_resume_from_signal   msp430_cpu_resume_from_signal
//#define cpu_io_recompile         msp430_cpu_io_recompile

//void tlb_fill(CPUState *cpu, target_ulong addr, int is_write, int mmu_idx,
//              uintptr_t retaddr);

// REQUIRED: QEMU Needs this function in order to survice and run as a CPU
// object. Without this, it will compile and complain about implicit declaration.
static inline void cpu_get_tb_cpu_state(MSP430CpuState *state, 
                                        target_ulong *pc,
                                        target_ulong *cs_base, 
                                        uint32_t *flags)
{
    *pc = (0xFFFF) & state->PC_REG;
    *cs_base = 0;
    *flags = 0;
}

static inline MSP430Cpu *msp430_env_get_cpu(MSP430CpuState *state)
{
    return container_of(state, MSP430Cpu, state);
}

static inline int cpu_mmu_index(MSP430CpuState *env, bool ifetch) {
    return 0;
}

void msp430_cpu_dump_state(CPUState *cs, 
                          FILE *f, 
                          fprintf_function cpu_fprintf,
                          int flags);

//////////////////////////////////////////////////////////////////////////////////////////////



#endif
