#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/log.h"
#include "exec/cpu-defs.h"
#include "exec/cpu-common.h"
#include "exec/exec-all.h"


void debug_print_translation_block(TranslationBlock *tb);

void helper_debug(MSP430CpuState *env);

void helper_raise_exception(MSP430CpuState *env, int ex);

void helper_check_flags(MSP430CpuState *env, 
                        int mask,
                        int value,
                        int access);

int32_t helper_bcd_add(MSP430CpuState *env, int op1, int op2);

/**
 * @brief Prints debugging information about the TranslationBlock.
 * 
 * @details Prints debugging information about the TranslationBlock. This is use during
 * development and should not be used during actual execution when it is release.
 * TODO: Fix to use QEMU Logging instead of fprintf
 * TODO: Fix so there is a compiler flag around this if this is needed.
 * 
 * @param tb TranslationBlock structure to print the information about. This is the block 
 * that contains the information used during translations.
 */
void debug_print_translation_block(TranslationBlock *tb)
{
    fprintf(stdout, "=================================\n");
    fprintf(stdout, "TranslationBlock:\n");
    fprintf(stdout, "tb->pc: %08x\n", tb->pc);
    fprintf(stdout, "tb->cs_base: %08x\n", tb->cs_base);
    fprintf(stdout, "tb->flags: %08x\n", (uint32_t)tb->flags);
    fprintf(stdout, "tb->cflags: %08x\n", tb->cflags);
    fprintf(stdout, "tb->icount: %d\n", tb->icount);
    fprintf(stdout, "tb->size: %d\n", tb->size);
}

void helper_debug(MSP430CpuState *env)
{
    CPUState *cs = CPU(msp430_env_get_cpu(env));
    cs->exception_index = EXCP_DEBUG;
    cpu_loop_exit(cs);
}

void helper_raise_exception(MSP430CpuState *env, int ex)
{
    CPUState *cs = CPU(msp430_env_get_cpu(env));
    if (ex == MSP430_EXCP_BREAK)
    {
        if( (env->regs[MSP430_SR_REGISTER] & 0xF0) == 0xF0 )
        {
            fprintf(stdout, "MSP430 CPU: Exiting with value %04x\n", env->regs[15]);
            exit(env->regs[15]);
        }
    }
    else if (ex == MSP430_EXCP_DEBUG)
    {
        log_cpu_state(cs, 0);
    }
    return;
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param value [description]
 */
static inline void check_vflag(MSP430CpuState *env,
                                int value, int access)
{
    if(value & ((access == ACCESS_BYTE) ? 0x80 : 0x8000))
        env->regs[MSP430_SR_REGISTER] |= STATUS_VFLAG;
    else
        env->regs[MSP430_SR_REGISTER] &= ~(STATUS_VFLAG);
}

/**
 * @brief Checks value to see if NFLAG needs to be set or cleared.
 * @details Checks value to see if NFLAG needs to be set or cleared. Checks to see 
 * if the MSBit is set, if so, set the N Flag to high. Reset N flag otherwise. The 
 * MSB is determined by the value within access
 * 
 * @param env Current CPU state in which we can change the registers if need be
 * @param value Value in which to check (operand that is currently being checked)
 * @param access Operand size to check on (Byte, Word, Dword, etc...)
 */
static inline void check_nflag(MSP430CpuState *env,
                        int value, int access)
{
    if(value & ((access == ACCESS_BYTE) ? 0x80 : 0x8000))
        env->regs[MSP430_SR_REGISTER] |= STATUS_NFLAG;
    else
        env->regs[MSP430_SR_REGISTER] &= ~(STATUS_NFLAG);
}

/**
 * @brief Checks value to see if ZFLAG needs to be set or cleared.
 * @details Checks value to see if ZFLAG needs to be set or cleared. Checks to see 
 * if value is zero. If so, set the Z flag to high, reset otherwise.
 * 
 * @param env Current CPU state in which we can change the registers if need be
 * @param value Value in which to check (operand that is currently being checked)
 * @param access Operand size to check on (Byte, Word, Dword, etc...)
 */
static inline void check_zflag(MSP430CpuState *env,
                        int value, int access)
{
    if(value & ((access == ACCESS_BYTE) ? 0xFF : 0xFFFF))
        env->regs[MSP430_SR_REGISTER] &= ~(STATUS_ZFLAG);
    else
        env->regs[MSP430_SR_REGISTER] |= STATUS_ZFLAG;
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param value [description]
 * @param access [description]
 */
static inline void check_borrow(MSP430CpuState *env,
                                int value, int access)
{
    uint32_t mask = (access == ACCESS_BYTE) ? 0xFFFFFF00 : 0xFFFF0000;
    if ((value & mask))
        env->regs[MSP430_SR_REGISTER] &= ~(STATUS_CFLAG);
    else
        env->regs[MSP430_SR_REGISTER] |= STATUS_CFLAG;

}

/**
 * @brief Checks value to see if CFLAG needs to be set or cleared.
 * @details Checks value to see if ZFLAG needs to be set or cleared. Checks to see 
 * if a carry was generated. If so, set C Flag to high, otherwise, reset C flag
 * 
 * @param env Current CPU state in which we can change the registers if need be
 * @param value Value in which to check (operand that is currently being checked)
 * @param access Operand size to check on (Byte, Word, Dword, etc...)
 */
static inline void check_cflag(MSP430CpuState *env,
                        int value, int access)
{
    uint32_t mask = (access == ACCESS_BYTE) ? 0xFFFFFF00 : 0xFFFF0000;
    if ((value & mask))
        env->regs[MSP430_SR_REGISTER] |= STATUS_CFLAG;
    else
        env->regs[MSP430_SR_REGISTER] &= ~(STATUS_CFLAG);
}

/**
 * @brief Checks value to see if CFLAG needs to be set or cleared.
 * @details Checks value to see if ZFLAG needs to be set or cleared. Checks to see 
 * if Z is not set. If so, set C Flag to high, otherwise, reset C flag
 * 
 * @param env Current CPU state in which we can change the registers if need be
 * @param value Value in which to check (operand that is currently being checked)
 * @param access Operand size to check on (Byte, Word, Dword, etc...)
 */
static inline void check_cflag_notz(MSP430CpuState *env,
                                    int value, int access)
{
    if(value & ((access == ACCESS_BYTE) ? 0xFF : 0xFFFF))
        env->regs[MSP430_SR_REGISTER] |= STATUS_CFLAG;
    else
        env->regs[MSP430_SR_REGISTER] &= ~(STATUS_CFLAG);
}


/**
 * @brief Top level check flags function which is used to perform flag checks.
 * @details Top level check flags function which is used to perform flag checks. Function
 * will take the bitmask value from "mask" and check to see what flag checks 
 * should be performed on "value" with operand size "access"
 * 
 * @param env Current CPU state in which we can change the registers if need be
 * @param mask Bitfield that represent the flag checks to perform.
 * @param value Value in which to check (operand that is currently being checked)
 * @param access Operand size to check on (Byte, Word, Dword, etc...)
 */
void helper_check_flags(MSP430CpuState *env, 
                        int mask,
                        int value,
                        int access)
{
    if (mask & CHECK_CFLAG)
        check_cflag(env, value, access);

    if (mask & CHECK_BORROW)
        check_borrow(env, value, access);

    if (mask & CHECK_ZFLAG)
        check_zflag(env, value, access);
    
    if (mask & CHECK_NFLAG)
        check_nflag(env, value, access);
    
    if (mask & CHECK_CFLAG_NOTZ)
        check_cflag_notz(env, value, access);

    if (mask & CHECK_VFLAG)
        check_vflag(env, value, access);
    
    return;
}

int32_t helper_bcd_add(MSP430CpuState *env, int op1, int op2)
{
    int i = 0;
    int value = 0x0;
    int carry = 0x0;
    for (i=0; i<32; i+=4)
    {
        int nibble1 = ((op1 >> i) & 0xF);
        int nibble2 = ((op2 >> i) & 0xF);
        nibble2 += (nibble1 + carry);
        if (nibble2 >= 10)
        {
            nibble2 = 0xF & (nibble2 + 6);
            carry = 1;
        }
        else
            carry = 0;
        value += (nibble2 << i);
    }
    return value;
}


