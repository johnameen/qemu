#include "cpu.h"
#include "tcg/tcg-op.h"
#include "exec/cpu_ldst.h"
#include "translate.h"
#include "qemu/qemu-print.h"
#include "exec/translator.h"
#include "exec/exec-all.h"

/* Variable is needed in order for gen-icount.h to compile correctly and work. It relies on this
 * global variable in order to operate the start/end functions of gen_tb_(start/end)() */
#include "exec/gen-icount.h"

/* Creating global TCG values that represent the register values and any other CPU registers 
 * within the CPU State Environment object. This maps the TCG variables to the structure offset
 * so the TCG engine knows where to load/store values from. */
#define TCGV_REG(x)    tcg_regs[x] 
#define TCGV_CPU_PC    tcg_regs[MSP430_PC_REGISTER]
#define TCGV_CPU_SR    tcg_regs[MSP430_SR_REGISTER]
static TCGv tcg_regs[MSP430_NUM_REGISTERS];

/* TODO: If there is a way to generate a compilation error to generate if the define does not match
 * up to the number of items in the list, that would be great */
/* List of register names that are currently being created within the MSP430CpuState structure */
static const char * reg_names[MSP430_NUM_REGISTERS] = 
{
    "$r0", "$r1", "$r2", "$r3",
    "$r4", "$r5", "$r6", "$r7",
    "$r8", "$r9", "$r10", "$r11",
    "$r12", "$r13", "$r14", "$r15",
};

// ===============================================================================================
// Function Prototypes
// ===============================================================================================


// ===============================================================================================
// Helper Functions for the MSP430.
//    These functions should be static, and possibly inline if appropriate
// ===============================================================================================

// (CPUState *cpu, FILE *, int flags)
void msp430_cpu_dump_state(CPUState *cs, 
                          FILE *f, 
                          int flags)
{
    MSP430Cpu *cpu = MSP430_CPU(cs);
    MSP430CpuState *env = &cpu->env;

    int i;
    qemu_fprintf(f, "========================================\n");
    for (i = 0; i < 16; i += 4) {
        qemu_fprintf(f, "$r%d=0x%04x $r%d=0x%04x $r%d=0x%04x $r%d=0x%04x\n",
                    i, env->regs[i], i+1, env->regs[i+1],
                    i+2, env->regs[i+2], i+3, env->regs[i+3]);
    }
    qemu_fprintf(f, "\n");
}

/**
 * @brief Prints error message and exits out of the MSP430 emulation
 * @details Prints error message and exits out of the MSP430 emulation. This is done
 * when a fatal error occurs and cannot handle the error accordingly.
 * 
 * @param condition Condition to test. If the condition is true, the msg will be printed
 * to stderr and the application will exit()
 * @param msg Message to print when condition is set to True.
 */
static inline void msp430_error(bool condition, const char *msg)
{
    if (condition)
    {
        fprintf(stderr, "%s", msg);
        exit(1);
    }    
}

/*
static void debug_dump_operand(msp430_operand *op, bool isSrcOp)
{
    fprintf(stdout, "%s Operand ================ \n", isSrcOp ? "Src" : "Dst");
    fprintf(stdout, "%d | %x | %x\n", op->type, op->reg, op->imm);
}
*/


// TODO: Comment
static inline void msp430_get_cflag(TCGv flag)
{
    tcg_gen_andi_i32(flag, TCGV_REG(MSP430_SR_REGISTER), 0x1);
    return;
}

/**
 * @brief Sets specific bits within the status register.
 * @details Sets specific bits within the status register. This
 * is done by generating an OR instruction with the appropriate 
 * mask value to perform a bitwise OR on the status register with
 * the appropriate mask value.
 * 
 * @param mask Bits to set to the status register
 */
static inline void msp430_set_flag(msp430_status_mask mask) 
{
    tcg_gen_ori_i32(TCGV_CPU_SR, TCGV_CPU_SR, mask);
}

/**
 * @brief Clears specific bits within the status register.
 * @details Clears specific bits within the status register. This
 * is done by generating an AND instruction with the appropriate 
 * mask value to perform a bitwise AND on the status register with
 * the appropriate mask value.
 * 
 * @param mask Bits to clear from the status register
 */
static inline void msp430_clear_flag(msp430_status_mask mask)
{
    tcg_gen_andi_i32(TCGV_CPU_SR, TCGV_CPU_SR, (STATUS_ALL ^ mask));
}

/**
 * @brief Sets the carry 'C' flag within the SR register with
 * TCG instructions. 
 * @details Sets the carry 'C' flag within the SR register
 * with TCG instructions. This is done by clearing the flag and 
 * or'ing the SR register with the value stored in FLAG
 * 
 * @param flag Value that contains the flag to set or clear the carry
 * flag. Flag must be either 1 or zero before passing into this function.
 */
static inline void msp430_set_cflag(TCGv flag)
{
    msp430_clear_flag(STATUS_CFLAG);
    tcg_gen_or_i32(TCGV_CPU_SR, TCGV_CPU_SR, flag);
}

static inline void msp430_raise_excp(int excpNum)
{
    TCGv num = tcg_const_i32(excpNum);
    gen_helper_raise_exception(cpu_env, num);
    tcg_temp_free_i32(num);
}

static inline void msp430_check_flags(msp430_flag_check flags,
                                      msp430_access_size access,
                                      TCGv temp)
{
    TCGv tcgAccess = tcg_const_i32(access);
    TCGv tcgFlags = tcg_const_i32(flags);

    gen_helper_check_flags(cpu_env, tcgFlags, temp, tcgAccess);
    tcg_temp_free_i32(tcgAccess);
    tcg_temp_free_i32(tcgFlags);
}

/**
 * @brief Fetch function in order to fetch the next word of code
 * @details Fetch function in order to fetch the next word of code. This function
 * calculates the offset, fetches the word data, stores it within the Disasm 
 * context, and updates the DisasmContext metadata that is needed
 * 
 * @param env Current MSP430 CPU Environment State
 * @param ctx Current Disassembler Context that is used
 */
static inline void msp430_fetch_next_word(MSP430CpuState *env, DisasmContext *ctx)
{
    target_ulong offset;

    // TODO: Report that this happened. Like an assert or something similar
    /* Check to see if we are going to be beyond the buffer size. 
     * If we are, return without a fetch. */
    if (ctx->instr_len >= MSP430_MAX_INSTR_WORDS)
        return;

    // Calculate the offset and fetch the next unsigned word from QEMU and 
    // store it within the DisasmContext
    offset = ctx->pc + (ctx->instr_len << 1);
    ctx->instr_buf[ctx->instr_len++] = cpu_lduw_code(env, offset);
}

/**
 * @brief Fetches the first word of an instruction
 * @details Fetches the first word of an instruction. This function also
 * handles dealing with typical information that is needed about the first
 * instruction in order to properly decode the instruction.
 * 
 * @param env The current CPU state.
 * @param ctx The current disassembler context.
 */
static inline void msp430_fetch_opcode_instr(MSP430CpuState *env, DisasmContext *ctx)
{
    /* Fetch the first 16 bits to determine what to do next within
     * the CPU. */ 
    msp430_fetch_next_word(env, ctx);
    ctx->instr = &ctx->instr_buf[ctx->instr_len-1];

    /* Generate  the Instruction Map Row Mask which can be found in the SLAU208N document 
     * which is referenced in Table 6.20: Instruction Map for MSP430X */
    ctx->rowIdx = ((ctx->instr[0] >> 10) & 0x3F);
    ctx->colIdx = ((ctx->instr[0] >> 6) & 0xF);    
}

/**
 * @brief Appends the preamble and the final instructions needed for
 * @details [long description]
 * 
 * @param tb [description]
 * @param targetAddr [description]
 * @param idx [description]
 */
static inline void msp430_exit_tb(DisasmContext *ctx, 
                                  target_ulong targetAddr, int idx)
{
    TranslationBlock *tb = ctx->tb;

    // TODO: Look into target page size and whether we can stay with this,
    // or if we have to implement that (Jumps inside to TranslationBlocks vs
    // jumps outside of translations blocks.)
    if((tb->pc & TARGET_PAGE_MASK) == (targetAddr & TARGET_PAGE_MASK)
        && !ctx->singlestep_enabled)
    {
        tcg_gen_goto_tb(idx);
        tcg_gen_movi_i32(TCGV_CPU_PC, targetAddr);
        tcg_gen_exit_tb(tb, idx);
    }
    else
    {
        tcg_gen_movi_i32(TCGV_CPU_PC, targetAddr);
        if(ctx->singlestep_enabled)
            msp430_raise_excp(MSP430_EXCP_DEBUG);
        tcg_gen_exit_tb(tb, 0);
    }
}

/**
 * @brief Helper function to handle generating load instructions
 * @details Helper function to handle generating load instructions. Function
 * determines which load function to use based on the msp430_access_size
 * value
 * 
 * @param val TCGv variable to store the value into
 * @param address TCGv variable which holds the address to load from
 * @param offset Offset of the address to load from
 * @param size Access Size to load from
 */
static inline void msp430_ld_helper(TCGv val,
                                    TCGv address,
                                    int32_t offset,
                                    msp430_access_size size)
{
    if(size == ACCESS_BYTE)
        tcg_gen_qemu_ld8u(val, address, offset);
    else if(size == ACCESS_WORD)
        tcg_gen_qemu_ld16u(val, address, offset);
    else
        msp430_error(true, "MSP430 Invalid Operand Access type for IndirAuto\n");
}

/**
 * @brief Helper function to handle generating load instructions
 * @details Helper function to handle generating load instructions. Function
 * determines which load function to use based on the msp430_access_size
 * value
 * 
 * @param val TCGv variable to load the value into
 * @param address TCGv variable which holds the address to store to
 * @param offset Offset of the address to store to
 * @param size Access Size to store with
 * TODO: Fix offset issue
 */
static inline void msp430_st_helper(TCGv val,
                                    TCGv address,
                                    int32_t offset,
                                    msp430_access_size size)
{
    if(size == ACCESS_BYTE)
        tcg_gen_qemu_st8(val, address, 0);
    else if(size == ACCESS_WORD)
        tcg_gen_qemu_st16(val, address, 0);
    else
        msp430_error(true, "MSP430 Invalid Operand Access type for IndirAuto\n");
}

// ===============================================================================================
// Private Functions
// ===============================================================================================
#define msp430_operand_preamble(x, y)        _msp430_operand_preamble(x, y, true)
#define msp430_operand_addr_preamble(x, y)   _msp430_operand_preamble(x, y, false)
static void _msp430_operand_preamble(msp430_operand *op,
                                     msp430_access_size access,
                                     bool loadStoreOp)
{
    /* Switch based on the operand type the operand is currently set as. This will
     * help us find out what to do for the preamble. */
    switch(op->type)
    {
        /* Register Addressing: Have to mask the temporary so we can read
         * the appropriate data size from the register. */
        case MSP430_ADDR_REG:
            op->tcgVal = tcg_temp_new();
            tcg_gen_mov_i32(op->tcgVal, TCGV_REG(op->reg));
            tcg_gen_andi_i32(op->tcgVal, op->tcgVal, (access == ACCESS_BYTE) ? 0xFF : 0xFFFF);
            break;

        case MSP430_ADDR_INDIR_AUTO:
        case MSP430_ADDR_INDIR_REG:
            op->tcgVal = tcg_temp_new();
            tcg_gen_mov_i32(op->tcgVal, TCGV_REG(op->reg));
            if(loadStoreOp)
                msp430_ld_helper(op->tcgVal, TCGV_REG(op->reg), 0x0, access);
            break;

        case MSP430_ADDR_IMM:
            op->tcgVal = tcg_temp_new();
            tcg_gen_movi_i32(op->tcgVal, op->imm);
            break;

        case MSP430_ADDR_SYM:
            op->tcgVal = tcg_temp_new();
            tcg_gen_addi_i32(op->tcgVal, TCGV_REG(op->reg), op->imm);
            tcg_gen_addi_i32(op->tcgVal, op->tcgVal, 2);
            if(loadStoreOp)
                msp430_ld_helper(op->tcgVal, op->tcgVal, 0x0, access);
            break;
        
        case MSP430_ADDR_INDEX:
            op->tcgVal = tcg_temp_new();
            tcg_gen_addi_i32(op->tcgVal, TCGV_REG(op->reg), op->imm);
            if(loadStoreOp)
                msp430_ld_helper(op->tcgVal, op->tcgVal, 0x0, access);
            break;
        
        case MSP430_ADDR_ABS:
           op->tcgVal = tcg_temp_new();
           tcg_gen_movi_i32(op->tcgVal, op->imm);
           if(loadStoreOp)
               msp430_ld_helper(op->tcgVal, op->tcgVal, 0x0, access);
           break;

        default:
           msp430_error(true, "MSP430: Invalid Operand Type. Exiting\n");
           exit(1);
    }
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param x [description]
 * @param int [description]
 * @param access [description]
 * @param loadStoreOp [description]
 * @return [description]
 */
#define msp430_operand_postamble(ctx, x, y)       _msp430_operand_postamble(ctx, x, y, true)
#define msp430_operand_addr_postamble(ctx, x, y)  _msp430_operand_postamble(ctx, x, y, false)
static int _msp430_operand_postamble(DisasmContext *ctx,
                                    msp430_operand *op,
                                    msp430_access_size access,
                                    bool loadStoreOp)
{
    TranslationBlock *tb = ctx->tb;
    /* Switch based on the operand type the operand is currently set as. This will
     * help us find out what to do for the postamble. */
    switch(op->type)
    {
        /* Register Addressing: Have to mask the temporary so we can write back the 
         * appropriate sized data to the register. Also, we need to or the data back
         * to the register after clearing the bits we are going to write to. */
        case MSP430_ADDR_REG:
            tcg_gen_andi_i32(op->tcgVal, op->tcgVal,
                             (access == ACCESS_BYTE) ? 0xFF : 0xFFFF);
            tcg_gen_andi_i32(TCGV_REG(op->reg), TCGV_REG(op->reg),
                             (access == ACCESS_BYTE) ? 0xFFFFFF00 : 0xFFFF0000);
            tcg_gen_or_i32(TCGV_REG(op->reg), TCGV_REG(op->reg), op->tcgVal);
            tcg_temp_free_i32(op->tcgVal);

            // Check to see if we are doing a write to the SR register. If we 
            // are, we want to throw an excpetion in QEMU to handle accordingly.
            // This is similiar to a control register for the oscillator/cpu 
            // off bits.
            if(!(op->isSrcOp) && op->reg == MSP430_SR_REGISTER)
                msp430_raise_excp(MSP430_EXCP_BREAK);

            // If we have a change in the PC register (Register addressing as
            // a destination), this means we have a control flow instruction.
            // Exit the TB.
            if(!(op->isSrcOp) && op->reg == MSP430_PC_REGISTER)
            {
                tcg_gen_exit_tb(tb, 0);
                return 1;
            }
            return 0;

        case MSP430_ADDR_SYM:
        case MSP430_ADDR_INDEX:
        {
            if(loadStoreOp)
            {
                TCGv temp = tcg_temp_new();
                tcg_gen_addi_i32(temp, TCGV_REG(op->reg), op->imm);
                if (op->type == MSP430_ADDR_SYM)
                    tcg_gen_addi_i32(temp, temp, 0x2);
                msp430_st_helper(op->tcgVal, temp, 0x0, access);
                tcg_temp_free_i32(temp);
            }
            break;
        }

        case MSP430_ADDR_ABS:
        {
            if(loadStoreOp)
            {
                TCGv temp = tcg_temp_new();
                tcg_gen_movi_i32(temp, op->imm);
                msp430_st_helper(op->tcgVal, temp, 0x0, access);
                tcg_temp_free_i32(temp);
            }
            break;
        }

        // =====================================================================
        // Source only operands. Don't need to store the value back into memory
        // since it is a source operand and should not be modified.
        // =====================================================================
        case MSP430_ADDR_INDIR_AUTO:
            if (op->reg == MSP430_SP_REGISTER)
                tcg_gen_addi_i32(TCGV_REG(op->reg), TCGV_REG(op->reg), 2);
            else
                tcg_gen_addi_i32(TCGV_REG(op->reg), TCGV_REG(op->reg), access);
            break;

        case MSP430_ADDR_INDIR_REG:
        case MSP430_ADDR_IMM:
            break;

        default:
           msp430_error(true, "MSP430: Invalid Operand Type. Exiting\n");
           exit(1);
    }

    // Free the op->tcgVal that was allocated by the preamble
    // code. Also, null out the value so that a leftover pointer does
    // not exist.
    tcg_temp_free_i32(op->tcgVal);
    return 0;
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param ctx [description]
 * @param op [description]
 */
static inline void msp430_operand_handle_0x1(MSP430CpuState *env,
                                             DisasmContext *ctx,
                                             msp430_operand *op)
{
    /* All the modes require an extra word for the x Value which is used to
     * determine the offset into the register. */
    msp430_fetch_next_word(env, ctx);

    /* Switch based on the register value that is stored within */
    switch (op->reg)
    {
        // Mode is Symbolic Mode. Symbolic mode is set when the 
        // rdst is set to PC register. (LABEL)
        case MSP430_PC_REGISTER:
            op->imm = SIGNEXT32(16, ctx->instr[ctx->instr_len-1]);
            op->type = MSP430_ADDR_SYM;
            break;

        // Mode is Absolute mode. Absolute mode is set when the 
        // rdst is set to SR register. (&LABEL)
        case MSP430_SR_REGISTER:
            op->imm = ctx->instr[ctx->instr_len-1];
            op->type = MSP430_ADDR_ABS;
            break;

        // Mode is Index mode. Index mode is set when the 
        // rdst is set to a regular register. ( x(Rn) )
        // TODO: Check 
        default:
            op->imm = SIGNEXT32(16, ctx->instr[ctx->instr_len-1]);
            op->type = MSP430_ADDR_INDEX;
            break;
    }
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param ctx [description]
 * @param op [description]
 * @param swtichValue [description]
 * @param isSrcOp [description]
 */
static inline void msp430_handle_operand(MSP430CpuState *env,
                                         DisasmContext *ctx,
                                         msp430_operand *op,
                                         uint8_t switchValue,
                                         bool isSrcOp)
{
    /* Saving information on whether this is a src or destination operand. This will be used later to 
     * determine if an operand is a destination. (e.g. : Exception when writing to SR) */
    op->isSrcOp = isSrcOp;

    /* Switch on the value specified within the function parameters. This input value is typically
     * the bits As or Ad from the instruction. */
    switch (switchValue)
    {
        // Register Addressing - If the value is 0b00, then the mode is a
        // register mode.
        case 0x0:
            if (op->reg == 0x3 && isSrcOp)
            {
                op->type = MSP430_ADDR_IMM;
                op->imm =  0;
            }
            else
                op->type = MSP430_ADDR_REG;
            break;

        // Indexed / Symbolic / Absolute Addressing mode. These modes use the
        // same value within the Ad value. The mode is determined by the rdst value.
        case 0x1:
            if (op->reg == 3 && isSrcOp)
            {
                op->type = MSP430_ADDR_IMM;
                op->imm = 1;
            }
            /* Handle the case where Addr value is 0x1 where there are
             * multiple addressing modes within this category. */
            else
                msp430_operand_handle_0x1(env, ctx, op);
            break;

        // Handle Addressing mode indirect register mode. 
        case 0x2:
            msp430_error((!isSrcOp), "Invalid Dest Operand. Exiting.\n");
            if (op->reg == 2)
            {
                op->type = MSP430_ADDR_IMM;
                op->imm = 4;
            }
            else if (op->reg == 3)
            {
                op->type = MSP430_ADDR_IMM;
                op->imm = 2;
            }
            else
                op->type = MSP430_ADDR_INDIR_REG;
            break;

        case 0x3:
            // Handle the case where we are using the indirect autoincrement.
            msp430_error((!isSrcOp), "Invalid Dest Operand. Exiting.\n");
            if (op->reg == MSP430_PC_REGISTER)
            {
                // Handle the case for an immediate value is being
                // used as an operand. The rdst register has to be value
                // 0 (or PC register) in order for Immediate register
                // can be used.
                msp430_fetch_next_word(env, ctx);
                op->imm = SIGNEXT32(16, ctx->instr[ctx->instr_len-1]);
                op->type = MSP430_ADDR_IMM;
            }
            else if (op->reg == 2)
            {
                op->type = MSP430_ADDR_IMM;
                op->imm = 8;
            }
            else if (op->reg == 3)
            {
                op->type = MSP430_ADDR_IMM;
                op->imm = -1;
            }
            else
            {
                op->type = MSP430_ADDR_INDIR_AUTO;
                break;
            }

            break;


        /* Handle the case where the value of the switchValue is an invalid option.
         * If this is the case, report to the user that there is an invalid instruction
         * and exit out */
        default:
            msp430_error(true, "Invalid operand parsing option. Exiting.\n");
            break;
    }
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param ctx [description]
 */
static void msp430_fmt2_operand(MSP430CpuState *env,
                                DisasmContext *ctx)
{
    msp430_format2 *fmt = &ctx->fmt.type2;
    msp430_operand *dest = &fmt->dest;

    // Save the value of the register that is being used for the destination
    // register. This register can determine the type of address is being used
    // by the instruction and operand.
    dest->reg = (ctx->instr[0] & 0xF);
    msp430_handle_operand(env, ctx, dest, fmt->ad, true);
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param ctx [description]
 */
static void msp430_translate_format2(MSP430CpuState *env, DisasmContext *ctx)
{
    TCGv temp;
    uint16_t opcodeVal = 0;
    TranslationBlock *tb = ctx->tb;
    msp430_access_size access = 0;
    msp430_format2 *fmt = &ctx->fmt.type2;
    msp430_opcode opcodeTable[] = { RRC, SWPB, RRA, SXT, 
                                    PUSH, CALL, RETI, INVALID };
    //fprintf(stdout, "MSP430 Translate: Decoding Format 2 Instruction\n");

    // Update the formatting of the 
    fmt->bw = (ctx->instr[0] & 0x40) ? true : false;
    fmt->ad = (ctx->instr[0] & 0x30) >> 4;
    msp430_fmt2_operand(env, ctx);

    // Find the opcode of the instruction in order to further translate
    // the instruction.
    opcodeVal = ((ctx->instr[0] & 0x380) >> 7) & 0x7;
    ctx->opcode = opcodeTable[opcodeVal];

    // Look at the access bit and see whether we are accessing via
    // a byte size or a word size.
    // TODO: Fix access
    access = (fmt->bw == 1) ? ACCESS_BYTE : ACCESS_WORD;


    switch(ctx->opcode)
    {
        case SXT:
            msp430_error((fmt->bw == 1), "MSP430 Translate: Invalid Call instruction..\n");
            msp430_operand_preamble(&fmt->dest, access);
            tcg_gen_shli_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, 24);
            tcg_gen_sari_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, 24); 
            msp430_clear_flag(STATUS_VFLAG);
            msp430_check_flags((CHECK_ZFLAG | CHECK_NFLAG | CHECK_CFLAG_NOTZ),
                               access, fmt->dest.tcgVal);
            msp430_operand_postamble(ctx, &fmt->dest, access);
            break;

        case RRA:
            // Allocate Registers
            msp430_operand_preamble(&fmt->dest, access);
            temp = tcg_temp_new();

            // Set the C flag with the lowbit
            tcg_gen_andi_i32(temp, fmt->dest.tcgVal, 0x1);
            msp430_set_cflag(temp);

            // Capture High Bit, Shift Right by 1, and Set High Bit
            // to bit value it was previously
            tcg_gen_andi_i32(temp, fmt->dest.tcgVal, 
                            ((access == ACCESS_BYTE) ? 0x80 : 0x8000));
            tcg_gen_shri_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, 1);
            tcg_gen_or_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, temp);

            msp430_clear_flag(STATUS_VFLAG);
            msp430_check_flags((CHECK_ZFLAG | CHECK_NFLAG),
                                access, fmt->dest.tcgVal);

            // Free Registers
            tcg_temp_free_i32(temp);
            msp430_operand_postamble(ctx, &fmt->dest, access);

            // TODO: Check status flags
            break;

        case RRC:
        {
            TCGv cflag;
            
            // Allocate temporary values
            msp430_operand_preamble(&fmt->dest, access);
            temp = tcg_temp_new();
            cflag = tcg_temp_new();

            // Get the Current C Flag and shift it over to get the
            // or instruction ready
            msp430_get_cflag(cflag);
            tcg_gen_shli_i32(cflag, cflag, ((access == ACCESS_BYTE) ? 7 : 15));

            // Set the C flag based on what BIT0 is on the tcgVal
            // that is stored within the operand. Prepare the new bit
            // by shifting the C bit value over based on access size
            tcg_gen_andi_i32(temp, fmt->dest.tcgVal, 0x1);
            msp430_set_cflag(temp);

            // Shift the value by 1 and or the value with the adjusted
            // C flag bit.
            tcg_gen_shri_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, 1);
            tcg_gen_or_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, cflag);

            msp430_clear_flag(STATUS_VFLAG);
            msp430_check_flags((CHECK_ZFLAG | CHECK_NFLAG),
                                access, fmt->dest.tcgVal);

            // Free the allocated temporary values
            tcg_temp_free_i32(cflag);
            tcg_temp_free_i32(temp);
            msp430_operand_postamble(ctx, &fmt->dest, access);

            // TODO: Check status flags
            break;
        }
        case SWPB:
            // Check to see if BW is set to 0, if the bit is not set to zero,
            // then there is an invalid decoding of the instruction.
            msp430_error((fmt->bw == 1), "MSP430 Translate: Invalid SWP instruction..\n");
            msp430_operand_preamble(&fmt->dest, access);
            tcg_gen_bswap16_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, TCG_BSWAP_IZ); // TODO: ???
            msp430_operand_postamble(ctx, &fmt->dest, access);
            break;

        case CALL:
            // Check to see if BW is set to 0, if the bit is not set to zero,
            // then there is an invalid decoding of the instruction.
            msp430_error((fmt->bw == 1), "MSP430 Translate: Invalid Call instruction..\n");
            
            // Load preamble of the destination operand so we can use it
            // properly when the instruction is translated.
            msp430_operand_addr_preamble(&fmt->dest, access);
            tcg_gen_subi_i32(TCGV_REG(MSP430_SP_REGISTER),
                             TCGV_REG(MSP430_SP_REGISTER),
                             2);

            // Add the instruction size to the current PC and push it to
            // the stack. This will allow the CPU to keep track of where
            // to return from the stack.
            temp = tcg_temp_new();
            tcg_gen_addi_i32(temp, TCGV_REG(MSP430_PC_REGISTER), (ctx->instr_len << 1));
            tcg_gen_qemu_st16(temp, TCGV_REG(MSP430_SP_REGISTER), 0);
            tcg_temp_free_i32(temp);
            
            // Update the PC counter with the value stored in tcgVal. This will
            // hold the address of where to jump to.
            tcg_gen_mov_i32(TCGV_REG(MSP430_PC_REGISTER), fmt->dest.tcgVal);
            msp430_operand_addr_postamble(ctx, &fmt->dest, access);
            
            // Then the translation block since we called into a subroutine which
            // modified the PC register.
            tcg_gen_exit_tb(tb, 0);
            ctx->state = STATE_END;
            break;

        case PUSH:
            // TODO: Fix Reference Issue
            tcg_gen_subi_i32(TCGV_REG(MSP430_SP_REGISTER),
                             TCGV_REG(MSP430_SP_REGISTER),
                             2);
            msp430_operand_preamble(&fmt->dest, access);
            if (access == ACCESS_WORD)
                tcg_gen_qemu_st16(fmt->dest.tcgVal, TCGV_REG(MSP430_SP_REGISTER), 0);
            else
                tcg_gen_qemu_st8(fmt->dest.tcgVal, TCGV_REG(MSP430_SP_REGISTER), 0);
            msp430_operand_postamble(ctx, &fmt->dest, access);
            break;

        case RETI:
            // Check to see if BW is set to 0, if the bit is not set to zero,
            // then there is an invalid decoding of the instruction.
            msp430_error((access != ACCESS_WORD), "MSP430 Translate: Invalid RETI instruction..\n");

            // Pop the SR value from the stack into the 
            // SR register while performing the RETI.
            // Pop the PC value from the stack to the
            // PC register.
            temp = tcg_temp_new();
            tcg_gen_qemu_ld16u(temp, TCGV_REG(MSP430_SP_REGISTER), 0);
            tcg_gen_mov_i32(TCGV_REG(MSP430_SR_REGISTER), temp);
            tcg_gen_addi_i32(TCGV_REG(MSP430_SP_REGISTER),
                             TCGV_REG(MSP430_SP_REGISTER), 2);
            tcg_gen_qemu_ld16u(temp, TCGV_REG(MSP430_SP_REGISTER), 0);
            tcg_gen_mov_i32(TCGV_REG(MSP430_PC_REGISTER), temp);
            tcg_gen_addi_i32(TCGV_REG(MSP430_SP_REGISTER),
                             TCGV_REG(MSP430_SP_REGISTER), 2);
            
            // Mark as an end to the translation block. This allows us
            // break out and jump to another TranslationBlock.
            tcg_gen_exit_tb(tb, 0);
            tcg_temp_free_i32(temp);
            ctx->state = STATE_END;
            break;

        default:
            // TODO: Fill implementation.
            msp430_error(true, "Translation error for Single Operand Instructions. Exiting...\n");
    }

    return;
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param ctx [description]
 * @param isSrcOp [description]
 */
static void msp430_fmt1_operand(MSP430CpuState *env,
                                DisasmContext *ctx,
                                bool isSrcOp)
{
    msp430_format1 *fmt = &ctx->fmt.type1;
    msp430_operand *op;
    uint8_t switchValue;

    // Fetch the following register values in order to properly parse the
    // source or dest operand. This is done by checking what the input argument
    // specified on which operand to use. 
    if (isSrcOp)
    {
        // We are using the source operand. Fetch the correct operand
        // structure and fetch the appropriate bits.
        op = &fmt->src;
        op->reg = (ctx->instr[0] & 0x0F00) >> 8;
        switchValue = fmt->as;

    }
    else
    {
        // We are using the destination operand. Fetch the correct
        // operand structure and fetch the appropriate bits
        op = &fmt->dest;
        op->reg = (ctx->instr[0] & 0x000F);
        switchValue = ((fmt->ad) ? 1 : 0);
    }

    // Handle parsing the operand. This will allow up
    msp430_handle_operand(env, ctx, op, switchValue, isSrcOp);
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param ctx [description]
 */
static void msp430_translate_format1(MSP430CpuState *env, DisasmContext *ctx)
{
    msp430_format1 *fmt = &ctx->fmt.type1;
    uint8_t opcodeVal;
    TCGv temp;
    bool skipCheck = false;
    msp430_access_size access;

    msp430_flag_check mask = CHECK_ZCN;

    /* Capture the misc bits within the Format 1 instruction that determine 
     * the addressing of the instruction as well as bit width. */
    fmt->ad = (ctx->instr[0] & 0x0080) >> 7;
    fmt->as = (ctx->instr[0] & 0x0030) >> 4;
    fmt->bw = (ctx->instr[0] & 0x0040) >> 6;
    opcodeVal = (ctx->instr[0] & 0xF000) >> 12;
    access = ((fmt->bw) ? ACCESS_BYTE : ACCESS_WORD);

    /* Parse out the functions thatare defined within this instruction. For
     * format 1 instructions, we use two operands. One operand is destination
     * and one is source. */
    msp430_fmt1_operand(env, ctx, true);
    msp430_fmt1_operand(env, ctx, false);
    
    msp430_operand_preamble(&fmt->src, access);
    msp430_operand_preamble(&fmt->dest, access);
           
   switch(opcodeVal)
    {
       case 0x4:   // Move Instruction
           tcg_gen_mov_i32(fmt->dest.tcgVal, fmt->src.tcgVal);
           skipCheck = true;
           break;

        case 0xC:   // BIC Instruction
            temp = tcg_temp_new();
            tcg_gen_not_i32(temp, fmt->src.tcgVal);
            tcg_gen_and_i32(fmt->dest.tcgVal,
                            fmt->dest.tcgVal,
                            temp);
            tcg_temp_free_i32(temp);
            skipCheck = true;
            break;

        case 0xD:   // BIS Instruction
            tcg_gen_or_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, fmt->src.tcgVal);
            skipCheck = true;
            break;

        case 0x5:   // Add Instruction
            temp = tcg_temp_new();
            tcg_gen_xor_i32(temp, fmt->dest.tcgVal, fmt->src.tcgVal);
            tcg_gen_add_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, fmt->src.tcgVal);
            tcg_gen_xor_i32(temp, temp, fmt->dest.tcgVal);
            msp430_check_flags(CHECK_VFLAG, access, temp);
            tcg_temp_free_i32(temp);
            break;

        case 0x6:   // Addc Instruction
        {
            TCGv vflag = tcg_temp_new();
            temp = tcg_temp_new();

            msp430_get_cflag(temp);
            
            tcg_gen_xor_i32(vflag, fmt->dest.tcgVal, fmt->src.tcgVal);
            
            tcg_gen_add_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, temp);
            tcg_gen_add_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, fmt->src.tcgVal);
            
            tcg_gen_xor_i32(vflag, vflag, fmt->dest.tcgVal);
            msp430_check_flags(CHECK_VFLAG, access, vflag);

            tcg_temp_free_i32(temp);
            tcg_temp_free_i32(vflag);
            break;
        }

        case 0x7:   // SUBC instruction
        case 0x8:   // SUB Instruction
        case 0x9:   // CMP Instruction
        {
            TCGv temp2 = tcg_temp_new();
            TCGv result = tcg_temp_new();
            temp = tcg_temp_new();
            tcg_gen_sub_i32(result,
                            fmt->dest.tcgVal,
                            fmt->src.tcgVal);

            // If the opcode is SUBC, add the carry to the
            // result before analyzing the flags of each instruction
            if (opcodeVal == 0x7)
            {
                msp430_get_cflag(temp);
                tcg_gen_add_i32(result, result, temp);            
            }

            tcg_gen_xor_i32(temp, fmt->dest.tcgVal, fmt->src.tcgVal);
            tcg_gen_xor_i32(temp2, fmt->dest.tcgVal, result);
            tcg_gen_and_i32(temp, temp, temp2);
            msp430_check_flags(CHECK_VFLAG, access, temp);
            msp430_check_flags(CHECK_ZBN, access, result);
            skipCheck = true;

            // If the opcode is CMP, the result is not saved, else
            // generate a move instruction to move the result to the 
            // destination
            if(opcodeVal != 0x9)
                tcg_gen_mov_i32(fmt->dest.tcgVal, result);

            tcg_temp_free_i32(temp);
            tcg_temp_free_i32(temp2);
            tcg_temp_free_i32(result);
            break;
        }

        case 0xE:   // XOR Instruction
            temp = tcg_temp_new();
            tcg_gen_and_i32(temp, fmt->dest.tcgVal, fmt->src.tcgVal);
            tcg_gen_xor_i32(fmt->dest.tcgVal,
                            fmt->dest.tcgVal,
                            fmt->src.tcgVal);
            mask = (CHECK_ZFLAG | CHECK_NFLAG | CHECK_CFLAG_NOTZ);
            msp430_check_flags(CHECK_VFLAG, access, temp);
            tcg_temp_free_i32(temp);
            break;
        
        case 0xB:   // BIT Instruction
            temp = tcg_temp_new();
            tcg_gen_and_i32(temp,
                            fmt->dest.tcgVal,
                            fmt->src.tcgVal);
            
            mask = (CHECK_ZFLAG | CHECK_NFLAG | CHECK_CFLAG_NOTZ);
            msp430_clear_flag(STATUS_VFLAG);
            msp430_check_flags(mask, access, temp);
            
            tcg_temp_free_i32(temp);
            skipCheck = true;
            break;

        case 0xF:   // AND Instruction
            tcg_gen_and_i32(fmt->dest.tcgVal, fmt->dest.tcgVal, fmt->src.tcgVal);
            msp430_clear_flag(STATUS_VFLAG);
            mask = (CHECK_ZFLAG | CHECK_NFLAG | CHECK_CFLAG_NOTZ);
            break;

        case 0xA:   // DADD Instruction
            gen_helper_bcd_add(fmt->dest.tcgVal, cpu_env, fmt->dest.tcgVal, fmt->src.tcgVal);
            mask = CHECK_ZCN;
            break;

         /* Exit out since we have an invalid opcode value
          * that is currently being processed by our translation function
          * for format 1 instructions. */
         default:
            msp430_error(true, "MSP430 Format1: Invalid opcode value.");
            break;

    }

    if (!skipCheck)
        msp430_check_flags(mask, access, fmt->dest.tcgVal);

    // Specifying destination postamble after the source because there
    // is a chance that a destination register can be control flow changing.
    // So far, no Format I instruction is by definition a regular CF
    // instruction  (Call/Jump). Emulated instructions (Ret) emulate CF 
    // with Format1 instructions
    msp430_operand_postamble(ctx, &fmt->src, access);
    if(msp430_operand_postamble(ctx, &fmt->dest, access))
        ctx->state = STATE_END;

    return;
}

/**
 * @brief Handle jump instructions to jump. 
 * @details Handle jump instructions to jump. Inserts TCG instructions
 * which, for conditional jumps, inserts the evaluation of the conditional
 * checks, as well as the jump instructions and the end of a basic block
 * code when jumping out.
 * 
 * @param env [description]
 * @param ctx [description]
 */
static void msp430_translate_jump(MSP430CpuState *env, DisasmContext *ctx)
{
    uint32_t target;
    
    //fprintf(stdout, "MSP430 Translate: Jump Instruction.\n");
    ctx->fmt.jump.offset = SIGNEXT32(10, (ctx->instr[0] & 0x3FF));
    ctx->fmt.jump.cond = ((ctx->instr[0] >> 10) & 0x7);
    target = ctx->pc + (ctx->instr_len << 1) + (2 * ctx->fmt.jump.offset);
    
    /* Handle the additional instructions that are needed in order to 
     * properly execute whether the condition follows through or not. 
     * This involves checking the status values and jumping accordingly. */
    if (ctx->fmt.jump.cond != ALWAYS)
    {
        TCGLabel *label = gen_new_label();
        TCGv temp, temp2;
        temp = tcg_temp_local_new();
        switch(ctx->fmt.jump.cond)
        {
            case EQ: case C:
                tcg_gen_andi_tl(temp, TCGV_CPU_SR, 
                                ((ctx->fmt.jump.cond == C) ? STATUS_CFLAG : STATUS_ZFLAG));
                tcg_gen_brcondi_tl(TCG_COND_EQ, temp,
                                  ((ctx->fmt.jump.cond == C) ? STATUS_CFLAG : STATUS_ZFLAG),
                                    label);
                break;

            case NE: case NC:
                tcg_gen_andi_i32(temp, TCGV_CPU_SR,
                                ((ctx->fmt.jump.cond == NC) ? STATUS_CFLAG : STATUS_ZFLAG));
                tcg_gen_brcondi_i32(TCG_COND_NE, temp,
                                    ((ctx->fmt.jump.cond == NC) ? STATUS_CFLAG : STATUS_ZFLAG),
                                    label);
                break;

            case N:
                tcg_gen_andi_i32(temp, TCGV_CPU_SR, STATUS_NFLAG);
                tcg_gen_brcondi_i32(TCG_COND_EQ, temp, STATUS_NFLAG, label);
                break;

            case GE: case L:
                temp2 = tcg_temp_new();
                tcg_gen_andi_i32(temp, TCGV_CPU_SR, STATUS_NFLAG);
                tcg_gen_andi_i32(temp2, TCGV_CPU_SR, STATUS_VFLAG);
                tcg_gen_sari_i32(temp, temp, 2);
                tcg_gen_sari_i32(temp2, temp2, 8);
                tcg_gen_xor_i32(temp, temp, temp2);
                tcg_temp_free_i32(temp2);
                tcg_gen_brcondi_i32(TCG_COND_EQ, temp,
                                    ((ctx->fmt.jump.cond == L) ? 1 : 0),
                                    label);
                break;

            default:
                fprintf(stderr, "MSP430: Warning....Hit Condition that is not supported for jump.\n");
                break;
        }

        // Default Case, Jump to the next instruction in front of this
        // instruction. Set the label which generates the successful case 
        // where we jump to the target PC value if we satify the branch
        tcg_temp_free_i32(temp);
        msp430_exit_tb(ctx, (ctx->pc + (ctx->instr_len << 1)), 1);
        gen_set_label(label);

       // Set the value if we are jumping out to the target address that is 
       // specified by the branch/jump instruction.
       msp430_exit_tb(ctx, target, 0);
       ctx->state = STATE_END;
    }
    else
    {
        // Set the value if we are jumping out to the target address that is 
        // specified by the branch/jump instruction.
        msp430_exit_tb(ctx, target, 0);
        ctx->state = STATE_END;
    }
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param ctx [description]
 */
static void msp430_decode_opcode(MSP430CpuState *env, DisasmContext *ctx)
{

    /* Check to see if this is an extension word. If it is,
     * move it to the extension field and fetch the appropriate first word. */
    msp430_fetch_opcode_instr(env, ctx);
    if (ctx->rowIdx == 0x6 || ctx->rowIdx == 0x7)
    {
        /* Check to see if we hit a second extension instruction. If we did, exit out. This is
         * not suppose to happen. */
        if (ctx->instr_ext != NULL)
        {
            fprintf(stdout, "MSP430: Translate ERROR: Translating extra extension word for instruction.\n");
            exit(1);
        }

        /* Move the current instruction into the extension instruction location so we can store
         * the proper instruction value in index 0. */
        ctx->instr_ext = ctx->instr;
        msp430_fetch_opcode_instr(env, ctx);
    }
    //fprintf(stdout, "MSP430 Translate: Decoding: %08x | %08x\n", ctx->pc, ctx->instr[0]);

    /* Switch to context on which instruction group we need to work on. */
    switch (ctx->rowIdx)
    {
        // Handle the first row in the instruction mapping. In this map, we see how there 
        // are a bunch of instructions that grouped within this group.
        // Functions within this group:
        // - MOVA, CMPA, ADDA, SUBA, RRCM, RRAM, RLAM, RRUM
        case 0x0: case 0x1: case 0x2: case 0x3:
            fprintf(stdout, "MSP430 Translate (0x%04x): Handling Cluster 0\n", ctx->pc);
            break;

        // Handle the second cluster, which is located in the second row of the
        // instruction map table.
        // Opcodes within this group:
        // - RCC, SWP, RRA, SXT, PUSH, CALL, RETI, CALLA
        case 0x4:
            msp430_translate_format2(env, ctx);
            return;

        case 0x5:
            //fprintf(stdout, "MSP430 Translate: Push/Pop Group\n");
            break;

        case 0x6: case 0x7:
            //fprintf(stdout, "MSP430 Translate: Extension Group\n");
            break;

        // Handle the Jxx Conditional Group. These are instructions that perform
        // conditional jumps (conditional branches)
        case 0x8: case 0x9: case 0xA: case 0xB:
        case 0xC: case 0xD: case 0xE: case 0xF:
            msp430_translate_jump(env, ctx);
            return;

        case 0x10: case 0x11: case 0x12: case 0x13:
        case 0x14: case 0x15: case 0x16: case 0x17:
        case 0x18: case 0x19: case 0x1A: case 0x1B:
        case 0x1C: case 0x1D: case 0x1E: case 0x1F:
        case 0x20: case 0x21: case 0x22: case 0x23:
        case 0x24: case 0x25: case 0x26: case 0x27:
        case 0x28: case 0x29: case 0x2A: case 0x2B:
        case 0x2C: case 0x2D: case 0x2E: case 0x2F:
        case 0x30: case 0x31: case 0x32: case 0x33:
        case 0x34: case 0x35: case 0x36: case 0x37:
        case 0x38: case 0x39: case 0x3A: case 0x3B:
        case 0x3C: case 0x3D: case 0x3E: case 0x3F:
            msp430_translate_format1(env, ctx);
            return;


    }

    // TEMPORARY: Add SudoNop to fill in for now until implementation
    // is filled into the CPU.
    // TODO: Remove as implementation is being filled.
    tcg_gen_mov_i32(tcg_regs[10], tcg_regs[9]);
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param tb [description]
 * @param isSearchPc [description]
 */
static void msp430_generate_tcg_code(MSP430CpuState *env, TranslationBlock *tb, bool isSearchPc)
{
    DisasmContext ctx;
    //CPUState *cs = CPU(msp430_env_get_cpu(env));
    int32_t instrCount = 0;
    target_ulong pc_start, pc_idx;
    target_ulong blockSize = 0;

    /* Initialize the start of the translation block translation by setting the pc_idx
     * pointer to where PC start is located. PC start is found within the TranslationBlock's
     * PC member. */
    pc_idx = pc_start = tb->pc;

    /* Signal to the QEMU TCG engine that this translation engine will start creating 
     * instruction for this translation block. */
    gen_tb_start(tb);
    
    /* Loop through, translationing instructions, until we hit a control flow change 
     * within our translation block. */
    do
    {
        /* Initialize the Context of which we are in */
        memset(&ctx, 0, sizeof(DisasmContext));
        ctx.tb = tb;
        ctx.pc = pc_idx;
        ctx.singlestep_enabled = 1;
        
        /* Decode the instruction at the current PC location. */
        tcg_gen_movi_i32(TCGV_REG(MSP430_PC_REGISTER), ctx.pc);
        if (ctx.singlestep_enabled)
            msp430_raise_excp(MSP430_EXCP_DEBUG);
        msp430_decode_opcode(env, &ctx);
        blockSize += (ctx.instr_len << 1);
        pc_idx += (ctx.instr_len << 1);
        instrCount++;

        /* Close the TranslationBlock so it can used by QEMU to execute and store for 
         * possible caching */
        //msp430_exit_tb(tb, tb_pc_start + blockSize);
        
    } while (ctx.state == STATE_TRANSLATE);
    
    /* At this point, the translation engine has finished creating TCG instructions.
     * this will signal to QEMU TCG engine to close buffers and whatever else it needs
     * to do. */
    gen_tb_end(tb, instrCount);
    
    /* Set the size of the tb and the number of instructions that are contained
     * within this translation block. This allows us to define what was created
     * within the translation block. */
    //*tcg_ctx.gen_opc_ptr = INDEX_op_end;
    tb->size = blockSize;
    tb->icount = instrCount;
    return;
}


// ===============================================================================================
// Public Functions
// ===============================================================================================
/**
 * @brief Initializes the Translation Engine for the MSP430
 * @details Initializes the Translation engine for the MSP430. This function does the following:
 * - Initialize TCG variables (TCGv) to map to the related member within the CPU state so the TCG
 * engine knows where to store/fetch data from
 */
void msp430_tcg_init(void)
{
    int i;

    /* Need to initialize the cpu_env variable in order for the gen_tb_ functions
     * to work properly. */
    // cpu_env = tcg_global_mem_new_ptr(TCG_AREG0, "env");

    /* Initialize the TCG variables that will be used when generating new TCG instructions
     * for the MSP430. This will help us generate instructions quicker and faster. */
    for(i=0; i<MSP430_NUM_REGISTERS; i++)
        tcg_regs[i] = tcg_global_mem_new_i32(cpu_env,
                                             offsetof(MSP430CpuState, regs[i]),
                                             reg_names[i]);
}

// static const TranslatorOps msp430_tr_ops = {
//     .init_disas_context = msp430_tr_init_disas_context,
//     .tb_start           = msp430_tr_tb_start,
//     .insn_start         = msp430_tr_insn_start,
//     .translate_insn     = msp430_tr_translate_insn,
//     .tb_stop            = msp430_tr_tb_stop,
//     .disas_log          = msp430_tr_disas_log,
// };
// ===============================================================================================
// Required Functions for a Translation Engine for a CPU (MSP430).
//    These functions are called by QEMU during execution. These functions have to be 
//    implemented in order for the CPU to execute properly.
// ===============================================================================================
/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param TranslationBlock [description]
 */
void gen_intermediate_code(CPUState *cs, struct TranslationBlock *tb, int max_insns) 
{
    // DisasContext dc = {};
    MSP430Cpu *cpu = MSP430_CPU(cs);
    MSP430CpuState *env = &cpu->env;

    msp430_generate_tcg_code(env, tb, false);
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param env [description]
 * @param tb [description]
 * @param pc_pos [description]
 */
void restore_state_to_opc(MSP430CpuState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->regs[MSP430_PC_REGISTER] = data[0];
}
