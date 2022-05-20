#ifndef __MSP430_TRANSLATE_H__
#define __MSP430_TRANSLATE_H__

#include "exec/exec-all.h"
#define MSP430_MAX_INSTR_WORDS   (4)

/**
 * @brief Sign extends val from the number of bits in bits to 16 bits
 * @details Helper macro to sign extend a specific signed size value to a
 * 16bit signed value
 * 
 * @param bits Bitsize of value currently
 * @param val Value to sign extend
 * @return Value sign extended to int16_t.
 */
#define SIGNEXT16(bits, val) ((int16_t)(val << (16-bits)) >> (16-bits))
#define SIGNEXT32(bits, val) ((int32_t)(val << (32-bits)) >> (32-bits))

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
void debug_print_translation_block(TranslationBlock *tb);

typedef struct
{
    TCGv tcgVal;                // Placeholder for the TCGv value that represents this
                                // operand after a preamble.
    msp430_addr_mode type;      // Type of operand.
    int32_t imm;               // Immediate Value
    uint8_t reg;                // Register Number
    bool isSrcOp;               // Is Source Operand or Not
} msp430_operand;

/***************************************************************************
 * Instruction Format Structures: These are the structures that are
 * used to store instruction type and relevant information about the 
 * structure at hand.
 **************************************************************************/
typedef struct 
{
    msp430_operand dest;
    msp430_operand src;
    uint8_t as;
    bool ad;
    bool bw;
} msp430_format1;

typedef struct 
{
    msp430_operand dest;
    uint8_t ad;
    bool bw;
} msp430_format2;

// MSP430 Format Jump Instruction Fields: This structure holds 
// all the data within the MSP430 Format Jump Instruction, such as
// the Offset value and Conditional Fields for the type of 
// jump to perform.
typedef struct
{
    MSP430_COND cond;
    int32_t offset;
} msp430_format_jump;

typedef union 
{
    msp430_format1 type1;
    msp430_format2 type2;
    msp430_format_jump jump;
} msp430_format;

/***************************************************************************
 **************************************************************************/
/* */
typedef enum 
{
    STATE_TRANSLATE = 0,
    STATE_END = 1,
} ContextState;

typedef struct 
{
    TranslationBlock *tb;
    ContextState state;
    uint16_t instr_buf[MSP430_MAX_INSTR_WORDS];      // Container to hold the instruction binaries
    uint16_t *instr;                                 // Pointer to the base of the instruction
    uint16_t *instr_ext;                             // Pointer to the base of the extension instruction
    size_t instr_len;                                // 16 Bit Length of the instruction. To get byte size,
                                                     // multiply this size by 2.
    uint8_t rowIdx, colIdx;                          // Index Values that are used to determine 
    target_ulong pc;                                 // PC Address we are currently at.
    msp430_opcode opcode;
    msp430_access_size access;
    msp430_format fmt;

    int singlestep_enabled;
} DisasmContext;



#endif
