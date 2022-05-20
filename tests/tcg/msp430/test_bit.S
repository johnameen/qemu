#define TEST(n) TEST##n: mov #(n), r15

// Tests to see if the bit is cleared, if not, it will
// exit out with error.
#define TEST_BIT_CLEAR(n)     mov #0xAAAA, r4 { \
                              bit #(n), r4    { \
                              jne _exit
#define TEST_BITB_CLEAR(n)    mov #0xAAAA, r4 { \
                              bit.b #(n), r4  { \
                              jne _exit

// Tests to see if the bit is set, if not, it will
// exit out with error.
#define TEST_BIT_SET(n)       mov #0xAAAA, r4 { \
                              bit #(n), r4    { \
                              jz _exit
#define TEST_BITB_SET(n)      mov #0xAAAA, r4 { \
                              bit.b #(n), r4  { \
                              jz _exit

.section .text,"ax",@progbits
_start:
    // Test 1: Bitclear
    TEST(1)
        TEST_BIT_CLEAR(0x1)
        TEST_BIT_SET(0x2)
        TEST_BIT_CLEAR(0x4)
        TEST_BIT_SET(0x8)

        TEST_BIT_CLEAR(0x10)
        TEST_BIT_SET(0x20)
        TEST_BIT_CLEAR(0x40)
        TEST_BIT_SET(0x80)
        
        TEST_BIT_CLEAR(0x100)
        TEST_BIT_SET(0x200)
        TEST_BIT_CLEAR(0x400)
        TEST_BIT_SET(0x800)
        
        TEST_BIT_CLEAR(0x1000)
        TEST_BIT_SET(0x2000)
        TEST_BIT_CLEAR(0x4000)
        TEST_BIT_SET(0x8000)

    // Test 2: Bitclear
    TEST(2)
        TEST_BITB_CLEAR(0x1)
        TEST_BITB_SET(0x2)
        TEST_BITB_CLEAR(0x4)
        TEST_BITB_SET(0x8)

        TEST_BITB_CLEAR(0x10)
        TEST_BITB_SET(0x20)
        TEST_BITB_CLEAR(0x40)
        TEST_BITB_SET(0x80)

    PASSED:
        // Successful tests. Return 0
        mov #0, r15
        jmp _exit

;; This is where I will infinite loop on
_exit:
    bis #240, r2
    jmp _exit


;; Vector Table: This table is a list of function
;; pointers to allow the CPU to jump to certain locations
;; of code.
.section .vectors,"ax",@progbits
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _exit
.word _start
