#define TEST(n) TEST##n: mov #(n), r15
#define BIT_SET(v)     mov #0x0000, r4   { \
                       bis #(v), r4      { \
                       mov #(v), r5      { \
                       cmp r4, r5        { \
                       jne _exit
#define BITB_SET(n, m)    mov #0xAD00, r4 { \
                          bis.b #(n), r4  { \
                          mov #(m), r5    { \
                          cmp r4, r5      { \
                          jne _exit

.section .text,"ax",@progbits
_start:
    // Test 1: Bitclear
    TEST(1)
        BIT_SET(0x1)
        BIT_SET(0x2)
        BIT_SET(0x4)
        BIT_SET(0x8)
        BIT_SET(0x10)
        BIT_SET(0x20)
        BIT_SET(0x40)
        BIT_SET(0x80)
        BIT_SET(0x100)
        BIT_SET(0x200)
        BIT_SET(0x400)
        BIT_SET(0x800)
        BIT_SET(0x1000)
        BIT_SET(0x2000)
        BIT_SET(0x4000)
        BIT_SET(0x8000)

    // Test 2: Bitclear
    TEST(2)
        BITB_SET(0x1, 0xAD01)
        BITB_SET(0x2, 0xAD02)
        BITB_SET(0x4, 0xAD04)
        BITB_SET(0x8, 0xAD08)
        BITB_SET(0x10, 0xAD10)
        BITB_SET(0x20, 0xAD20)
        BITB_SET(0x40, 0xAD40)
        BITB_SET(0x80, 0xAD80)

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
