#define TEST(n) TEST##n: mov # 1, r15
#define BIT_CLEAR(n, m)     mov #0xFFFF, r4 { \
                            bic #(n), r4    { \
                            mov #(m), r5    { \
                            cmp r4, r5      { \
                            jne _exit
#define BITB_CLEAR(n, m)     mov #0xFFFF, r4 { \
                             bic.b #(n), r4  { \
                             mov #(m), r5    { \
                             cmp r4, r5      { \
                             jne _exit

.section .text,"ax",@progbits
_start:
    // Test 1: Bitclear
    TEST(1)
        BIT_CLEAR(0x1, 0xFFFE)
        BIT_CLEAR(0x2, 0xFFFD)
        BIT_CLEAR(0x4, 0xFFFB)
        BIT_CLEAR(0x8, 0xFFF7)
        BIT_CLEAR(0x10, 0xFFEF)
        BIT_CLEAR(0x20, 0xFFDF)
        BIT_CLEAR(0x40, 0xFFBF)
        BIT_CLEAR(0x80, 0xFF7F)
        BIT_CLEAR(0x100, 0xFEFF)
        BIT_CLEAR(0x200, 0xFDFF)
        BIT_CLEAR(0x400, 0xFBFF)
        BIT_CLEAR(0x800, 0xF7FF)
        BIT_CLEAR(0x1000, 0xEFFF)
        BIT_CLEAR(0x2000, 0xDFFF)
        BIT_CLEAR(0x4000, 0xBFFF)
        BIT_CLEAR(0x8000, 0x7FFF)

    // Test 2: Bitclear
    TEST(2)
        BITB_CLEAR(0x1, 0xFFFE)
        BITB_CLEAR(0x2, 0xFFFD)
        BITB_CLEAR(0x4, 0xFFFB)
        BITB_CLEAR(0x8, 0xFFF7)
        BITB_CLEAR(0x10, 0xFFEF)
        BITB_CLEAR(0x20, 0xFFDF)
        BITB_CLEAR(0x40, 0xFFBF)
        BITB_CLEAR(0x80, 0xFF7F)
    

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
