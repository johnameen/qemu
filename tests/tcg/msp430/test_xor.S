#define TEST(n) TEST##n: mov #(n), r15
#define XOR_SET(n)          mov #0x0, r4     {\
                            xor #(n), r4     {\
                            mov #(n), r5     {\
                            cmp r4, r5       {\
                            jne _exit
#define XORB_SET(n)         mov #0x0, r4     {\
                            xor.b #(n), r4   {\
                            mov #(n), r5     {\
                            cmp r4, r5       {\
                            jne _exit

#define XOR_CLEAR(n, m)     mov #0xFFFF, r4  {\
                            xor #(n), r4     {\
                            mov #(m), r5     {\
                            cmp r4, r5       {\
                            jne _exit
#define XORB_CLEAR(n, m)     mov #0xFFFF, r4 {\
                            xor.b #(n), r4    {\
                            mov #(m), r5     {\
                            cmp r4, r5       {\
                            jne _exit

.section .text,"ax",@progbits
_start:
    TEST(1)
        XOR_SET(0x1)
        XOR_SET(0x2)
        XOR_SET(0x4)
        XOR_SET(0x8)
        XOR_SET(0x10)
        XOR_SET(0x20)
        XOR_SET(0x40)
        XOR_SET(0x80)
        XOR_SET(0x100)
        XOR_SET(0x200)
        XOR_SET(0x400)
        XOR_SET(0x800)
        XOR_SET(0x1000)
        XOR_SET(0x2000)
        XOR_SET(0x4000)
        XOR_SET(0x8000)

    TEST(2)
        XOR_CLEAR(0x1, 0xFFFE)
        XOR_CLEAR(0x2, 0xFFFD)
        XOR_CLEAR(0x4, 0xFFFB)
        XOR_CLEAR(0x8, 0xFFF7)
        XOR_CLEAR(0x10, 0xFFEF)
        XOR_CLEAR(0x20, 0xFFDF)
        XOR_CLEAR(0x40, 0xFFBF)
        XOR_CLEAR(0x80, 0xFF7F)
        XOR_CLEAR(0x100, 0xFEFF)
        XOR_CLEAR(0x200, 0xFDFF)
        XOR_CLEAR(0x400, 0xFBFF)
        XOR_CLEAR(0x800, 0xF7FF)
        XOR_CLEAR(0x1000, 0xEFFF)
        XOR_CLEAR(0x2000, 0xDFFF)
        XOR_CLEAR(0x4000, 0xBFFF)
        XOR_CLEAR(0x8000, 0x7FFF)

    TEST(3)
        XORB_SET(0x1)
        XORB_SET(0x2)
        XORB_SET(0x4)
        XORB_SET(0x8)
        XORB_SET(0x10)
        XORB_SET(0x20)
        XORB_SET(0x40)
        XORB_SET(0x80)
        XORB_CLEAR(0x1, 0xFFFE)
        XORB_CLEAR(0x2, 0xFFFD)
        XORB_CLEAR(0x4, 0xFFFB)
        XORB_CLEAR(0x8, 0xFFF7)
        XORB_CLEAR(0x10, 0xFFEF)
        XORB_CLEAR(0x20, 0xFFDF)
        XORB_CLEAR(0x40, 0xFFBF)
        XORB_CLEAR(0x80, 0xFF7F)

    TEST(4)
        mov #0xF000, r4
        mov #0xC000, r5
        xor r4, r5
        mov r2, r6
        and #0x100, r6
        jz _exit
        jnz TEST5
        jmp _exit

    TEST(5)
        mov #0x0000, r4
        mov #0xC000, r5
        xor r4, r5
        mov r2, r6
        and #0x100, r6
        jnz _exit
        jz TEST6
        jmp _exit

    TEST(6)
        mov #0x00F0, r4
        mov #0xAAC0, r5
        xor.b r4, r5
        mov r2, r6
        and #0x100, r6
        jz _exit
        jnz TEST7
        jmp _exit


    TEST(7)
        mov #0x00, r4
        mov #0xC0, r5
        xor.b r4, r5
        mov r2, r6
        and #0x100, r6
        jnz _exit
        jz PASSED
        jmp _exit



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
