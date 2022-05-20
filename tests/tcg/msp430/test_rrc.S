#define TEST(n) TEST##n: mov #(n), r15

#define TEST_CLR_RRC(n) clrc         { \
                        TEST_RRC(n)

#define TEST_SET_RRC(n) setc         { \
                        TEST_RRC(n)

#define TEST_RRC(n)     rrc r4       { \
                        mov #(n), r5 { \
                        cmp r4, r5   { \
                        jne _exit

.section .text,"ax",@progbits
_start:
    // Check to see if each shift will output the
    // correct value for each shift
    TEST(1)
        mov #0, r5
        mov #0xFFFF, r6
        mov #0xFFFF, r4
        LOOP1:
            setc
            rrc r4
            cmp r4, r6
            jne _exit
            add #1, r5
            cmp #16, r5
            jne LOOP1

    // Check to see if each shift will output the
    // correct value for each shift
    TEST(2)
        mov #0xFFFF, r4
        TEST_CLR_RRC(0x7FFF)
        TEST_CLR_RRC(0x3FFF)
        TEST_CLR_RRC(0x1FFF)
        TEST_CLR_RRC(0xFFF)
        TEST_CLR_RRC(0x7FF)
        TEST_CLR_RRC(0x3FF)
        TEST_CLR_RRC(0x1FF)
        TEST_CLR_RRC(0xFF)
        TEST_CLR_RRC(0x7F)
        TEST_CLR_RRC(0x3F)
        TEST_CLR_RRC(0x1F)
        TEST_CLR_RRC(0xF)
        TEST_CLR_RRC(0x7)
        TEST_CLR_RRC(0x3)
        TEST_CLR_RRC(0x1)
        TEST_CLR_RRC(0x0)

    // Check to see if each shift will output the
    // correct value for each shift
    TEST(3)
        mov #0x7FFF, r4
        TEST_SET_RRC(0xBFFF)

    TEST(4)
        mov #0x0001, r4
        setc
        rrc r4
        jnc _exit
        mov #0x8000, r5
        cmp r4, r5
        jne _exit

    TEST(5)
        mov #0x0001, r4
        clrc
        rrc r4
        jnc _exit
        mov #0x0000, r5
        cmp r4, r5
        jne _exit

    TEST(6)
        mov #0x0001, r4
        setc
        rrc r4
        jn TEST7
        jmp _exit

    TEST(7)
        mov #0x0001, r4
        clrc
        rrc r4
        jn _exit

    TEST(8)
        mov #0x0001, r4
        setc
        rrc r4
        jz _exit

    TEST(9)
        mov #0x0001, r4
        clrc
        rrc r4
        jnz _exit


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
