#define TEST(n) TEST##n: mov #(n), r15
#define TEST_RRA(n)     rra.b r4     { \
                        mov #(n), r5 { \
                        cmp r4, r5   { \
                        jne _exit
#define TEST_RRA_CFLAG_SET       rra.b r4  {\
                                 jnc _exit
#define TEST_RRA_CFLAG_CLEAR     rra.b r4  {\
                                 jc _exit

.section .text,"ax",@progbits
_start:
    // Check to see if each shift will output the
    // correct value for each shift
    TEST(1)
        mov #0, r5
        mov #0xFFFF, r6
        setc
        LOOP1:
            mov #0xFFFF, r4
            rra.b r4
            cmp r4, r6
            jne _exit
            add #1, r5
            cmp #16, r5
            jne LOOP1

    // Check to see if each shift will output the
    // correct value for each shift
    TEST(2)
        mov #0xAB7F, r4
        TEST_RRA(0xAB3F)
        TEST_RRA(0xAB1F)
        TEST_RRA(0xAB0F)
        TEST_RRA(0xAB07)
        TEST_RRA(0xAB03)
        TEST_RRA(0xAB01)
        TEST_RRA(0xAB00)

    // Test to see if the negative flag gets set 
    // within each RRA
    TEST(3)
        mov #0x0EFF, r4
        clrn
        rra.b r4
        jn TEST4
        jmp _exit

    // Test to see if the negative flag is cleared
    // within each RRA
    TEST(4)
        mov #0x8E7F, r4
        setn
        rra.b r4
        jn _exit 

    // Test to see if the zero flag is setting and 
    // clearing properly for each rra
    TEST(5)
        mov #0xAD03, r4
        setz
        rra.b r4
        jz _exit
        clrz
        rra.b r4
        jnz _exit

    TEST(6)
        mov #0xAAAA, r4
        setc
        // Nibble 0
        TEST_RRA_CFLAG_CLEAR
        TEST_RRA_CFLAG_SET
        TEST_RRA_CFLAG_CLEAR
        TEST_RRA_CFLAG_SET

        // Nibble 1
        TEST_RRA_CFLAG_CLEAR
        TEST_RRA_CFLAG_SET
        TEST_RRA_CFLAG_CLEAR
        TEST_RRA_CFLAG_SET

        mov #0xAAFF, r5
        cmp r4, r5
        jne _exit


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
