#define TEST(n) TEST##n: mov #(n), r15
#define TEST_SUB(d,s)      mov s, r4 {\
                           mov d, r5 {\
                           sub r4, r5
#define TEST_SUBB(d,s)     mov s, r4 {\
                           mov d, r5 {\
                           sub.b r4, r5
#define TEST_SUBC(d,s)     mov s, r4 {\
                           mov d, r5 {\
                           subc r4, r5
#define TEST_SUBCB(d,s)    mov s, r4 {\
                           mov d, r5 {\
                           subc.b r4, r5

#define TEST_SUBR(d,s,r)    mov r, r6 {\
                           TEST_SUB(d,s)
#define TEST_SUBBR(d,s,r)   mov r, r6 {\
                           TEST_SUBB(d,s)
#define TEST_SUBCR(d,s,r)   mov r, r6 {\
                           TEST_SUBC(d,s)
#define TEST_SUBCBR(d,s,r)  mov r, r6 {\
                           TEST_SUBCB(d,s)


.section .text,"ax",@progbits
    // Test V Flag
    TEST(16)
        TEST_SUB(#0x0001, #0xFFFF)
        mov r2, r6
        and #0x100, r6
        jnz _exit

        TEST_SUBB(#0xAA01, #0xAAFF)
        mov r2, r6
        and #0x100, r6
        jnz _exit

        setc
        TEST_SUBC(#0x0001, #0xFFFF)
        mov r2, r6
        and #0x100, r6
        jnz _exit

        setc
        TEST_SUBCB(#0xCC01, #0xAAFF)
        mov r2, r6
        and #0x100, r6
        jnz _exit

        jmp PASSED

    TEST(15)
        TEST_SUB(#0x7FFF, #0xFFFF)
        mov r2, r6
        and #0x100, r6
        jz _exit

        TEST_SUBB(#0xAA7F, #0xAAFF)
        mov r2, r6
        and #0x100, r6
        jz _exit

        setc
        TEST_SUBC(#0x7FFE, #0xFFFF)
        mov r2, r6
        and #0x100, r6
        jz _exit

        setc
        TEST_SUBCB(#0xCC7E, #0xAAFF)
        mov r2, r6
        and #0x100, r6
        jz _exit

        jmp TEST16

    // Test C Flag
    TEST(14)
        TEST_SUB(#0x0010, #0x0012)
        jc _exit
        TEST_SUBB(#0x0010, #0x0012)
        jc _exit
        TEST_SUBC(#0x0010, #0x0012)
        jc _exit
        TEST_SUBCB(#0x0010, #0x0012)
        jc _exit
        jmp TEST15

    TEST(13)
        TEST_SUB(#0x0010, #0x000F)
        jnc _exit

        TEST_SUBB(#0xAA10, #0x000F)
        jnc _exit

        TEST_SUBC(#0x0010, #0x000F)
        jnc _exit

        TEST_SUBCB(#0xAA10, #0x000F)
        jnc _exit

        jmp TEST14

    // Test Z Flag
    TEST(12)
        TEST_SUB(#0x0010, #0x0012)
        jz 0x0, _exit
        TEST_SUBB(#0x0010, #0x0012)
        jz 0x0, _exit
        
        setc
        TEST_SUBC(#0x0010, #0x0012)
        jz 0x0, _exit
        setc
        TEST_SUBCB(#0x0010, #0x0012)
        jz 0x0, _exit

        jmp TEST13

    TEST(11)
        TEST_SUB(#0x0010, #0x0010)
        jnz _exit

        TEST_SUBB(#0xAA10, #0x0010)
        jnz _exit

        setc
        TEST_SUBC(#0x0010, #0x0011)
        jnz _exit

        setc
        TEST_SUBCB(#0xAA10, #0x0011)
        jnz _exit

        jmp TEST12

    // Test N Flag
    TEST(10)
        TEST_SUB(#0x0010, #0x0012)
        jge 0x0, _exit
        TEST_SUBC(#0x0010, #0x0012)
        jge 0x0, _exit
        TEST_SUBB(#0x0010, #0x0012)
        jge 0x0, _exit
        TEST_SUBCB(#0x0010, #0x0012)
        jge 0x0, _exit
        jmp TEST11

    TEST(9)
        TEST_SUB(#0x0010, #0x0009)
        jn _exit
        TEST_SUBC(#0x0010, #0x0009)
        jn _exit
        TEST_SUBB(#0xAA10, #0x0009)
        jn _exit
        TEST_SUBCB(#0xAA10, #0x0009)
        jn _exit
        jmp TEST10

    TEST(8)
        setc
        TEST_SUBCBR(#0xAD13,#0x15,#0xADFF)
        cmp r6, r5
        jne _exit
        jeq TEST9
        jmp _exit

    TEST(7)
        setc
        TEST_SUBCBR(#0x1114,#0x4,#0x1111)
        cmp r6, r5
        jne _exit
        jeq TEST8
        jmp _exit

    TEST(6)
        setc
        TEST_SUBCR(#0x13,#0x15,#0xFFFF)
        cmp r6, r5
        jne _exit
        jeq TEST7
        jmp _exit

    TEST(5)
        setc
        TEST_SUBCR(#0x14,#0x4,#0x11)
        cmp r6, r5
        jne _exit
        jeq TEST6
        jmp _exit

    TEST(4)
        TEST_SUBBR(#0xAD13,#0x14,#0xADFF)
        cmp r6, r5
        jne _exit
        jeq TEST5
        jmp _exit

    TEST(3)
        TEST_SUBBR(#0x1114,#0x4,#0x1110)
        cmp r6, r5
        jne _exit
        jeq TEST4
        jmp _exit

    TEST(2)
        TEST_SUBR(#0x13,#0x14,#0xFFFF)
        cmp r6, r5
        jne _exit
        jeq TEST3
        jmp _exit

_start:
    TEST(1)
        TEST_SUBR(#0x14,#0x4,#0x10)
        cmp r6, r5
        jne _exit
        jeq TEST2
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
