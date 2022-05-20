#define TEST(n) TEST##n: mov #(n), r15

.section .text,"ax",@progbits
_start:
    // Test 1: Simple Add
    TEST(1)
        mov #0x0010, r4
        mov #0x0020, r5
        mov #0x0030, r6
        add.b r4, r5
        cmp.b r5, r6
        jne _exit

    // Test 2: Overflow Add - Check result
    TEST(2)
        mov #0x00FF, r4
        mov #0x0001, r5
        mov #0x0000, r6
        add.b r4, r5
        cmp r5, r6
        jne _exit

    // Test 3: Overflow Add - Check result
    TEST(3)
        mov #0x00FF, r4
        mov #0xAA01, r5
        mov #0xAA00, r6
        add.b r4, r5
        cmp r5, r6
        jne _exit

    // Test 4: Check Zero Flag from ADD
    TEST(4)
        mov #0xFF00, r4
        mov #0x0100, r5
        add r4, r5
        jz TEST5
        jmp _exit

    // Test 5: Check Carry Flag from ADD
    TEST(5)
        mov #0xFF00, r4
        mov #0x0100, r5
        add r4, r5
        jc TEST6
        jmp _exit


    // Test 6: Check Carry Flag from ADD
    TEST(6)
        mov #0xF000, r4
        mov #0x0100, r5
        add r4, r5
        jn TEST7
        jmp _exit

    // Test 7: Check V Flag 
    TEST(7)
        mov #0x007F, r4
        mov #0x0001, r5
        add.b r4, r5
        mov r2, r6
        and #0x100, r6
        jz _exit
        jnz TEST8
        jmp _exit

    TEST(8)
        mov #0xFF70, r4
        mov #0xFF01, r5
        add.b r4, r5
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
