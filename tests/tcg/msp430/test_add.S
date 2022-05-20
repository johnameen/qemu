.section .text,"ax",@progbits
_start:
    // Test 1: Simple Add
    TEST1:
        mov #1, r15
        mov #0x100, r4
        mov #0x200, r5
        mov #0x300, r6
        add r4, r5
        cmp r5, r6
        jne _exit

    // Test 2: Overflow Add - Check result
    TEST2:
        mov #2, r15
        mov #0xFF00, r4
        mov #0x0100, r5
        mov #0x0000, r6
        add r4, r5
        cmp r5, r6
        jne _exit

    // Test 3: Check Zero Flag from ADD
    TEST3:
        mov #3, r15
        mov #0xFF00, r4
        mov #0x0100, r5
        add r4, r5
        jz TEST4
        jmp _exit

    // Test 4: Check Carry Flag from ADD
    TEST4:
        mov #4, r15
        mov #0xFF00, r4
        mov #0x0100, r5
        add r4, r5
        jc TEST5
        jmp _exit


    // Test 5: Check Carry Flag from ADD
    TEST5:
        mov #5, r15
        mov #0xF000, r4
        mov #0x0100, r5
        add r4, r5
        jn TEST6
        jmp _exit

    // Test 6: Check V Flag 
    TEST6:
        mov #0x7FFF, r4
        mov #0x0001, r5
        add r4, r5
        mov r2, r6
        and #0x100, r6
        jz _exit
        jnz TEST7
        jmp _exit

    TEST7:
        mov #0x7F00, r4
        mov #0x0001, r5
        add r4, r5
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
