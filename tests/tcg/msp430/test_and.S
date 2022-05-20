.section .text,"ax",@progbits
_start:
    // Test 1: Simple And
    TEST1:
        mov #1, r15
        mov #0xFFFF, r4
        mov #0x0001, r5
        mov #0x0001, r6
        and r4, r5
        cmp r5, r6
        jne _exit

    // Test 2: Simple And
    TEST2:
        mov #2, r15
        mov #0xFFFF, r4
        mov #0xBEEF, r5
        mov #0xBEEF, r6
        and r4, r5
        cmp r5, r6
        jne _exit

    // Test 3: Check Zero Flag from AND
    TEST3:
        mov #3, r15
        mov #0xFFFE, r4
        mov #0x0001, r5
        and r4, r5
        jz TEST4
        jmp _exit

    // Test 4: Check Carry Flag from AND
    TEST4:
        mov #4, r15
        mov #0xFFFF, r4
        mov #0x0100, r5
        and r4, r5
        jc TEST5
        jmp _exit

    // Test 5: Check Negative Flag from AND
    TEST5:
        mov #5, r15
        mov #0xFFFF, r4
        mov #0x8000, r5
        and r4, r5
        jn PASSED
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
