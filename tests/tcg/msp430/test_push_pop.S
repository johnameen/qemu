#define TEST(n) TEST##n: mov #(n), r15

.section .text,"ax",@progbits
_start:
    mov #0x27E, r1
    TEST1:
        mov #0xDEAD, r4
        mov #0xBEEF, r5
        mov #0xDEAD, r6
        mov #0xBEEF, r7
        push r4
        push r5
        pop r8
        pop r9
        cmp r7, r8
        jne _exit
        cmp r6, r9
        jne _exit
        mov #0x27E, r10
        cmp r10, r2
        jne _exit

    TEST2:
        mov #0xDEAD, r4
        mov #0xBEEF, r5
        mov #0xDEEF, r6
        mov #0xBEAD, r7
        push.b r4
        push.b r5
        pop.b r4
        pop.b r5
        cmp r6, r4
        jne _exit
        cmp r7, r5
        jne _exit

    PASSED:
        mov #0, r15

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

.section .ram
stack_buffer:
    .space 32
    .word 16
stack_start:
