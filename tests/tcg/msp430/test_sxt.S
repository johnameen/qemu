#define TEST(n) TEST##n: mov #(n), r15

.section .text,"ax",@progbits
_start:
    TEST(1)
        mov.b #0xa, r4
        mov #0xa, r5
        sxt r4
        cmp r4, r5
        jne _exit

    TEST(2)
        mov.b #0x00aa, r4
        mov #0xffaa, r5
        sxt r4
        cmp r4, r5
        jne _exit

    TEST(3)
        mov #0xCCaa, r4
        mov #0xffaa, r5
        sxt r4
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
