#define TEST(n) TEST##n: mov #(n), r15

.section .text,"ax",@progbits
_start:
    TEST(1)
        mov #0x1111, r4
        mov #0x2222, r5
        mov #0x2233, r6
        dadd.b r4, r5
        cmp r5, r6
        jne _exit

    TEST(2)
        mov #0x1099, r4
        mov #0x1099, r5
        mov #0x1098, r6
        dadd.b r4, r5
        cmp r5, r6
        jne _exit

    TEST(3)
        mov #0x1099, r4
        mov #0x1099, r5
        clrc
        dadd.b r4, r5
        jnc _exit

    TEST(4)
        mov #0x1099, r4
        mov #0x1099, r5
        clrn
        dadd r4, r5
        jn TEST5

    TEST(5)
        mov #0x1099, r4
        mov #0x0001, r5
        clrz
        dadd r4, r5
        jz PASSED
    
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
