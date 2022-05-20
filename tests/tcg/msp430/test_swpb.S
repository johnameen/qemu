#define TEST(n) TEST##n: mov #(n), r15

.section .text,"ax",@progbits
_start:
    // Check to see if each shift will output the
    // correct value for each shift
    TEST(1)
        mov #0xDEAD, r4
        mov #0xDEAD, r5
        mov #0xADDE, r6
        swpb r4
        cmp r4, r6
        jne _exit
        swpb r4
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
