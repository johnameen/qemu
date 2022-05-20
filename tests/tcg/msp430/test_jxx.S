#define TEST(n) TEST##n:      mov #(n), r15

.section .text,"ax",@progbits

TEST(10)
    mov #0xFFFF, r4
    mov #0xFFFE, r5
    cmp r5, r4
    jl _exit
    jge PASSED
    jmp _exit

TEST(9)
    mov #0xFFFF, r4
    mov #0xFFFE, r5
    cmp r4, r5
    jge _exit
    jl TEST10
    jmp _exit

TEST(8)
    mov #0x10, r4
    mov #0x20, r5
    cmp r5, r4
    jhs _exit
    jlo TEST9

TEST(7)
    mov #0x10, r4
    mov #0x20, r5
    cmp r4, r5
    jlo _exit
    jhs TEST8

TEST(6)
    clrn
    jn _exit
    setn 
    jn TEST7

TEST(5)
    clrc
    jc _exit
    jnc TEST6
    jmp _exit

TEST(4)
    setc
    jnc _exit
    jc TEST5
    jmp _exit

TEST(3)
    mov #0x2, r4
    mov #0x1, r5
    cmp r4, r5
    jz _exit
    jnz TEST4
    jmp _exit 

TEST(2)
    mov #0x1, r4
    mov #0x1, r5
    cmp r4, r5
    jnz _exit
    jz TEST3
    jmp _exit 

TEST(1)
    mov #0x1, r4
    mov #0x1, r5
    cmp r4, r5
    jne _exit
    jeq TEST2
    jmp _exit

_start:
    mov #0x1, r4
    mov #0x2, r5
    cmp r4, r5
    jeq _exit
    jnz TEST1
    jmp _exit

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
