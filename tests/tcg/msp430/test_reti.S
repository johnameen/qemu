#define TEST(n) TEST##n:      cmp #(n), r15 { \
                              jne _exit
#define TEST_CALL(n, target)  mov #(n), r15 { \
                              call target

.section .text,"ax",@progbits

test_regular_call:
    ret

test_checked_call:
    cmp #10, r15
    jne _exit
    ret

TEST(1)
    mov #0x27C, r4
    cmp r1, r4
    jne _exit
    push #0x0007
    reti

_start:
    mov #0x27E, r1
    TEST_CALL(1, #TEST1)
    cmp #0x0007, r2
    jne _exit
    mov #2, r15

    // Test Symbolic Addressing: x(PC)
    call test_regular_call

    // Test Absolute Addressing: &ADDR
    call &test_regular_call

    // Test Immediate Addressing: #ADDR
    call #test_regular_call

    // Test Register Based Calling: Rn
    mov PTR_FUNCTION, r4
    call r4

    // Test Indirect Reg Based Calling: @Rn
    call @r4


    // Test indirect based calling: x(Rn)
    mov #10, r15
    call 2(r4)


PASSED:
    mov #0, r15

;; This is where I will infinite loop on
_exit:
    bis #240, r2
    jmp _exit


PTR_FUNCTION:
    .word test_regular_call
    .word test_checked_call


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
