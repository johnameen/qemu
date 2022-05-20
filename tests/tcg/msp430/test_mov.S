#define TEST(n) TEST##n:      mov #(n), r15
#define TEST_STORE(label, value)            mov value, r5           {\
                                            mov r5, label           {\
                                            clr r4                  {\
                                            TEST_LOAD(label, value)
#define TEST_LOAD(label, value)             mov label, r4           {\
                                            cmp value, r4           {\
                                            jne _exit
#define TEST_STOREB(label, value)           mov value, r5            {\
                                            mov.b r5, label          {\
                                            clr r4                   {\
                                            TEST_LOADB(label, value)
#define TEST_LOADB(label, value)            mov.b label, r4          {\
                                            cmp value, r4            {\
                                            jne _exit

.section .text,"ax",@progbits
STORAGE1:
.word 0xAAAA
.word 0xBBBB
.word 0xCCCC
.word 0xDDDD

###############################################################
# Test Source Only Operand Types
###############################################################
# Test Indirect Register AutoIncrement
TEST(5)
    mov #STORAGE, r10
    TEST_LOAD(@r10+, #0xAAAA)
    TEST_LOAD(@r10+, #0xBBBB)
    mov #STORAGE, r10
    TEST_LOADB(@r10+, #0xBBAA)
    TEST_LOADB(@r10+, #0xBBAA)
    TEST_LOADB(@r10+, #0xBBBB)
    TEST_LOADB(@r10+, #0xBBBB)
    jmp PASSED

# Test Indirect Register
TEST(4)
    mov #STORAGE, r10
    TEST_LOAD(@r10, #0xAAAA)
    mov #STORAGE+2, r10
    TEST_LOAD(@r10, #0xBBBB)
    mov #STORAGE+1, r10
    TEST_LOADB(@r10, #0xBBAA)
    mov #STORAGE+2, r10
    TEST_LOADB(@r10, #0xBBBB)
    jmp TEST5

# Test Immediate
# No Need to test this due to using this feature to test other
# operand types

###############################################################
# Test Source and Destination Operand Types
###############################################################
# Test Register Addressing
# No need to test this since we are already doing that
# in this or other tests

# Test Absolute Addressing
TEST(3)
    TEST_LOAD(&STORAGE, #0xAAAA)
    TEST_LOAD(&STORAGE+2, #0xBBBB)
    TEST_LOADB(&STORAGE+1, #0xBBAA)
    TEST_LOADB(&STORAGE+2, #0xBBBB)
    TEST_STORE(&STORAGE+4, #0x1234)
    TEST_STORE(&STORAGE+6, #0x9876)
    TEST_STOREB(&STORAGE+4, #0xAC)
    TEST_STOREB(&STORAGE+5, #0xDC)
    jmp TEST4


#Test Index Operand Type Loads and Stores
TEST(2)
    mov #STORAGE, r10
    TEST_LOAD(0(r10), #0xAAAA)
    TEST_LOAD(2(r10), #0xBBBB)
    TEST_LOADB(0(r10), #0xBBAA)
    TEST_LOADB(2(r10), #0xBBBB)
    TEST_STORE(4(r10), #0xDAAD)
    TEST_STORE(6(r10), #0xDEEF)
    TEST_STOREB(4(r10), #0xAD)
    TEST_STOREB(5(r10), #0x99)
    jmp TEST3

# Test Indirect Word and Byte Moves
TEST(1)
    mov #0xAAAA, &STORAGE
    mov #0xBBBB, &STORAGE+2
    mov #0xCCCC, &STORAGE+4
    mov #0xDDDD, &STORAGE+6

    TEST_LOAD(STORAGE1, #0xAAAA)
    TEST_LOAD(STORAGE1+2, #0xBBBB)
    TEST_LOADB(STORAGE1, #0xBBAA)
    TEST_LOADB(STORAGE1+2, #0xBBBB)
    # TEST_STORE(STORAGE+4, #0xBEEF)
    # TEST_STORE(STORAGE+6, #0xDEEF)
    # TEST_STOREB(STORAGE+4, #0xAD)
    # TEST_STOREB(STORAGE+5, #0x99)
    jmp TEST2


_start:
    jmp TEST1

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
