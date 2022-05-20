#define TEST(n) TEST##n: mov #(n), r15
.section .text,"ax",@progbits

_start:
    // Test 1: Simple Addc
    TEST(1)
        mov #0x0010, r4
        mov #0x0020, r5
        mov #0x0030, r6
        clrc
        addc.b r4, r5
        cmp r5, r6
        jne _exit

    // Test 2: Simple Addc with carry
    TEST(2)
        mov #0x0010, r4
        mov #0x0020, r5
        mov #0x0031, r6
        setc
        addc.b r4, r5
        cmp r5, r6
        jne _exit

    // Test 3: Overflow Addc - Check result
    TEST(3)
        mov #0x00FF, r4
        mov #0x0001, r5
        mov #0x0000, r6
        clrc
        addc.b r4, r5
        cmp r5, r6
        jne _exit

    // Test 4: Overflow Addc w/carry- Check result 
    TEST(4)
        mov #0x0001, r4
        mov #0xaaFF, r5
        mov #0xaa01, r6
        setc
        addc.b r4, r5
        cmp r5, r6
        jne _exit


    // Test 5: Check Zero Flag from ADDC
    TEST(5)
        mov #0xFF, r4
        mov #0x01, r5
        clrc
        addc.b r4, r5
        jz TEST6
        jmp _exit

    // Test 6: Check Carry Flag from ADDC
    TEST(6)
        mov #0xFF, r4
        mov #0x01, r5
        setc
        addc.b r4, r5
        jc TEST7
        jmp _exit


    // Test 7: Check Carry Flag from ADDC
    TEST(7)
        mov #0xF0, r4
        mov #0x01, r5
        setc
        addc.b r4, r5
        jn TEST8
        jmp _exit

    // Test 8: Check V Flag 
    TEST(8)
        setc
        mov #0x007F, r4
        mov #0xAD01, r5
        addc.b r4, r5
        mov r2, r6
        and #0x100, r6
        jz _exit
        jnz TEST9
        jmp _exit

    TEST(9)
        setc
        mov #0x0070, r4
        mov #0xAD01, r5
        addc.b r4, r5
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
