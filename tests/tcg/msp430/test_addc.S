.section .text,"ax",@progbits

_start:
    // Test 1: Simple Addc
    TEST1:
        mov #1, r15
        mov #0x100, r4
        mov #0x200, r5
        mov #0x300, r6
        clrc
        addc r4, r5
        cmp r5, r6
        jne _exit

    // Test 2: Simple Addc with carry
    TEST2:
        mov #2, r15
        mov #0x100, r4
        mov #0x200, r5
        mov #0x301, r6
        setc
        addc r4, r5
        cmp r5, r6
        jne _exit

    // Test 3: Overflow Addc - Check result
    TEST3:
        mov #3, r15
        mov #0xFF00, r4
        mov #0x0100, r5
        mov #0x0000, r6
        clrc
        addc r4, r5
        cmp r5, r6
        jne _exit

    // Test 4: Overflow Addc w/carry- Check result 
    TEST4:
        mov #4, r15
        mov #0xFF00, r4
        mov #0x0100, r5
        mov #0x0001, r6
        setc
        addc r4, r5
        cmp r5, r6
        jne _exit


    // Test 5: Check Zero Flag from ADDC
    TEST5:
        mov #5, r15
        mov #0xFF00, r4
        mov #0x0100, r5
        clrc
        addc r4, r5
        jz TEST6
        jmp _exit

    // Test 6: Check Carry Flag from ADDC
    TEST6:
        mov #6, r15
        mov #0xFF00, r4
        mov #0x0100, r5
        setc
        addc r4, r5
        jc TEST7
        jmp _exit


    // Test 7: Check Carry Flag from ADDC
    TEST7:
        mov #7, r15
        mov #0xF000, r4
        mov #0x0100, r5
        setc
        addc r4, r5
        jn TEST8
        jmp _exit

    // Test 8: Check V Flag 
    TEST8:
        setc
        mov #0x7FFE, r4
        mov #0x0001, r5
        addc r4, r5
        mov r2, r6
        and #0x100, r6
        jz _exit
        jnz TEST9
        jmp _exit

    TEST9:
        setc
        mov #0x7F00, r4
        mov #0x0001, r5
        addc r4, r5
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
