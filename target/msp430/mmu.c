#include "cpu.h"

int msp430_cpu_handle_mmu_fault(CPUState *cs, vaddr address,
                               int rw, int mmu_idx);

int msp430_cpu_handle_mmu_fault(CPUState *cs, vaddr address,
                               int rw, int mmu_idx)
{
    int prot;
    target_ulong phy;

    address &= TARGET_PAGE_MASK;
    prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    phy = address;
    tlb_set_page(cs, address, phy, prot, mmu_idx, TARGET_PAGE_SIZE);
    return 0;
}

/* Try to fill the TLB and return an exception if error. If retaddr is
   NULL, it means that the function was called in C code (i.e. not
   from generated code or from helper.c) */
void tlb_fill(CPUState *cs, 
              target_ulong addr, 
              MMUAccessType access_type, 
              int mmu_idx,
              uintptr_t retaddr)
{
    int ret;

    ret = msp430_cpu_handle_mmu_fault(cs, addr, access_type, mmu_idx);
    if (unlikely(ret)) {
        if (retaddr) {
            cpu_restore_state(cs, retaddr);
        }
    }
}
