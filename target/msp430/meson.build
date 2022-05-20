msp430_ss = ss.source_set()
msp430_ss.add(files(
  'translate.c',
  'cpu.c',
  'helper.c'
))

msp430_softmmu_ss = ss.source_set()
msp430_softmmu_ss.add(files(
  'mmu.c'
))

target_arch += {'msp430': msp430_ss}
target_softmmu_arch += {'msp430': msp430_softmmu_ss}
