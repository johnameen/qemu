-include ../../config-host.mak

SIM = ../../../msp430-softmmu/qemu-system-msp430
SIMFLAGS = -M msp430f1x -nographic -d op,cpu -D $$case.testlog -kernel

CROSS = msp430-
CC = $(CROSS)gcc
OBJCOPY = $(CROSS)objcopy
OBJDUMP = $(CROSS)objdump

TESTCASES+=test_addc.bin \
			test_jxx.bin \
			test_reti.bin \
			test_addcb.bin \
			test_add.bin \
			test_addb.bin \
			test_and.bin \
			test_andb.bin \
			test_bic.bin \
			test_bis.bin \
			test_bit.bin \
			test_dadd.bin \
			test_daddb.bin \
			test_push_pop.bin \
			test_xor.bin \
			test_sxt.bin \
			test_rra.bin \
			test_rrab.bin \
			test_rrc.bin \
			test_rrcb.bin \
			test_swpb.bin \
			test_subs.bin \
			test_mov.bin

all: $(TESTCASES)

%.tst: %.S
	$(CC) $< -o $@ -mmcu=msp430f1101a -nostartfiles -Wl,--defsym=STORAGE=0x200

%.tst: %.c
	$(CC) $< -o $@ -mmcu=msp430f1101a

%.bin: %.tst
	$(OBJCOPY) -O binary $< $@
	$(OBJDUMP) -DS $< > $@.dmp

clean:
	$(RM) -rf $(TESTCASES)
	$(RM) -rf *.testlog
	$(RM) -rf *.dmp


check: $(TESTCASES)
	@for case in $(TESTCASES); do \
		echo Testing $$case ; \
		$(SIM) $(SIMFLAGS) $$case || exit 1; \
		echo $$case pass!; \
		sleep 0.2; \
	done
