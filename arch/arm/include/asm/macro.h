/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/asm-arm/macro.h
 *
 * Copyright (C) 2009 Jean-Christophe PLAGNIOL-VILLARD <plagnioj@jcrosoft.com>
 */

#ifndef __ASM_ARM_MACRO_H__
#define __ASM_ARM_MACRO_H__

#ifdef CONFIG_ARM64
#include <asm/system.h>
#endif

#ifdef __ASSEMBLY__

/*
 * These macros provide a convenient way to write 8, 16 and 32 bit data
 * to any address.
 * Registers r4 and r5 are used, any data in these registers are
 * overwritten by the macros.
 * The macros are valid for any ARM architecture, they do not implement
 * any memory barriers so caution is recommended when using these when the
 * caches are enabled or on a multi-core system.
 */

.macro	write32, addr, data
	ldr	r4, =\addr
	ldr	r5, =\data
	str	r5, [r4]
.endm

.macro	write16, addr, data
	ldr	r4, =\addr
	ldrh	r5, =\data
	strh	r5, [r4]
.endm

.macro	write8, addr, data
	ldr	r4, =\addr
	ldrb	r5, =\data
	strb	r5, [r4]
.endm

/*
 * This macro generates a loop that can be used for delays in the code.
 * Register r4 is used, any data in this register is overwritten by the
 * macro.
 * The macro is valid for any ARM architeture. The actual time spent in the
 * loop will vary from CPU to CPU though.
 */

.macro	wait_timer, time
	ldr	r4, =\time
1:
	nop
	subs	r4, r4, #1
	bcs	1b
.endm

#ifdef CONFIG_ARM64
/*
 * Register aliases.
 */
lr	.req	x30

/*
 * Branch according to exception level
 */
.macro	switch_el, xreg, el3_label, el2_label, el1_label
	mrs	\xreg, CurrentEL
	cmp	\xreg, #0x8
	b.gt	\el3_label
	b.eq	\el2_label
	b.lt	\el1_label
.endm

/*
 * Branch if we are not in the highest exception level
 */
.macro	branch_if_not_highest_el, xreg, label
	switch_el \xreg, 3f, 2f, 1f

2:	mrs	\xreg, ID_AA64PFR0_EL1
	and	\xreg, \xreg, #(ID_AA64PFR0_EL1_EL3)
	cbnz	\xreg, \label
	b	3f

1:	mrs	\xreg, ID_AA64PFR0_EL1
	and	\xreg, \xreg, #(ID_AA64PFR0_EL1_EL3 | ID_AA64PFR0_EL1_EL2)
	cbnz	\xreg, \label

3:
.endm

/*
 * Branch if current processor is a Cortex-A57 core.
 */
.macro	branch_if_a57_core, xreg, a57_label
	mrs	\xreg, midr_el1
	lsr	\xreg, \xreg, #4
	and	\xreg, \xreg, #0x00000FFF
	cmp	\xreg, #0xD07		/* Cortex-A57 MPCore processor. */
	b.eq	\a57_label
.endm

/*
 * Branch if current processor is a Cortex-A53 core.
 */
.macro	branch_if_a53_core, xreg, a53_label
	mrs	\xreg, midr_el1
	lsr	\xreg, \xreg, #4
	and	\xreg, \xreg, #0x00000FFF
	cmp	\xreg, #0xD03		/* Cortex-A53 MPCore processor. */
	b.eq	\a53_label
.endm

/*
 * Branch if current processor is a slave,
 * choose processor with all zero affinity value as the master.
 */
.macro	branch_if_slave, xreg, slave_label
#ifdef CONFIG_ARMV8_MULTIENTRY
	mrs	\xreg, mpidr_el1
	and	\xreg, \xreg,  0xffffffffff	/* clear bits [63:40] */
	and	\xreg, \xreg, ~0x00ff000000	/* also clear bits [31:24] */
	cbnz	\xreg, \slave_label
#endif
.endm

/*
 * Branch if current processor is a master,
 * choose processor with all zero affinity value as the master.
 */
.macro	branch_if_master, xreg, master_label
#ifdef CONFIG_ARMV8_MULTIENTRY
	mrs	\xreg, mpidr_el1
	and	\xreg, \xreg,  0xffffffffff	/* clear bits [63:40] */
	and	\xreg, \xreg, ~0x00ff000000	/* also clear bits [31:24] */
	cbz	\xreg, \master_label
#else
	b	\master_label
#endif
.endm

/*
 * Switch from EL3 to EL2 for ARMv8
 * @ep:     kernel entry point
 * @flag:   The execution state flag for lower exception
 *          level, ES_TO_AARCH64 or ES_TO_AARCH32
 * @tmp:    temporary register
 *
 * For loading 32-bit OS, x1 is machine nr and x2 is ftaddr.
 * For loading 64-bit OS, x0 is physical address to the FDT blob.
 * They will be passed to the guest.
 */
.macro armv8_switch_to_el2_m, ep, flag, tmp
	msr	cptr_el3, xzr		/* Disable coprocessor traps to EL3 */
	mov	\tmp, #CPTR_EL2_RES1
	msr	cptr_el2, \tmp		/* Disable coprocessor traps to EL2 */

	/* Initialize Generic Timers */
	msr	cntvoff_el2, xzr

	/* Initialize SCTLR_EL2
	 *
	 * setting RES1 bits (29,28,23,22,18,16,11,5,4) to 1
	 * and RES0 bits (31,30,27,26,24,21,20,17,15-13,10-6) +
	 * EE,WXN,I,SA,C,A,M to 0
	 */
	ldr	\tmp, =(SCTLR_EL2_RES1 | SCTLR_EL2_EE_LE |\
			SCTLR_EL2_WXN_DIS | SCTLR_EL2_ICACHE_DIS |\
			SCTLR_EL2_SA_DIS | SCTLR_EL2_DCACHE_DIS |\
			SCTLR_EL2_ALIGN_DIS | SCTLR_EL2_MMU_DIS)
	msr	sctlr_el2, \tmp

	mov	\tmp, sp
	msr	sp_el2, \tmp		/* Migrate SP */
	mrs	\tmp, vbar_el3
	msr	vbar_el2, \tmp		/* Migrate VBAR */

	/* Check switch to AArch64 EL2 or AArch32 Hypervisor mode */
	cmp	\flag, #ES_TO_AARCH32
	b.eq	1f

	/*
	 * The next lower exception level is AArch64, 64bit EL2 | HCE |
	 * RES1 (Bits[5:4]) | Non-secure EL0/EL1.
	 * and the SMD depends on requirements.
	 */
#ifdef CONFIG_ARMV8_PSCI
	ldr	\tmp, =(SCR_EL3_RW_AARCH64 | SCR_EL3_HCE_EN |\
			SCR_EL3_RES1 | SCR_EL3_NS_EN)
#else
	ldr	\tmp, =(SCR_EL3_RW_AARCH64 | SCR_EL3_HCE_EN |\
			SCR_EL3_SMD_DIS | SCR_EL3_RES1 |\
			SCR_EL3_NS_EN)
#endif

#ifdef CONFIG_ARMV8_EA_EL3_FIRST
	orr	\tmp, \tmp, #SCR_EL3_EA_EN
#endif
	msr	scr_el3, \tmp

	/* Return to the EL2_SP2 mode from EL3 */
	ldr	\tmp, =(SPSR_EL_DEBUG_MASK | SPSR_EL_SERR_MASK |\
			SPSR_EL_IRQ_MASK | SPSR_EL_FIQ_MASK |\
			SPSR_EL_M_AARCH64 | SPSR_EL_M_EL2H)
	msr	spsr_el3, \tmp
	msr	elr_el3, \ep
	eret

1:
	/*
	 * The next lower exception level is AArch32, 32bit EL2 | HCE |
	 * SMD | RES1 (Bits[5:4]) | Non-secure EL0/EL1.
	 */
	ldr	\tmp, =(SCR_EL3_RW_AARCH32 | SCR_EL3_HCE_EN |\
			SCR_EL3_SMD_DIS | SCR_EL3_RES1 |\
			SCR_EL3_NS_EN)
	msr	scr_el3, \tmp

	/* Return to AArch32 Hypervisor mode */
	ldr     \tmp, =(SPSR_EL_END_LE | SPSR_EL_ASYN_MASK |\
			SPSR_EL_IRQ_MASK | SPSR_EL_FIQ_MASK |\
			SPSR_EL_T_A32 | SPSR_EL_M_AARCH32 |\
			SPSR_EL_M_HYP)
	msr	spsr_el3, \tmp
	msr     elr_el3, \ep
	eret
.endm

/*
 * Switch from EL2 to EL1 for ARMv8
 * @ep:     kernel entry point
 * @flag:   The execution state flag for lower exception
 *          level, ES_TO_AARCH64 or ES_TO_AARCH32
 * @tmp:    temporary register
 *
 * For loading 32-bit OS, x1 is machine nr and x2 is ftaddr.
 * For loading 64-bit OS, x0 is physical address to the FDT blob.
 * They will be passed to the guest.
 */
.macro armv8_switch_to_el1_m, ep, flag, tmp, tmp2
	/* Initialize Generic Timers */
	mrs	\tmp, cnthctl_el2
	/* Enable EL1 access to timers */
	orr	\tmp, \tmp, #(CNTHCTL_EL2_EL1PCEN_EN |\
		CNTHCTL_EL2_EL1PCTEN_EN)
	msr	cnthctl_el2, \tmp
	msr	cntvoff_el2, xzr

	/* Initilize MPID/MPIDR registers */
	mrs	\tmp, midr_el1
	msr	vpidr_el2, \tmp
	mrs	\tmp, mpidr_el1
	msr	vmpidr_el2, \tmp

	/* Disable coprocessor traps */
	mov	\tmp, #CPTR_EL2_RES1
	msr	cptr_el2, \tmp		/* Disable coprocessor traps to EL2 */
	msr	hstr_el2, xzr		/* Disable coprocessor traps to EL2 */
	mov	\tmp, #CPACR_EL1_FPEN_EN
	msr	cpacr_el1, \tmp		/* Enable FP/SIMD at EL1 */

	/* SCTLR_EL1 initialization
	 *
	 * setting RES1 bits (29,28,23,22,20,11) to 1
	 * and RES0 bits (31,30,27,21,17,13,10,6) +
	 * UCI,EE,EOE,WXN,nTWE,nTWI,UCT,DZE,I,UMA,SED,ITD,
	 * CP15BEN,SA0,SA,C,A,M to 0
	 */
	ldr	\tmp, =(SCTLR_EL1_RES1 | SCTLR_EL1_UCI_DIS |\
			SCTLR_EL1_EE_LE | SCTLR_EL1_WXN_DIS |\
			SCTLR_EL1_NTWE_DIS | SCTLR_EL1_NTWI_DIS |\
			SCTLR_EL1_UCT_DIS | SCTLR_EL1_DZE_DIS |\
			SCTLR_EL1_ICACHE_DIS | SCTLR_EL1_UMA_DIS |\
			SCTLR_EL1_SED_EN | SCTLR_EL1_ITD_EN |\
			SCTLR_EL1_CP15BEN_DIS | SCTLR_EL1_SA0_DIS |\
			SCTLR_EL1_SA_DIS | SCTLR_EL1_DCACHE_DIS |\
			SCTLR_EL1_ALIGN_DIS | SCTLR_EL1_MMU_DIS)
	msr	sctlr_el1, \tmp

	mov	\tmp, sp
	msr	sp_el1, \tmp		/* Migrate SP */
	mrs	\tmp, vbar_el2
	msr	vbar_el1, \tmp		/* Migrate VBAR */

	/* Check switch to AArch64 EL1 or AArch32 Supervisor mode */
	cmp	\flag, #ES_TO_AARCH32
	b.eq	1f

	/* Initialize HCR_EL2 */
	/* Only disable PAuth traps if PAuth is supported */
	mrs	\tmp, id_aa64isar1_el1
	ldr	\tmp2, =(ID_AA64ISAR1_EL1_GPI | ID_AA64ISAR1_EL1_GPA | \
		      ID_AA64ISAR1_EL1_API | ID_AA64ISAR1_EL1_APA)
	tst	\tmp, \tmp2
	mov	\tmp2, #(HCR_EL2_RW_AARCH64 | HCR_EL2_HCD_DIS)
	orr	\tmp, \tmp2, #(HCR_EL2_APK | HCR_EL2_API)
	csel	\tmp, \tmp2, \tmp, eq
	msr	hcr_el2, \tmp

	/* Return to the EL1_SP1 mode from EL2 */
	ldr	\tmp, =(SPSR_EL_DEBUG_MASK | SPSR_EL_SERR_MASK |\
			SPSR_EL_IRQ_MASK | SPSR_EL_FIQ_MASK |\
			SPSR_EL_M_AARCH64 | SPSR_EL_M_EL1H)
	msr	spsr_el2, \tmp
	msr     elr_el2, \ep
	eret

1:
	/* Initialize HCR_EL2 */
	ldr	\tmp, =(HCR_EL2_RW_AARCH32 | HCR_EL2_HCD_DIS)
	msr	hcr_el2, \tmp

	/* Return to AArch32 Supervisor mode from EL2 */
	ldr	\tmp, =(SPSR_EL_END_LE | SPSR_EL_ASYN_MASK |\
			SPSR_EL_IRQ_MASK | SPSR_EL_FIQ_MASK |\
			SPSR_EL_T_A32 | SPSR_EL_M_AARCH32 |\
			SPSR_EL_M_SVC)
	msr     spsr_el2, \tmp
	msr     elr_el2, \ep
	eret
.endm

/*
 * Switch from EL2 to EL1 for ARMv8
 * @ep:     kernel entry point
 * @tmp:    temporary register
 *
 * x0 is argc and x1 is argv
 */
.macro cs452_switch_to_el1_m, ep, tmp
	/* Initialize Generic Timers */
    ldr \tmp, =CS452_HCR_RW
    msr hcr_el2, \tmp

    ldr \tmp, =CS452_SPSR_VALUE
    msr spsr_el2, \tmp

    msr elr_el2, \ep

    eret // -> el1_entry
.endm

#if defined(CONFIG_GICV3)
.macro gic_wait_for_interrupt_m xreg1
0 :	wfi
	mrs     \xreg1, ICC_IAR1_EL1
	msr     ICC_EOIR1_EL1, \xreg1
	cbnz    \xreg1, 0b
.endm
#elif defined(CONFIG_GICV2)
.macro gic_wait_for_interrupt_m xreg1, wreg2
0 :	wfi
	ldr     \wreg2, [\xreg1, GICC_AIAR]
	str     \wreg2, [\xreg1, GICC_AEOIR]
	and	\wreg2, \wreg2, #0x3ff
	cbnz    \wreg2, 0b
.endm
#endif

#endif /* CONFIG_ARM64 */

#endif /* __ASSEMBLY__ */
#endif /* __ASM_ARM_MACRO_H__ */
