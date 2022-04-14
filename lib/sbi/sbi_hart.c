/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_barrier.h>
#include <sbi/riscv_encoding.h>
#include <sbi/riscv_fp.h>
#include <sbi/sbi_bitops.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_csr_detect.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_math.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_trap.h>

extern void __sbi_expected_trap(void);
extern void __sbi_expected_trap_hext(void);

void (*sbi_hart_expected_trap)(void) = &__sbi_expected_trap;

struct hart_features {
	unsigned long features;
	unsigned int pmp_count;
	unsigned int pmp_addr_bits;
	unsigned long pmp_gran;
	unsigned int mhpm_count;
	unsigned int mhpm_bits;
};
static unsigned long hart_features_offset;

/*
1.是否支持浮点数扩展
2.是否支持向量扩展
3.是否支持SCOUNTEREN
4.是否支持MCOUNTEREN
5.是否支持MCOUNTINHIBIT
如果支持则开启
6.关闭中断
7.禁止satp
*/
static void mstatus_init(struct sbi_scratch *scratch)
{
	unsigned long mstatus_val = 0;

	/* Enable FPU */
	/*是否支持浮点数*/
	if (misa_extension('D') || misa_extension('F'))
		mstatus_val |=  MSTATUS_FS;

	/* Enable Vector context */
	/*是否支持向量扩展*/
	if (misa_extension('V'))
		mstatus_val |=  MSTATUS_VS;

	csr_write(CSR_MSTATUS, mstatus_val);

	/* Disable user mode usage of all perf counters except default ones (CY, TM, IR) */
	/*counter-enable寄存器scounteren是一个32位寄存器，它控制u模式下硬件性能监控计数器的可用性。
	当scounteren寄存器中的CY、TM、IR或HPMn位被清除时，在u模式下执行时试图读取 cycle、time、instret
	或hpmcountern寄存器将导致非法指令异常。当这些位中的一个被设置后，就允许访问相应的寄存器
	*/
	if (misa_extension('S') &&
	    sbi_hart_has_feature(scratch, SBI_HART_HAS_SCOUNTEREN))
		csr_write(CSR_SCOUNTEREN, 7);

	/**
	 * OpenSBI doesn't use any PMU counters in M-mode.
	 * Supervisor mode usage for all counters are enabled by default
	 * But counters will not run until mcountinhibit is set.
	 */
	/*
	当清除mcounteren寄存器中的CY、TM、IR或HPMn位时，当以s模式或u模式执行时，试图读取周期、时
	间、instret或hpmcountern寄存器将导致非法的指令异常。当设置了这些位中的一个，就允许在下一
	个实现的特权模式中访问相应的寄存器(如果实现了s模式，否则采用u模式)。
	*/
	if (sbi_hart_has_feature(scratch, SBI_HART_HAS_MCOUNTEREN))
		csr_write(CSR_MCOUNTEREN, -1);

	/* All programmable counters will start running at runtime after S-mode request */
	/*所有可编程计数器将在 S 模式请求后开始运行*/
	/*
	寄存器mcountinhibit是一个32位的WARL寄存器，它控制硬件性能监控计数器的增涨。该寄存器中
	的设置仅控制计数器是否递增;它的可访问性不受此寄存器设置的影响。
	当清除 mcountinhibit寄存器的CY、IR或HPMn比特位时，cycle、instret或 
	hpmcounterN寄存器会照样增长，当设置了CY、IR或HPMn时，对应的寄存器不增长。
	*/
	if (sbi_hart_has_feature(scratch, SBI_HART_HAS_MCOUNTINHIBIT))
		csr_write(CSR_MCOUNTINHIBIT, 0xFFFFFFF8);

	/* Disable all interrupts */
	csr_write(CSR_MIE, 0);

	/* Disable S-mode paging */
	if (misa_extension('S'))
		csr_write(CSR_SATP, 0);
}

static int fp_init(struct sbi_scratch *scratch)
{
#ifdef __riscv_flen
	int i;
#endif

	if (!misa_extension('D') && !misa_extension('F'))
		return 0;

	if (!(csr_read(CSR_MSTATUS) & MSTATUS_FS))
		return SBI_EINVAL;

#ifdef __riscv_flen
	for (i = 0; i < 32; i++)
		init_fp_reg(i);
	csr_write(CSR_FCSR, 0);
#endif

	return 0;
}
/*
将M模式的中断和异常发送到S模式下执行
*/
static int delegate_traps(struct sbi_scratch *scratch)
{
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);
	unsigned long interrupts, exceptions;

	if (!misa_extension('S'))
		/* No delegation possible as mideleg does not exist */
		return 0;

	/* Send M-mode interrupts and most exceptions to S-mode */
	/*
	mideleg寄存器和mip寄存器每个位相对应，
	SSIP supervisor模式下软件中断的挂起位，可以通过plateform-specific中断控制器设置为1
	STIP 可通过M-mode的软件传递timer中断到S-mode
	SEIP mip.SEIP是可写的，并且可以由M-mode软件编写来指示S-mode外部中断正在等待，此外，
		平台级中断控制器可以产生supervisor-level外部中断，supervisor-level外部中断可以
		被设置成pending是通过软件程序使用逻辑或SEIP比特位操作和外部中断控制器。当使用
		CSR指令读取mip寄存器时，SEIP比特位的值保存到rd目的寄存器中，虽然软件可以写SEIP
		比特位的值且这个SEIP的值受到中断控制的影响，但是中断控制器不能写SEIP的值，仅仅软
		可以read-modify-write其值，通过CSSRS或CSRRC指令
	*/
	interrupts = MIP_SSIP | MIP_STIP | MIP_SEIP;
	/*sscofpmf是对hpmcounter和mhpmevent csr进行标准化的扩展*/
	if (sbi_hart_has_feature(scratch, SBI_HART_HAS_SSCOFPMF))
		interrupts |= MIP_LCOFIP;
	/*
	CAUSE_MISALIGNED_FETCH  指令地址未对齐异常
	CAUSE_BREAKPOINT		断点异常
	CAUSE_USER_ECALL		U模式call异常
	CAUSE_FETCH_PAGE_FAULT	instruction page fault
	CAUSE_LOAD_PAGE_FAULT	load page fault
	CAUSE_STORE_PAGE_FAULT  store page fault
	*/
	exceptions = (1U << CAUSE_MISALIGNED_FETCH) | (1U << CAUSE_BREAKPOINT) |
		     (1U << CAUSE_USER_ECALL);
	if (sbi_platform_has_mfaults_delegation(plat))
		exceptions |= (1U << CAUSE_FETCH_PAGE_FAULT) |
			      (1U << CAUSE_LOAD_PAGE_FAULT) |
			      (1U << CAUSE_STORE_PAGE_FAULT);

	/*
	 * If hypervisor extension available then we only handle hypervisor
	 * calls (i.e. ecalls from HS-mode) in M-mode.
	 *
	 * The HS-mode will additionally handle supervisor calls (i.e. ecalls
	 * from VS-mode), Guest page faults and Virtual interrupts.
	 */
	if (misa_extension('H')) {
		exceptions |= (1U << CAUSE_VIRTUAL_SUPERVISOR_ECALL);
		exceptions |= (1U << CAUSE_FETCH_GUEST_PAGE_FAULT);
		exceptions |= (1U << CAUSE_LOAD_GUEST_PAGE_FAULT);
		exceptions |= (1U << CAUSE_VIRTUAL_INST_FAULT);
		exceptions |= (1U << CAUSE_STORE_GUEST_PAGE_FAULT);
	}

	csr_write(CSR_MIDELEG, interrupts);
	csr_write(CSR_MEDELEG, exceptions);

	return 0;
}

void sbi_hart_delegation_dump(struct sbi_scratch *scratch,
			      const char *prefix, const char *suffix)
{
	if (!misa_extension('S'))
		/* No delegation possible as mideleg does not exist*/
		return;

	sbi_printf("%sMIDELEG%s: 0x%" PRILX "\n",
		   prefix, suffix, csr_read(CSR_MIDELEG));
	sbi_printf("%sMEDELEG%s: 0x%" PRILX "\n",
		   prefix, suffix, csr_read(CSR_MEDELEG));
}

unsigned int sbi_hart_mhpm_count(struct sbi_scratch *scratch)
{
	struct hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->mhpm_count;
}

unsigned int sbi_hart_pmp_count(struct sbi_scratch *scratch)
{
	struct hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->pmp_count;
}

unsigned long sbi_hart_pmp_granularity(struct sbi_scratch *scratch)
{
	struct hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->pmp_gran;
}

unsigned int sbi_hart_pmp_addrbits(struct sbi_scratch *scratch)
{
	struct hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->pmp_addr_bits;
}

unsigned int sbi_hart_mhpm_bits(struct sbi_scratch *scratch)
{
	struct hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->mhpm_bits;
}

int sbi_hart_pmp_configure(struct sbi_scratch *scratch)
{
	struct sbi_domain_memregion *reg;
	struct sbi_domain *dom = sbi_domain_thishart_ptr();
	unsigned int pmp_idx = 0, pmp_flags, pmp_bits, pmp_gran_log2;
	unsigned int pmp_count = sbi_hart_pmp_count(scratch);
	unsigned long pmp_addr = 0, pmp_addr_max = 0;

	if (!pmp_count)
		return 0;

	pmp_gran_log2 = log2roundup(sbi_hart_pmp_granularity(scratch));
	pmp_bits = sbi_hart_pmp_addrbits(scratch) - 1;
	pmp_addr_max = (1UL << pmp_bits) | ((1UL << pmp_bits) - 1);

	sbi_domain_for_each_memregion(dom, reg) {
		if (pmp_count <= pmp_idx)
			break;

		pmp_flags = 0;
		if (reg->flags & SBI_DOMAIN_MEMREGION_READABLE)
			pmp_flags |= PMP_R;
		if (reg->flags & SBI_DOMAIN_MEMREGION_WRITEABLE)
			pmp_flags |= PMP_W;
		if (reg->flags & SBI_DOMAIN_MEMREGION_EXECUTABLE)
			pmp_flags |= PMP_X;
		if (reg->flags & SBI_DOMAIN_MEMREGION_MMODE)
			pmp_flags |= PMP_L;

		pmp_addr =  reg->base >> PMP_SHIFT;
		if (pmp_gran_log2 <= reg->order && pmp_addr < pmp_addr_max)
			pmp_set(pmp_idx++, pmp_flags, reg->base, reg->order);
		else {
			sbi_printf("Can not configure pmp for domain %s", dom->name);
			sbi_printf(" because memory region address %lx or size %lx is not in range\n",
				    reg->base, reg->order);
		}
	}

	return 0;
}

/**
 * Check whether a particular hart feature is available
 *
 * @param scratch pointer to the HART scratch space
 * @param feature the feature to check
 * @returns true (feature available) or false (feature not available)
 */
bool sbi_hart_has_feature(struct sbi_scratch *scratch, unsigned long feature)
{
	struct hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	if (hfeatures->features & feature)
		return true;
	else
		return false;
}

static unsigned long hart_get_features(struct sbi_scratch *scratch)
{
	struct hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->features;
}

static inline char *sbi_hart_feature_id2string(unsigned long feature)
{
	char *fstr = NULL;

	if (!feature)
		return NULL;

	switch (feature) {
	case SBI_HART_HAS_SCOUNTEREN:
		fstr = "scounteren";
		break;
	case SBI_HART_HAS_MCOUNTEREN:
		fstr = "mcounteren";
		break;
	case SBI_HART_HAS_MCOUNTINHIBIT:
		fstr = "mcountinhibit";
		break;
	case SBI_HART_HAS_SSCOFPMF:
		fstr = "sscofpmf";
		break;
	case SBI_HART_HAS_TIME:
		fstr = "time";
		break;
	default:
		break;
	}

	return fstr;
}

/**
 * Get the hart features in string format
 *
 * @param scratch pointer to the HART scratch space
 * @param features_str pointer to a char array where the features string will be
 *		       updated
 * @param nfstr length of the features_str. The feature string will be truncated
 *		if nfstr is not long enough.
 */
void sbi_hart_get_features_str(struct sbi_scratch *scratch,
			       char *features_str, int nfstr)
{
	unsigned long features, feat = 1UL;
	char *temp;
	int offset = 0;

	if (!features_str || nfstr <= 0)
		return;
	sbi_memset(features_str, 0, nfstr);

	features = hart_get_features(scratch);
	if (!features)
		goto done;

	do {
		if (features & feat) {
			temp = sbi_hart_feature_id2string(feat);
			if (temp) {
				sbi_snprintf(features_str + offset, nfstr,
					     "%s,", temp);
				offset = offset + sbi_strlen(temp) + 1;
			}
		}
		feat = feat << 1;
	} while (feat <= SBI_HART_HAS_LAST_FEATURE);

done:
	if (offset)
		features_str[offset - 1] = '\0';
	else
		sbi_strncpy(features_str, "none", nfstr);
}
/*
原理：软件可以通过写入0到pmp0cfg来确定PMP的粒度，然后将所有的1都写入pmpaddr0，
然后再读回pmpaddr0。如果G是最低有效位集的下标，则PMP粒度为2^(G+2)字节。
作用：获取CSR_PMPADDR0允许的地址范围
*/
static unsigned long hart_pmp_get_allowed_addr(void)
{
	unsigned long val = 0;
	struct sbi_trap_info trap = {0};

	/*这里尝试将0写入CSR_PMPCFG0,然后看是否会出现异常，trap.cause为0，表示没有出现异常*/
	csr_write_allowed(CSR_PMPCFG0, (ulong)&trap, 0);
	if (trap.cause)
		return 0;
	/*向CSR_PMPADDR0中写入PMP_ADDR_MASK，trap.cause为0，表示没有出现异常*/
	csr_write_allowed(CSR_PMPADDR0, (ulong)&trap, PMP_ADDR_MASK);
	if (!trap.cause) {
		/*根据riscv-privileged.pdf第3.7章描述，对CSR_PMPADDR0写入全1后，再读取，可以得到支持的CSR_PMPADDR0*/
		val = csr_read_allowed(CSR_PMPADDR0, (ulong)&trap);
		if (trap.cause)
			val = 0;
	}

	return val;
}

static int hart_pmu_get_allowed_bits(void)
{
	unsigned long val = ~(0UL);
	struct sbi_trap_info trap = {0};
	int num_bits = 0;

	/**
	 * It is assumed that platforms will implement same number of bits for
	 * all the performance counters including mcycle/minstret.
	 */
	csr_write_allowed(CSR_MHPMCOUNTER3, (ulong)&trap, val);
	if (!trap.cause) {
		val = csr_read_allowed(CSR_MHPMCOUNTER3, (ulong)&trap);
		if (trap.cause)
			return 0;
	}
	num_bits = __fls(val) + 1;
#if __riscv_xlen == 32
	csr_write_allowed(CSR_MHPMCOUNTER3H, (ulong)&trap, val);
	if (!trap.cause) {
		val = csr_read_allowed(CSR_MHPMCOUNTER3H, (ulong)&trap);
		if (trap.cause)
			return num_bits;
	}
	num_bits += __fls(val) + 1;

#endif

	return num_bits;
}

/*
检测hart 支持的feature
1. PMPADDR粒度，PMPADDR支持多少个entry
2. mhpm counters
3. scounteren
4. mcounteren
5. mcountinhibit
6. sscofpmf 
7. time CSR 
*/
static void hart_detect_features(struct sbi_scratch *scratch)
{
	struct sbi_trap_info trap = {0};
	struct hart_features *hfeatures;
	unsigned long val;

	/* Reset hart features */
	hfeatures = sbi_scratch_offset_ptr(scratch, hart_features_offset);
	hfeatures->features = 0;
	hfeatures->pmp_count = 0;
	hfeatures->mhpm_count = 0;

#define __check_csr(__csr, __rdonly, __wrval, __field, __skip)	\
	val = csr_read_allowed(__csr, (ulong)&trap);			\
	if (!trap.cause) {						\
		if (__rdonly) {						\
			(hfeatures->__field)++;				\
		} else {						\
			csr_write_allowed(__csr, (ulong)&trap, __wrval);\
			if (!trap.cause) {				\
				if (csr_swap(__csr, val) == __wrval)	\
					(hfeatures->__field)++;		\
				else					\
					goto __skip;			\
			} else {					\
				goto __skip;				\
			}						\
		}							\
	} else {							\
		goto __skip;						\
	}
#define __check_csr_2(__csr, __rdonly, __wrval, __field, __skip)	\
	__check_csr(__csr + 0, __rdonly, __wrval, __field, __skip)	\
	__check_csr(__csr + 1, __rdonly, __wrval, __field, __skip)
#define __check_csr_4(__csr, __rdonly, __wrval, __field, __skip)	\
	__check_csr_2(__csr + 0, __rdonly, __wrval, __field, __skip)	\
	__check_csr_2(__csr + 2, __rdonly, __wrval, __field, __skip)
#define __check_csr_8(__csr, __rdonly, __wrval, __field, __skip)	\
	__check_csr_4(__csr + 0, __rdonly, __wrval, __field, __skip)	\
	__check_csr_4(__csr + 4, __rdonly, __wrval, __field, __skip)
#define __check_csr_16(__csr, __rdonly, __wrval, __field, __skip)	\
	__check_csr_8(__csr + 0, __rdonly, __wrval, __field, __skip)	\
	__check_csr_8(__csr + 8, __rdonly, __wrval, __field, __skip)
#define __check_csr_32(__csr, __rdonly, __wrval, __field, __skip)	\
	__check_csr_16(__csr + 0, __rdonly, __wrval, __field, __skip)	\
	__check_csr_16(__csr + 16, __rdonly, __wrval, __field, __skip)
#define __check_csr_64(__csr, __rdonly, __wrval, __field, __skip)	\
	__check_csr_32(__csr + 0, __rdonly, __wrval, __field, __skip)	\
	__check_csr_32(__csr + 32, __rdonly, __wrval, __field, __skip)

	/**
	 * Detect the allowed address bits & granularity. At least PMPADDR0
	 * should be implemented.
	 */
	val = hart_pmp_get_allowed_addr();
	if (val) {
		/*__ffs()找到从左边到右边第一个1的位置，__fls()找到从左边到右边最后一个1的位置，
		如：0x1ff，__ffs()返回0， __fls()返回8*/
		hfeatures->pmp_gran =  1 << (__ffs(val) + 2); //PMPADDR粒度
		hfeatures->pmp_addr_bits = __fls(val) + 1;
		/* Detect number of PMP regions. At least PMPADDR0 should be implemented
		检测最大支持到CSR_PMPADDRi，这里就让CSR_PMPADDR的地址依次增加，然后一个一个探测，
		并把最大支持到的个数放入hfeatures->pmp_count中。
		*/
		__check_csr_64(CSR_PMPADDR0, 0, val, pmp_count, __pmp_skip);
	}
__pmp_skip:

	/* Detect number of MHPM counters */
	__check_csr(CSR_MHPMCOUNTER3, 0, 1UL, mhpm_count, __mhpm_skip);
	hfeatures->mhpm_bits = hart_pmu_get_allowed_bits();

	__check_csr_4(CSR_MHPMCOUNTER4, 0, 1UL, mhpm_count, __mhpm_skip);
	__check_csr_8(CSR_MHPMCOUNTER8, 0, 1UL, mhpm_count, __mhpm_skip);
	__check_csr_16(CSR_MHPMCOUNTER16, 0, 1UL, mhpm_count, __mhpm_skip);

	/**
	 * No need to check for MHPMCOUNTERH for RV32 as they are expected to be
	 * implemented if MHPMCOUNTER is implemented.
	 */

__mhpm_skip:

#undef __check_csr_64
#undef __check_csr_32
#undef __check_csr_16
#undef __check_csr_8
#undef __check_csr_4
#undef __check_csr_2
#undef __check_csr

	/* Detect if hart supports SCOUNTEREN feature */
	val = csr_read_allowed(CSR_SCOUNTEREN, (unsigned long)&trap);
	if (!trap.cause) {
		csr_write_allowed(CSR_SCOUNTEREN, (unsigned long)&trap, val);
		if (!trap.cause)
			hfeatures->features |= SBI_HART_HAS_SCOUNTEREN;
	}

	/* Detect if hart supports MCOUNTEREN feature */
	val = csr_read_allowed(CSR_MCOUNTEREN, (unsigned long)&trap);
	if (!trap.cause) {
		csr_write_allowed(CSR_MCOUNTEREN, (unsigned long)&trap, val);
		if (!trap.cause)
			hfeatures->features |= SBI_HART_HAS_MCOUNTEREN;
	}

	/* Detect if hart supports MCOUNTINHIBIT feature */
	val = csr_read_allowed(CSR_MCOUNTINHIBIT, (unsigned long)&trap);
	if (!trap.cause) {
		csr_write_allowed(CSR_MCOUNTINHIBIT, (unsigned long)&trap, val);
		if (!trap.cause)
			hfeatures->features |= SBI_HART_HAS_MCOUNTINHIBIT;
	}

	/* Counter overflow/filtering is not useful without mcounter/inhibit */
	if (hfeatures->features & SBI_HART_HAS_MCOUNTINHIBIT &&
	    hfeatures->features & SBI_HART_HAS_MCOUNTEREN) {
		/* Detect if hart supports sscofpmf */
		csr_read_allowed(CSR_SCOUNTOVF, (unsigned long)&trap);
		if (!trap.cause)
			hfeatures->features |= SBI_HART_HAS_SSCOFPMF;
	}

	/* Detect if hart supports time CSR */
	csr_read_allowed(CSR_TIME, (unsigned long)&trap);
	if (!trap.cause)
		hfeatures->features |= SBI_HART_HAS_TIME;
}

/*
1.是否支持浮点数扩展
2.是否支持向量扩展
3.是否支持SCOUNTEREN
4.是否支持MCOUNTEREN
5.是否支持MCOUNTINHIBIT
如果支持则开启
6.关闭中断
7.禁止satp
8.将M模式的中断和异常发送到S模式下执行
*/
int sbi_hart_reinit(struct sbi_scratch *scratch)
{
	int rc;

	mstatus_init(scratch);

	rc = fp_init(scratch);
	if (rc)
		return rc;

	rc = delegate_traps(scratch);
	if (rc)
		return rc;

	return 0;
}
/*
1.支持hypervisor扩展模式的话，设置trap_hext
2.分配在扩展区域分配struct hart_features结构体
3.记录feature到struct hart_features结构体中
4.1.是否支持浮点数扩展
4.2.是否支持向量扩展
4.3.是否支持SCOUNTEREN
4.4.是否支持MCOUNTEREN
4.5.是否支持MCOUNTINHIBIT
    如果支持则开启
4.6.关闭中断
4.7.禁止satp
4.8.将M模式的中断和异常发送到S模式下执行
*/
int sbi_hart_init(struct sbi_scratch *scratch, bool cold_boot)
{
	if (cold_boot) {
		/*判断是否支持Hypervisor extension*/
		if (misa_extension('H'))
			sbi_hart_expected_trap = &__sbi_expected_trap_hext;

		//在扩展区域分配struct hart_features结构体
		hart_features_offset = sbi_scratch_alloc_offset(
						sizeof(struct hart_features));
		if (!hart_features_offset)
			return SBI_ENOMEM;
	}
	//检测hart支持的feature，并记录支持的feature到struct hart_features中
	hart_detect_features(scratch);

	return sbi_hart_reinit(scratch);
}

void __attribute__((noreturn)) sbi_hart_hang(void)
{
	while (1)
		wfi();
	__builtin_unreachable();
}

void __attribute__((noreturn))
sbi_hart_switch_mode(unsigned long arg0, unsigned long arg1,
		     unsigned long next_addr, unsigned long next_mode,
		     bool next_virt)
{
#if __riscv_xlen == 32
	unsigned long val, valH;
#else
	unsigned long val;
#endif

	switch (next_mode) {
	case PRV_M:
		break;
	case PRV_S:
		if (!misa_extension('S'))
			sbi_hart_hang();
		break;
	case PRV_U:
		if (!misa_extension('U'))
			sbi_hart_hang();
		break;
	default:
		sbi_hart_hang();
	}

	val = csr_read(CSR_MSTATUS);
	val = INSERT_FIELD(val, MSTATUS_MPP, next_mode);
	val = INSERT_FIELD(val, MSTATUS_MPIE, 0);
#if __riscv_xlen == 32
	if (misa_extension('H')) {
		valH = csr_read(CSR_MSTATUSH);
		if (next_virt)
			valH = INSERT_FIELD(valH, MSTATUSH_MPV, 1);
		else
			valH = INSERT_FIELD(valH, MSTATUSH_MPV, 0);
		csr_write(CSR_MSTATUSH, valH);
	}
#else
	if (misa_extension('H')) {
		if (next_virt)
			val = INSERT_FIELD(val, MSTATUS_MPV, 1);
		else
			val = INSERT_FIELD(val, MSTATUS_MPV, 0);
	}
#endif
	csr_write(CSR_MSTATUS, val);
	csr_write(CSR_MEPC, next_addr);

	if (next_mode == PRV_S) {
		csr_write(CSR_STVEC, next_addr);
		csr_write(CSR_SSCRATCH, 0);
		csr_write(CSR_SIE, 0);
		csr_write(CSR_SATP, 0);
	} else if (next_mode == PRV_U) {
		if (misa_extension('N')) {
			csr_write(CSR_UTVEC, next_addr);
			csr_write(CSR_USCRATCH, 0);
			csr_write(CSR_UIE, 0);
		}
	}

	register unsigned long a0 asm("a0") = arg0;
	register unsigned long a1 asm("a1") = arg1;
	__asm__ __volatile__("mret" : : "r"(a0), "r"(a1));
	__builtin_unreachable();
}
