 /*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#include <sbi/riscv_locks.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_hartmask.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>

u32 last_hartid_having_scratch = SBI_HARTMASK_MAX_BITS - 1;
struct sbi_scratch *hartid_to_scratch_table[SBI_HARTMASK_MAX_BITS] = { 0 };

static spinlock_t extra_lock = SPIN_LOCK_INITIALIZER;
static unsigned long extra_offset = SBI_SCRATCH_EXTRA_SPACE_OFFSET;

typedef struct sbi_scratch *(*hartid2scratch)(ulong hartid, ulong hartindex);

/*
初始化hartid_to_scratch_table数组，将hartid对应的sbi_scratch结构体地址填入此数组
*/
int sbi_scratch_init(struct sbi_scratch *scratch)
{
	u32 i;
	/*从sbi_scrach结构体中得到sbi_platfrom*/
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);

	for (i = 0; i < SBI_HARTMASK_MAX_BITS; i++) {
		if (sbi_platform_hart_invalid(plat, i))
			continue;
		/*hartid_to_scratch函数是在fw_base.S文件中的_hartid_to_scratch汇编函数，
		_hartid_to_scratch作用是通过hartid找到对应的sbi_scratch结构体，
		将hartid对应的sib_scratch结构体地址填入到hartid_to_scratch_table数组中*/
		hartid_to_scratch_table[i] =
			((hartid2scratch)scratch->hartid_to_scratch)(i,
					sbi_platform_hart_index(plat, i));
		if (hartid_to_scratch_table[i])
			last_hartid_having_scratch = i;
	}

	return 0;
}

/*对所有的hart的extra space进行操作,在extra space分配size字节的内存*/
unsigned long sbi_scratch_alloc_offset(unsigned long size)
{
	u32 i;
	void *ptr;
	unsigned long ret = 0;
	struct sbi_scratch *rscratch;

	/*
	 * We have a simple brain-dead allocator which never expects
	 * anything to be free-ed hence it keeps incrementing the
	 * next allocation offset until it runs-out of space.
	 *
	 * In future, we will have more sophisticated allocator which
	 * will allow us to re-claim free-ed space.
	 */

	if (!size)
		return 0;

	if (size & (__SIZEOF_POINTER__ - 1))
		size = (size & ~(__SIZEOF_POINTER__ - 1)) + __SIZEOF_POINTER__;

	spin_lock(&extra_lock);
	/*extra_offset 为sbi_stratch结构体额外空间偏移
		extra_offset = 11 * 8 = 88, 
		sbi_scratch额外的空间，紧跟在sb_scratch结构体后面*/
	if (SBI_SCRATCH_SIZE < (extra_offset + size))
		goto done;

	ret = extra_offset;
	extra_offset += size;//增加偏移量

done:
	spin_unlock(&extra_lock);

	if (ret) {
		for (i = 0; i <= sbi_scratch_last_hartid(); i++) {
			/*得到sbi_scratch地址*/
			rscratch = sbi_hartid_to_scratch(i);
			if (!rscratch)
				continue;
			/*sbi_scratch地址加上offset后的地址，就是从这里开始分配size大小字节的内存*/
			ptr = sbi_scratch_offset_ptr(rscratch, ret);
			sbi_memset(ptr, 0, size);
		}
	}

	return ret;
}

void sbi_scratch_free_offset(unsigned long offset)
{
	if ((offset < SBI_SCRATCH_EXTRA_SPACE_OFFSET) ||
	    (SBI_SCRATCH_SIZE <= offset))
		return;

	/*
	 * We don't actually free-up because it's a simple
	 * brain-dead allocator.
	 */
}
