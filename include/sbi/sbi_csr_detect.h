/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Atish Patra <atish.patra@wdc.com>
 */

#ifndef __SBI_CSR_DETECT__H
#define __SBI_CSR_DETECT__H

#include <sbi/riscv_encoding.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_trap.h>

#define csr_read_allowed(csr_num, trap)					\
	({								\
	register ulong tinfo asm("a3") = (ulong)trap;			\
	register ulong ttmp asm("a4");					\
	register ulong mtvec = sbi_hart_expected_trap_addr();		\
	register ulong ret = 0;						\
	((struct sbi_trap_info *)(trap))->cause = 0;			\
	asm volatile(							\
		"add %[ttmp], %[tinfo], zero\n"				\
		"csrrw %[mtvec], " STR(CSR_MTVEC) ", %[mtvec]\n"	\
		"csrr %[ret], %[csr]\n"					\
		"csrw " STR(CSR_MTVEC) ", %[mtvec]"			\
	    : [mtvec] "+&r"(mtvec), [tinfo] "+&r"(tinfo),		\
	      [ttmp] "+&r"(ttmp), [ret] "=&r" (ret)			\
	    : [csr] "i" (csr_num)					\
	    : "memory");						\
	ret;								\
	})								\

/*
csr_write_allowed函数解析：
局部变量mtvec为异常向量基地址
第一行汇编add，表示将tinfo赋值给ttmp
第二行汇编csrrw,设置csr寄存器mtvec为异常向量基地址，并设置局部变量mtvec为原来csr寄存器mtvec的值
第三行汇编csrw，将参数value的值，赋值给csr_num寄存器
第四行汇编csrw, 局部变量mtvec的值赋值给csr_mtvec，恢复以前的值
*/
#define csr_write_allowed(csr_num, trap, value)				\
	({								\
	register ulong tinfo asm("a3") = (ulong)trap;			\
	register ulong ttmp asm("a4");					\
	register ulong mtvec = sbi_hart_expected_trap_addr();/*异常向量基地址*/		\
	((struct sbi_trap_info *)(trap))->cause = 0;			\
	asm volatile(							\
		"add %[ttmp], %[tinfo], zero\n"				\
		"csrrw %[mtvec], " STR(CSR_MTVEC) ", %[mtvec]\n" /*csrrw rd, csr, rs1先把csr的值即为t，把寄存器rs1的值写入csr，然后再把t写入rd中*/	\
		"csrw %[csr], %[val]\n"					\
		"csrw " STR(CSR_MTVEC) ", %[mtvec]"			\
	    : [mtvec] "+&r"(mtvec),					\
	      [tinfo] "+&r"(tinfo), [ttmp] "+&r"(ttmp)			\
	    : [csr] "i" (csr_num), [val] "r" (value)			\
	    : "memory");						\
	})								\

#endif
