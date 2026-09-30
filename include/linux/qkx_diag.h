/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_QKX_DIAG_H
#define _LINUX_QKX_DIAG_H

/*
 * Shared opt-in diagnostics for the kexec handoff investigation.
 * Enabled with qkx_stallmon=1 on the kernel command line; all hooks are
 * cheap no-ops otherwise.  The goal is to locate the first loss of CPU,
 * interrupt or RPMh progress when alternate Android stalls.
 */
#include <linux/percpu.h>
#include <linux/types.h>

extern bool qkx_stallmon;

/* Current contended spinlock wait on this CPU (lock == NULL: none). */
struct qkx_lockwait {
	void *lock;
	unsigned long ip[4];
	u64 since;
};
DECLARE_PER_CPU(struct qkx_lockwait, qkx_lockwait);

/* kernel/sched stall monitor hooks (safe from any context). */
extern void qkx_stall_note_switch(void);
extern void qkx_stall_note_idle(void);
extern void qkx_stall_note_tick(void);
DECLARE_PER_CPU(u64, qkx_poll_count);

/*
 * Per-CPU "currently inside" records for the stall monitor. A CPU that stalls
 * on a hardware access takes no interrupts, so these name what it was doing.
 * enter returns the previous record's token so nesting restores it.
 */
enum { QKX_CTX_IRQ, QKX_CTX_ATTR, QKX_CTX_SCM, QKX_CTX_CLK, QKX_CTX_GDSC,
       QKX_CTX_SPMI, QKX_CTX_CACHE, QKX_CTX_CPUPM, QKX_CTX_IRQMASK, QKX_CTX_UFS,
       QKX_CTX_NR };
struct qkx_ctx {
	const void *fn;		/* handler / show / store */
	int irq;
	char what[48];		/* device or irq name */
	u64 since;
	u64 arg0, arg1;
};
DECLARE_PER_CPU(struct qkx_ctx, qkx_ctx[QKX_CTX_NR]);
extern void qkx_ctx_enter(int kind, struct qkx_ctx *save, const void *fn,
			  int irq, const char *what);
extern void qkx_ctx_exit(int kind, const struct qkx_ctx *save);
extern void qkx_ctx_set_args(int kind, u64 arg0, u64 arg1);
extern void qkx_gic_dump_cpu(unsigned int cpu);
extern bool qkx_irqmask_ready;
extern void notrace qkx_irqmask_note(bool disabled, const void *caller);
extern void notrace qkx_mmio_enter(const void *addr, const void *caller, bool write);
extern void notrace qkx_tick_note(unsigned long pc, bool user);
extern void notrace qkx_mmio_exit(void);

struct qkx_attr_save {
	const void *fn;
	char what[32];
	u64 since;
};
extern void qkx_attr_enter(struct qkx_attr_save *save, const void *fn,
			   const char *what);
extern void qkx_attr_exit(const struct qkx_attr_save *save);

/*
 * The last 4 KiB of the retained log page mirror every CPU's qkx_ctx records
 * so they survive a watchdog warm reset after all CPUs have stalled.
 */
#define QKX_CTXAREA_SIZE	0x3000
#define QKX_CTXAREA_MAGIC	0x514b5843	/* "QKXC" */
struct qkx_ctxarea {
	u32 magic;
	u32 nr_cpus;
	u32 nr_kinds;
	u32 rec_size;
	u64 text;		/* runtime _text, for KASLR-free symbolization */
	u64 last_pet;		/* monotonic ns of last watchdog pet */
	struct qkx_ctx rec[];	/* [cpu][kind] */
};
extern struct qkx_ctxarea __iomem *qkx_ctxarea;

/*
 * Per-CPU tick PC history at ctxarea + QKX_PCHIST_OFF: for each of 8 CPUs,
 * u64 count followed by QKX_PCHIST_N entries of { pc, daif, gic }.
 * Bit 0 of pc marks user mode; gic packs ICC_PMR_EL1 and ICC_IGRPEN1_EL1.
 */
#define QKX_PCHIST_OFF		0x2800
#define QKX_PCHIST_CPUS		8
#define QKX_PCHIST_N		7
#define QKX_PCHIST_WORDS	3

#endif /* _LINUX_QKX_DIAG_H */
