// SPDX-License-Identifier: GPL-2.0
/*
 * qkx_stall - CPU progress observer for the kexec handoff investigation.
 *
 * One pinned hrtimer per online CPU.  Each observer reports only OTHER
 * CPUs, so any healthy CPU can report a wedged one.  Per CPU we record the
 * last context switch, the last idle-loop iteration and a scheduler tick
 * count, which lets a report distinguish:
 *   - ticks advancing, no switch: IRQs work, CPU spins with preemption off
 *     (or a lone CPU-bound task),
 *   - ticks frozen: CPU has IRQs masked, or is stuck in a hardware access /
 *     power transition,
 *   - idle, no wakeup: our own 5s timer is not being delivered to it.
 * No live PC is available (no NMI backtrace on this platform); the task
 * shown is the one currently on the CPU.
 *
 * All hooks are no-ops unless qkx_stallmon=1 is on the command line.
 * With a single online CPU there is no observer; only the heartbeat's
 * absence is informative.
 */

#include "sched.h"
#include <linux/hrtimer.h>
#include <linux/cpuhotplug.h>
#include <linux/ktime.h>
#include <linux/printk.h>
#include <linux/qkx_diag.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <asm/irq_regs.h>
#include <asm/sysreg.h>

#define QKX_TIMER_PERIOD_NS	(2ULL * NSEC_PER_SEC)

/* qkx_beat=1 turns the observer tick into a dense host-visible heartbeat. */
static bool qkx_beat;
static int __init qkx_beat_setup(char *s)
{
	return kstrtobool(s, &qkx_beat);
}
early_param("qkx_beat", qkx_beat_setup);
#define QKX_STALL_BUSY_NS	(4ULL * NSEC_PER_SEC)
#define QKX_STALL_IDLE_NS	(15ULL * NSEC_PER_SEC)
#define QKX_REP_INTERVAL_NS	(3ULL * NSEC_PER_SEC)
#define QKX_HB_INTERVAL_NS	(60ULL * NSEC_PER_SEC)

static DEFINE_PER_CPU(u64, qkx_last_switch);
static DEFINE_PER_CPU(u64, qkx_last_idle);
static DEFINE_PER_CPU(u64, qkx_tick_count);
static DEFINE_PER_CPU(u64, qkx_next_report);
static DEFINE_PER_CPU(u64, qkx_rep_ticks);
static DEFINE_PER_CPU(u64, qkx_rep_poll);
DEFINE_PER_CPU(u64, qkx_poll_count);
static DEFINE_PER_CPU(struct hrtimer, qkx_obs_timer);
DEFINE_PER_CPU(struct qkx_lockwait, qkx_lockwait);
DEFINE_PER_CPU(struct qkx_ctx, qkx_ctx[QKX_CTX_NR]);
struct qkx_ctxarea __iomem *qkx_ctxarea;
bool qkx_irqmask_ready;
static DEFINE_PER_CPU(bool, qkx_irqmask_busy);
static DEFINE_PER_CPU(struct qkx_ctx, qkx_irqmask_save);
struct qkx_mmio_state {
	const void *addr, *caller;
	u64 since;
	bool write, busy;
};
static DEFINE_PER_CPU(struct qkx_mmio_state, qkx_mmio);

struct qkx_tickpc {
	u64 ts;
	unsigned long pc;
	bool user;
};
#define QKX_TICKPC_NR 32
static DEFINE_PER_CPU(struct qkx_tickpc[QKX_TICKPC_NR], qkx_tickpc);
static DEFINE_PER_CPU(int, qkx_tickpc_idx);
static int qkx_tickpc_ready;

void notrace qkx_tick_note(unsigned long pc, bool user)
{
	int i;

	if (!READ_ONCE(qkx_tickpc_ready))
		return;
	i = this_cpu_add_return(qkx_tickpc_idx, 1) - 1;
	this_cpu_write(qkx_tickpc[i & (QKX_TICKPC_NR - 1)].ts, ktime_get_mono_fast_ns());
	this_cpu_write(qkx_tickpc[i & (QKX_TICKPC_NR - 1)].pc, pc);
	this_cpu_write(qkx_tickpc[i & (QKX_TICKPC_NR - 1)].user, user);
}

void notrace qkx_mmio_enter(const void *addr, const void *caller, bool write)
{
	struct qkx_mmio_state *m;

	if (!READ_ONCE(qkx_irqmask_ready))
		return;
	m = this_cpu_ptr(&qkx_mmio);
	if (m->busy)
		return;
	m->busy = true;
	m->addr = addr;
	m->caller = caller;
	m->write = write;
	m->since = ktime_get_mono_fast_ns();
}

void notrace qkx_mmio_exit(void)
{
	if (READ_ONCE(qkx_irqmask_ready))
		this_cpu_write(qkx_mmio.busy, false);
}

void notrace qkx_irqmask_note(bool disabled, const void *caller)
{
	struct qkx_ctx *c;

	if (!READ_ONCE(qkx_irqmask_ready) || this_cpu_read(qkx_irqmask_busy))
		return;
	this_cpu_write(qkx_irqmask_busy, true);
	c = this_cpu_ptr(&qkx_ctx[QKX_CTX_IRQMASK]);
	if (disabled) {
		if (!READ_ONCE(c->fn))
			qkx_ctx_enter(QKX_CTX_IRQMASK,
				this_cpu_ptr(&qkx_irqmask_save), caller, 0,
				"local-irq-disabled");
	} else if (READ_ONCE(c->fn)) {
		qkx_ctx_exit(QKX_CTX_IRQMASK,
			     this_cpu_ptr(&qkx_irqmask_save));
	}
	this_cpu_write(qkx_irqmask_busy, false);
}

static void notrace qkx_ctx_mirror(int kind, const struct qkx_ctx *c)
{
	struct qkx_ctxarea __iomem *a = READ_ONCE(qkx_ctxarea);
	unsigned int cpu = smp_processor_id();

	if (!a || cpu >= NR_CPUS)
		return;
	memcpy_toio(&a->rec[cpu * QKX_CTX_NR + kind], c, sizeof(*c));
}

/*
 * Apps watchdog (captured DT: qcom,wdt@17c10000, WDT_HZ 32765).  Once boot
 * initcalls are over (they force it off), every observer pets it.  Petting
 * stops when every other CPU is stalled or any CPU has been stalled for 60s,
 * so the bite warm-resets with the retained log and context area intact.
 */
static bool qkx_wdt_bite = true;
module_param(qkx_wdt_bite, bool, 0444);
#define QKX_WDT_HZ		32765
#define QKX_WDT_BITE_S		25
#define QKX_WDT_GIVEUP_NS	(60ULL * NSEC_PER_SEC)
static void __iomem *qkx_wdt_regs;
static atomic_t qkx_wdt_state = ATOMIC_INIT(0);	/* 0 off, 1 arming, 2 armed */
static u64 qkx_first_stall;
static u64 qkx_wdt_last_log;

static void qkx_wdt_pet(u64 now)
{
	if (!qkx_wdt_bite || system_state != SYSTEM_RUNNING)
		return;
	if (atomic_read(&qkx_wdt_state) != 2) {
		if (atomic_cmpxchg(&qkx_wdt_state, 0, 1) != 0 || !qkx_wdt_regs)
			return;
		writel_relaxed(0, qkx_wdt_regs + 0x08);
		writel_relaxed(1, qkx_wdt_regs + 0x04);
		writel_relaxed(QKX_WDT_BITE_S * QKX_WDT_HZ, qkx_wdt_regs + 0x10);
		writel_relaxed(QKX_WDT_BITE_S * QKX_WDT_HZ, qkx_wdt_regs + 0x14);
		writel_relaxed(1, qkx_wdt_regs + 0x08);
		mb();
		pr_info("qkx-wdt: apps watchdog armed, bite %us, en=%x\n",
			QKX_WDT_BITE_S, readl_relaxed(qkx_wdt_regs + 0x08));
		atomic_set(&qkx_wdt_state, 2);
		return;
	}
	writel_relaxed(1, qkx_wdt_regs + 0x04);
	if (qkx_ctxarea)
		writeq_relaxed(now, &qkx_ctxarea->last_pet);
	if (now - qkx_wdt_last_log >= 5ULL * NSEC_PER_SEC) {
		qkx_wdt_last_log = now;
		pr_info("qkx-wdt: pet now=%llu en=%x\n",
			(unsigned long long)now,
			readl_relaxed(qkx_wdt_regs + 0x08));
	}
}

void notrace qkx_ctx_enter(int kind, struct qkx_ctx *save, const void *fn,
			   int irq, const char *what)
{
	struct qkx_ctx *c = this_cpu_ptr(&qkx_ctx[kind]);

	*save = *c;
	c->fn = NULL;
	barrier();
	c->irq = irq;
	strlcpy(c->what, what ? what : "?", sizeof(c->what));
	c->since = ktime_get_mono_fast_ns();
	barrier();
	WRITE_ONCE(c->fn, fn);
	qkx_ctx_mirror(kind, c);
}

void notrace qkx_ctx_set_args(int kind, u64 arg0, u64 arg1)
{
	struct qkx_ctx *c = this_cpu_ptr(&qkx_ctx[kind]);

	c->arg0 = arg0;
	c->arg1 = arg1;
	qkx_ctx_mirror(kind, c);
}

void notrace qkx_ctx_exit(int kind, const struct qkx_ctx *save)
{
	struct qkx_ctx *c = this_cpu_ptr(&qkx_ctx[kind]);

	WRITE_ONCE(c->fn, NULL);
	barrier();
	*c = *save;
	qkx_ctx_mirror(kind, c);
}

void qkx_attr_enter(struct qkx_attr_save *save, const void *fn,
		    const char *what)
{
	struct task_struct *t = current;

	save->fn = t->qkx_attr_fn;
	memcpy(save->what, t->qkx_attr_what, sizeof(save->what));
	save->since = t->qkx_attr_since;
	WRITE_ONCE(t->qkx_attr_fn, NULL);
	barrier();
	strlcpy(t->qkx_attr_what, what ? what : "?", sizeof(t->qkx_attr_what));
	t->qkx_attr_since = ktime_get_mono_fast_ns();
	barrier();
	WRITE_ONCE(t->qkx_attr_fn, fn);
}

void qkx_attr_exit(const struct qkx_attr_save *save)
{
	struct task_struct *t = current;

	WRITE_ONCE(t->qkx_attr_fn, NULL);
	barrier();
	memcpy(t->qkx_attr_what, save->what, sizeof(t->qkx_attr_what));
	t->qkx_attr_since = save->since;
	barrier();
	WRITE_ONCE(t->qkx_attr_fn, save->fn);
}

/* Caller holds rcu_read_lock(); p is that CPU's current task. */
static void qkx_report_attr(int cpu, struct task_struct *p, u64 now)
{
	const void *fn = READ_ONCE(p->qkx_attr_fn);
	char what[32];
	u64 since, age;

	if (!fn)
		return;
	memcpy(what, p->qkx_attr_what, sizeof(what));
	what[sizeof(what) - 1] = 0;
	since = READ_ONCE(p->qkx_attr_since);
	age = now > since ? now - since : 0;
	pr_err("qkx-ctx: cpu%d task %s/%d inside task-op for %llu.%03llus: %s fn=%pS\n",
	       cpu, p->comm, p->pid, age / NSEC_PER_SEC,
	       (age % NSEC_PER_SEC) / NSEC_PER_MSEC, what, fn);
}

static void qkx_report_ctx(int cpu, u64 now)
{
	static const char * const kinds[] = { "hardirq", "sysfs-attr(unused)", "scm", "clk", "gdsc", "spmi", "cache", "cpu-pm", "irq-mask", "ufs-mmio" };
	int k;

	for (k = 0; k < QKX_CTX_NR; k++) {
		struct qkx_ctx c = per_cpu(qkx_ctx[k], cpu);
		u64 age;

		if (!c.fn)
			continue;
		c.what[sizeof(c.what) - 1] = 0;
		age = now > c.since ? now - c.since : 0;
		pr_err("qkx-ctx: cpu%d inside %s for %llu.%03llus: irq=%d %s fn=%pS arg=%llx/%llx\n",
		       cpu, kinds[k], age / NSEC_PER_SEC,
		       (age % NSEC_PER_SEC) / NSEC_PER_MSEC, c.irq, c.what,
		       c.fn, c.arg0, c.arg1);
	}
}

/*
 * IPI probe: sent to a CPU that stopped ticking. If it arrives, the handler
 * records the interrupted context (the live PC) and the local virtual timer
 * state; if it never arrives, interrupt delivery to that CPU is dead.
 */
struct qkx_probe {
	u64 sent, got;
	unsigned long pc, lr;
	u64 pstate, cntv_ctl, cntv_cval, cntvct, isr;
	bool user;
	int inflight;	/* 1 while the csd may be queued: never reuse it */
};
static DEFINE_PER_CPU(struct qkx_probe, qkx_probe);
static DEFINE_PER_CPU(call_single_data_t, qkx_probe_csd);

static void qkx_probe_fn(void *unused)
{
	struct qkx_probe *pr = this_cpu_ptr(&qkx_probe);
	struct pt_regs *regs = get_irq_regs();

	if (regs) {
		pr->pc = regs->pc;
		pr->lr = regs->regs[30];
		pr->pstate = regs->pstate;
		pr->user = user_mode(regs);
	}
	pr->cntv_ctl = read_sysreg(cntv_ctl_el0);
	pr->cntv_cval = read_sysreg(cntv_cval_el0);
	pr->cntvct = read_sysreg(cntvct_el0);
	pr->isr = read_sysreg(isr_el1);
	smp_wmb();
	WRITE_ONCE(pr->got, ktime_get_mono_fast_ns());
	smp_store_release(&pr->inflight, 0);
}

static void qkx_probe_cpu(int cpu, u64 now)
{
	struct qkx_probe *pr = per_cpu_ptr(&qkx_probe, cpu);
	call_single_data_t *csd = per_cpu_ptr(&qkx_probe_csd, cpu);
	u64 got = READ_ONCE(pr->got);

	smp_rmb();
	if (READ_ONCE(pr->inflight)) {
		pr_err("qkx-ipi: cpu%d has NOT answered an IPI sent %llu.%03llus ago\n",
		       cpu, (now - pr->sent) / NSEC_PER_SEC,
		       ((now - pr->sent) % NSEC_PER_SEC) / NSEC_PER_MSEC);
		return;
	}
	if (pr->sent && got >= pr->sent) {
		if (pr->user)
			pr_err("qkx-ipi: cpu%d answered after %lluus: USER pc=%lx pstate=%llx cntv_ctl=%llx cval-cnt=%lld isr=%llx\n",
			       cpu, (got - pr->sent) / NSEC_PER_USEC, pr->pc,
			       pr->pstate, pr->cntv_ctl,
			       (s64)(pr->cntv_cval - pr->cntvct), pr->isr);
		else
			pr_err("qkx-ipi: cpu%d answered after %lluus: pc=%pS lr=%pS pstate=%llx cntv_ctl=%llx cval-cnt=%lld isr=%llx\n",
			       cpu, (got - pr->sent) / NSEC_PER_USEC,
			       (void *)pr->pc, (void *)pr->lr, pr->pstate,
			       pr->cntv_ctl, (s64)(pr->cntv_cval - pr->cntvct),
			       pr->isr);
	}
	/* Observers on several CPUs may get here; only one may queue it. */
	if (cmpxchg(&pr->inflight, 0, 1) != 0)
		return;
	csd->func = qkx_probe_fn;
	csd->info = NULL;
	pr->sent = now;
	smp_wmb();
	smp_call_function_single_async(cpu, csd);
}

void qkx_stall_note_switch(void)
{
	this_cpu_write(qkx_last_switch, ktime_get_mono_fast_ns());
}

void qkx_stall_note_idle(void)
{
	this_cpu_write(qkx_last_idle, ktime_get_mono_fast_ns());
}

void qkx_stall_note_tick(void)
{
	struct qkx_ctxarea __iomem *a = READ_ONCE(qkx_ctxarea);
	struct pt_regs *regs = get_irq_regs();
	unsigned int cpu = smp_processor_id();

	this_cpu_inc(qkx_tick_count);
	if (a && regs && cpu < QKX_PCHIST_CPUS) {
		u64 __iomem *h = (void __iomem *)a + QKX_PCHIST_OFF + cpu *
			(1 + QKX_PCHIST_N * QKX_PCHIST_WORDS) * sizeof(u64);
		u64 n = this_cpu_read(qkx_tick_count);
		u64 slot = 1 + (n % QKX_PCHIST_N) * QKX_PCHIST_WORDS;
		u64 pmr = read_sysreg_s(SYS_ICC_PMR_EL1);
		u64 grp = read_sysreg_s(SYS_ICC_IGRPEN1_EL1);

		writeq_relaxed(instruction_pointer(regs) |
			       (user_mode(regs) ? 1 : 0), &h[slot]);
		writeq_relaxed(read_sysreg(daif), &h[slot + 1]);
		writeq_relaxed((grp << 32) | (pmr & 0xffffffff), &h[slot + 2]);
		writeq_relaxed(n, &h[0]);
	}
}

static void qkx_report_lockwait(int cpu, u64 now)
{
	struct qkx_lockwait w = per_cpu(qkx_lockwait, cpu);
	u64 age;

	if (!w.lock)
		return;
	age = now > w.since ? now - w.since : 0;
	pr_err("qkx-lock: cpu%d spinning %llu.%03llus on lock %px from %pS <- %pS <- %pS <- %pS\n",
	       cpu, age / NSEC_PER_SEC, (age % NSEC_PER_SEC) / NSEC_PER_MSEC,
	       w.lock, (void *)w.ip[0], (void *)w.ip[1], (void *)w.ip[2],
	       (void *)w.ip[3]);
}

static void qkx_report_tickpc(int cpu, u64 now)
{
	int i;

	for (i = 0; i < 8; i++) {
		int idx = (per_cpu(qkx_tickpc_idx, cpu) - 1 - i) & (QKX_TICKPC_NR - 1);
		struct qkx_tickpc *t = &per_cpu(qkx_tickpc, cpu)[idx];

		if (!t->ts)
			continue;
		pr_err("qkx-tickpc: cpu%d -%d %llu.%03llus %s %pS\n", cpu, i,
		       (now - t->ts) / NSEC_PER_SEC,
		       ((now - t->ts) % NSEC_PER_SEC) / NSEC_PER_MSEC,
		       t->user ? "U" : "K", (void *)t->pc);
	}
}

static void qkx_report_mmio(int cpu, u64 now)
{
	struct qkx_mmio_state m = per_cpu(qkx_mmio, cpu);

	if (!m.busy)
		return;
	pr_err("qkx-mmio: cpu%d stuck %s addr=%px for %llu.%03llus caller=%pS\n",
	       cpu, m.write ? "write" : "read", m.addr,
	       (now - m.since) / NSEC_PER_SEC,
	       ((now - m.since) % NSEC_PER_SEC) / NSEC_PER_MSEC, m.caller);
}

static void qkx_check_cpu(int cpu, u64 now)
{
	struct task_struct *p;
	u64 act, age, ticks;
	bool idle;

	rcu_read_lock();
	p = READ_ONCE(cpu_curr(cpu));
	if (!p)
		goto out;
	idle = is_idle_task(p);
	act = idle ? READ_ONCE(per_cpu(qkx_last_idle, cpu))
		   : READ_ONCE(per_cpu(qkx_last_switch, cpu));
	age = now > act ? now - act : 0;
	if (age < (idle ? QKX_STALL_IDLE_NS : QKX_STALL_BUSY_NS))
		goto out;
	if (!READ_ONCE(qkx_first_stall))
		WRITE_ONCE(qkx_first_stall, now);
	if (now < per_cpu(qkx_next_report, cpu))
		goto out;
	per_cpu(qkx_next_report, cpu) = now + QKX_REP_INTERVAL_NS;
	ticks = READ_ONCE(per_cpu(qkx_tick_count, cpu));
	{
		u64 poll = READ_ONCE(per_cpu(qkx_poll_count, cpu));
		pr_err("qkx-sched: cpu%d %s %llu.%03llus task=%s pid=%d state=%lx ticks=%llu (+%llu) poll=%llu (+%llu)\n",
		       cpu, idle ? "IDLE-NOWAKE" : "NO-SWITCH",
		       age / NSEC_PER_SEC, (age % NSEC_PER_SEC) / NSEC_PER_MSEC,
		       p->comm, p->pid, (unsigned long)p->state,
		       ticks, ticks - per_cpu(qkx_rep_ticks, cpu), poll,
		       poll - per_cpu(qkx_rep_poll, cpu));
		per_cpu(qkx_rep_poll, cpu) = poll;
	}
	per_cpu(qkx_rep_ticks, cpu) = ticks;
	qkx_report_lockwait(cpu, now);
	qkx_report_ctx(cpu, now);
	qkx_report_attr(cpu, p, now);
	qkx_report_mmio(cpu, now);
	qkx_report_tickpc(cpu, now);
	qkx_gic_dump_cpu(cpu);
	qkx_probe_cpu(cpu, now);
out:
	rcu_read_unlock();
}

static enum hrtimer_restart qkx_obs_fn(struct hrtimer *t)
{
	u64 now = ktime_get_mono_fast_ns();
	int self = smp_processor_id();
	int cpu;

	u64 first;

	for_each_online_cpu(cpu)
		if (cpu != self)
			qkx_check_cpu(cpu, now);

	first = READ_ONCE(qkx_first_stall);
	if (!first || now - first < QKX_WDT_GIVEUP_NS)
		qkx_wdt_pet(now);
	else if (atomic_read(&qkx_wdt_state) == 2 &&
		 atomic_xchg(&qkx_wdt_state, 3) == 2)
		pr_emerg("qkx-wdt: a CPU has been stalled %llus; no longer petting, expect warm reset\n",
			 (now - first) / NSEC_PER_SEC);

	/* Dense sequenced beat: lets the host tell a device reset (beats stop)
	 * apart from losing the USB/ECM transport (beats resume on reconnect,
	 * with a gap in the sequence). Only the lowest online CPU emits it. */
	if (qkx_beat && self == cpumask_first(cpu_online_mask)) {
		static u64 qkx_beat_seq;

		pr_info("qkx-beat: seq=%llu t=%llu.%03llus online=%*pbl\n",
			(unsigned long long)++qkx_beat_seq,
			(unsigned long long)(now / NSEC_PER_SEC),
			(unsigned long long)((now % NSEC_PER_SEC) / 1000000),
			cpumask_pr_args(cpu_online_mask));
	}

	/* Heartbeat from the lowest online CPU proves an observer is alive. */
	if (self == cpumask_first(cpu_online_mask)) {
		static u64 qkx_last_hb;

		if (now - qkx_last_hb >= QKX_HB_INTERVAL_NS) {
			qkx_last_hb = now;
			pr_info("qkx-sched: alive cpu%d jiffies=%lu online=%*pbl\n",
				self, jiffies, cpumask_pr_args(cpu_online_mask));
		}
	}

	hrtimer_forward_now(t, ns_to_ktime(QKX_TIMER_PERIOD_NS));
	return HRTIMER_RESTART;
}

/* cpuhp AP_ONLINE_DYN callbacks run on the CPU being brought up/down. */
static int qkx_stall_cpu_online(unsigned int cpu)
{
	struct hrtimer *t = this_cpu_ptr(&qkx_obs_timer);
	u64 now = ktime_get_mono_fast_ns();

	this_cpu_write(qkx_last_switch, now);
	this_cpu_write(qkx_last_idle, now);
	this_cpu_write(qkx_next_report, 0);
	hrtimer_init(t, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
	t->function = qkx_obs_fn;
	hrtimer_start(t, ns_to_ktime(QKX_TIMER_PERIOD_NS),
		      HRTIMER_MODE_REL_PINNED);
	pr_info("qkx-hp: cpu%u online, observer armed\n", cpu);
	return 0;
}

static int qkx_stall_cpu_offline(unsigned int cpu)
{
	hrtimer_cancel(this_cpu_ptr(&qkx_obs_timer));
	pr_info("qkx-hp: cpu%u going offline\n", cpu);
	return 0;
}

static int __init qkx_stall_init(void)
{
	int ret;

	BUILD_BUG_ON(sizeof(struct qkx_ctxarea) +
		     NR_CPUS * QKX_CTX_NR * sizeof(struct qkx_ctx) > QKX_PCHIST_OFF);
	if (!qkx_stallmon)
		return 0;
	if (qkx_wdt_bite)
		qkx_wdt_regs = ioremap(0x17c10000, 0x1000);

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "qkx/stall:online",
				qkx_stall_cpu_online, qkx_stall_cpu_offline);
	if (ret < 0) {
		pr_err("qkx: stall monitor hotplug setup failed: %d\n", ret);
		return ret;
	}
	WRITE_ONCE(qkx_irqmask_ready, true);
	WRITE_ONCE(qkx_tickpc_ready, 1);
	pr_info("qkx: stall monitor active: period %llus busy-warn %llus idle-warn %llus\n",
		QKX_TIMER_PERIOD_NS / NSEC_PER_SEC,
		QKX_STALL_BUSY_NS / NSEC_PER_SEC,
		QKX_STALL_IDLE_NS / NSEC_PER_SEC);
	return 0;
}
late_initcall(qkx_stall_init);
