/* //20100930 jack_wong for asus debug mechanisms +++++
 *  asusdebug.c
 * //20100930 jack_wong for asus debug mechanisms -----
 *
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/rtc.h>
#include <linux/list.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
extern int g_user_dbg_mode;

#include <linux/rtc.h>
#include "locking/rtmutex_common.h"
// ASUS_BSP +++
char evtlog_bootup_reason[100];
char evtlog_poweroff_reason[100];
char evtlog_warm_reset_reason[100];
extern u16 warm_reset_value;
// ASUS_BSP ---

#ifdef CONFIG_HAS_EARLYSUSPEND
int entering_suspend = 0;
#endif
phys_addr_t PRINTK_BUFFER_PA = 0xA0000000;
void *PRINTK_BUFFER_VA;
phys_addr_t RTB_BUFFER_PA = 0xA0000000 + SZ_1M;
extern struct timezone sys_tz;
#define RT_MUTEX_HAS_WAITERS    1UL
#define RT_MUTEX_OWNER_MASKALL  1UL
struct mutex fake_mutex;
struct completion fake_completion;
struct rt_mutex fake_rtmutex;
struct work_struct slow_work;
static unsigned long trigger_slowlog_time;
int asus_rtc_read_time(struct rtc_time *tm)
{
	struct timespec ts;

	getnstimeofday(&ts);
	ts.tv_sec -= sys_tz.tz_minuteswest * 60;
	rtc_time_to_tm(ts.tv_sec, tm);
	printk("now %04d%02d%02d-%02d%02d%02d, tz=%d\r\n", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, sys_tz.tz_minuteswest);
	return 0;
}
EXPORT_SYMBOL(asus_rtc_read_time);

//--------------------   debug message logger   ------------------------------------
struct workqueue_struct *ASUSDebugMsg_workQueue;
EXPORT_SYMBOL(ASUSDebugMsg_workQueue);
//--------------  phone hang log part  --------------------------------------------------

///////////////////////////////////////////////////////////////////////////////////////////////////
//                             all thread information
///////////////////////////////////////////////////////////////////////////////////////////////////


/*
 * memset for non cached memory
 */
void *memset_nc(void *s, int c, size_t count)
{
	u8 *p = s;

	while (count--)
		*p++ = c;
	return s;
}
EXPORT_SYMBOL(memset_nc);

static char *g_phonehang_log;
static int g_iPtr = 0;
int save_log(const char *f, ...)
{
	va_list args;
	int len;

	if (g_iPtr < PHONE_HANG_LOG_SIZE) {
		va_start(args, f);
		len = vsnprintf(g_phonehang_log + g_iPtr, PHONE_HANG_LOG_SIZE - g_iPtr, f, args);
		va_end(args);

		if (g_iPtr < PHONE_HANG_LOG_SIZE) {
			g_iPtr += len;
			return 0;
		}
	}
	g_iPtr = PHONE_HANG_LOG_SIZE;
	return -1;
}

static char *task_state_array[] = {
	"RUNNING",              /*  0 */
	"INTERRUPTIBLE",        /*  1 */
	"UNINTERRUPTIB",        /*  2 */
	"STOPPED",              /*  4 */
	"TRACED",               /*  8 */
	"EXIT ZOMBIE",          /* 16 */
	"EXIT DEAD",            /* 32 */
	"DEAD",                 /* 64 */
	"WAKEKILL",             /* 128 */
	"WAKING",               /* 256 */
	"PARKED"                /* 512 */
};
struct thread_info_save;
struct thread_info_save {
	struct task_struct *		pts;
	pid_t				pid;
	u64				sum_exec_runtime;
	u64				vruntime;
	struct thread_info_save *	pnext;
};
static char *print_state(long state)
{
	int i;

	if (state == 0)
		return task_state_array[0];
	for (i = 1; i <= 10; i++)
		if (1 << (i - 1) & state)
			return task_state_array[i];
	return "NOTFOUND";
}

/*
 * Ease the printing of nsec fields:
 */
static long long nsec_high(unsigned long long nsec)
{
	if ((long long)nsec < 0) {
		nsec = -nsec;
		do_div(nsec, 1000000);
		return -nsec;
	}
	do_div(nsec, 1000000);

	return nsec;
}

static unsigned long nsec_low(unsigned long long nsec)
{
	unsigned long long nsec1;

	if ((long long)nsec < 0)
		nsec = -nsec;

	nsec1 = do_div(nsec, 1000000);
	return do_div(nsec1, 1000000);
}
#define MAX_STACK_TRACE_DEPTH   64
struct stack_trace_data {
	struct stack_trace *	trace;
	unsigned int		no_sched_functions;
	unsigned int		skip;
};

struct stackframe {
	unsigned long	fp;
	unsigned long	sp;
	unsigned long	lr;
	unsigned long	pc;
};
int unwind_frame(struct stackframe *frame);
void notrace walk_stackframe(struct stackframe *frame, int (*fn)(struct stackframe *, void *), void *data);

void show_stack1(struct task_struct *p1, void *p2)
{
	struct stack_trace trace;
	unsigned long *entries;
	int i;

	entries = kmalloc(MAX_STACK_TRACE_DEPTH * sizeof(*entries), GFP_KERNEL);
	if (!entries) {
		printk("entries malloc failure\n");
		return;
	}
	trace.nr_entries = 0;
	trace.max_entries = MAX_STACK_TRACE_DEPTH;
	trace.entries = entries;
	trace.skip = 0;
	save_stack_trace_tsk(p1, &trace);

	for (i = 0; i < trace.nr_entries; i++)
		if (entries[i] != ULONG_MAX)
			save_log("%pS\n", (void *)entries[i]);
	kfree(entries);
}


#define SPLIT_NS(x) nsec_high(x), nsec_low(x)
void print_all_thread_info(void)
{
	struct task_struct *pts;
	struct thread_info *pti;
	struct rtc_time tm;

	asus_rtc_read_time(&tm);

	g_phonehang_log = (char *)PHONE_HANG_LOG_BUFFER;
	g_iPtr = 0;
	memset_nc(g_phonehang_log, 0, PHONE_HANG_LOG_SIZE);

	save_log("PhoneHang-%04d%02d%02d-%02d%02d%02d.txt  ---  ASUS_SW_VER : %s----------------------------------------------\r\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ASUS_SW_VER);
	save_log(" pID----ppID----NAME----------------SumTime---vruntime--SPri-NPri-State----------PmpCnt-Binder----Waiting\r\n");

	for_each_process(pts){
		pti = task_thread_info(pts);
		save_log("-----------------------------------------------------\r\n");
		save_log(" %-7d", pts->pid);

		if (pts->parent)
			save_log("%-8d", pts->parent->pid);
		else
			save_log("%-8d", 0);

		save_log("%-20s", pts->comm);
		save_log("%lld.%06ld", SPLIT_NS(pts->se.sum_exec_runtime));
		save_log("     %lld.%06ld     ", SPLIT_NS(pts->se.vruntime));
		save_log("%-5d", pts->static_prio);
		save_log("%-5d", pts->normal_prio);
		save_log("%-15s", print_state((pts->state & TASK_REPORT) | pts->exit_state));

		save_log("%-6d", pti->preempt_count);


		if (pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL) {
			if (pti->pWaitingMutex->name) {
				save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
				printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
			} else {
				printk("pti->pWaitingMutex->name == NULL\r\n");
			}

			if (pti->pWaitingMutex->mutex_owner_asusdebug) {
				save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
				printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
			} else {
				printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");
			}

			if (pti->pWaitingMutex->mutex_owner_asusdebug->comm) {
				save_log(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
				printk(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
			} else {
				printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
			}
		}

		if (pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion != NULL) {
			if (pti->pWaitingCompletion->name)
				save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name);
			else
				printk("pti->pWaitingCompletion->name == NULL\r\n");
		}

		if (pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL) {
			struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
			if (temp)
				save_log("    RTMutex: Owned by pID(%d)", temp->pid);
			else
				printk("pti->pWaitingRTMutex->temp == NULL\r\n");
			if (temp->comm)
				save_log(" %s", temp->comm);
			else
				printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
		}

		save_log("\r\n");
		show_stack1(pts, NULL);

		save_log("\r\n");

		if (!thread_group_empty(pts)) {
			struct task_struct *p1 = next_thread(pts);
			do {
				pti = task_thread_info(p1);
				save_log(" %-7d", p1->pid);

				if (pts->parent)
					save_log("%-8d", p1->parent->pid);
				else
					save_log("%-8d", 0);

				save_log("%-20s", p1->comm);
				save_log("%lld.%06ld", SPLIT_NS(p1->se.sum_exec_runtime));
				save_log("     %lld.%06ld     ", SPLIT_NS(p1->se.vruntime));
				save_log("%-5d", p1->static_prio);
				save_log("%-5d", p1->normal_prio);
				save_log("%-15s", print_state((p1->state & TASK_REPORT) | p1->exit_state));
				save_log("%-6d", pti->preempt_count);

				if (pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL) {
					if (pti->pWaitingMutex->name) {
						save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
						printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
					} else {
						printk("pti->pWaitingMutex->name == NULL\r\n");
					}

					if (pti->pWaitingMutex->mutex_owner_asusdebug) {
						save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
						printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
					} else {
						printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");
					}

					if (pti->pWaitingMutex->mutex_owner_asusdebug->comm) {
						save_log(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
						printk(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
					} else {
						printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
					}
				}

				if (pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion != NULL) {
					if (pti->pWaitingCompletion->name)
						save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name);
					else
						printk("pti->pWaitingCompletion->name == NULL\r\n");
				}

				if (pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL) {
					struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
					if (temp)
						save_log("    RTMutex: Owned by pID(%d)", temp->pid);
					else
						printk("pti->pWaitingRTMutex->temp == NULL\r\n");
					if (temp->comm)
						save_log(" %s", temp->comm);
					else
						printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
				}
				save_log("\r\n");
				show_stack1(p1, NULL);

				save_log("\r\n");
				p1 = next_thread(p1);
			} while (p1 != pts);
		}
		save_log("-----------------------------------------------------\r\n\r\n\r\n");
	}
	save_log("\r\n\r\n\r\n\r\n");
}

struct thread_info_save *ptis_head = NULL;
int find_thread_info(struct task_struct *pts, int force)
{
	struct thread_info *pti;
	struct thread_info_save *ptis, *ptis_ptr;
	u64 vruntime = 0, sum_exec_runtime;

	if (ptis_head != NULL) {
		ptis = ptis_head->pnext;
		ptis_ptr = NULL;
		while (ptis) {
			if (ptis->pid == pts->pid && ptis->pts == pts) {
				ptis_ptr = ptis;
				break;
			}
			ptis = ptis->pnext;
		}

		if (ptis_ptr)
			sum_exec_runtime = pts->se.sum_exec_runtime - ptis->sum_exec_runtime;
		else
			sum_exec_runtime = pts->se.sum_exec_runtime;

		if (sum_exec_runtime > 0 || force) {
			pti = task_thread_info(pts);
			save_log(" %-7d", pts->pid);

			if (pts->parent)
				save_log("%-8d", pts->parent->pid);
			else
				save_log("%-8d", 0);

			save_log("%-20s", pts->comm);
			save_log("%lld.%06ld", SPLIT_NS(sum_exec_runtime));
			if (nsec_high(sum_exec_runtime) > 1000)
				save_log(" ******");
			save_log("     %lld.%06ld     ", SPLIT_NS(vruntime));
			save_log("%-5d", pts->static_prio);
			save_log("%-5d", pts->normal_prio);
			save_log("%-15s", print_state(pts->state));
			save_log("%-6d", pti->preempt_count);

			if (pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL) {
				if (pti->pWaitingMutex->name) {
					save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
					printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
				} else {
					printk("pti->pWaitingMutex->name == NULL\r\n");
				}

				if (pti->pWaitingMutex->mutex_owner_asusdebug) {
					save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
					printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
				} else {
					printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");
				}

				if (pti->pWaitingMutex->mutex_owner_asusdebug->comm) {
					save_log(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
					printk(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
				} else {
					printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
				}
			}

			if (pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion != NULL) {
				if (pti->pWaitingCompletion->name)
					save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name);
				else
					printk("pti->pWaitingCompletion->name == NULL\r\n");
			}

			if (pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL) {
				struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
				if (temp)
					save_log("    RTMutex: Owned by pID(%d)", temp->pid);
				else
					printk("pti->pWaitingRTMutex->temp == NULL\r\n");
				if (temp->comm)
					save_log(" %s", temp->comm);
				else
					printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
			}

			save_log("\r\n");

			show_stack1(pts, NULL);
			save_log("\r\n");
		} else {
			return 0;
		}
	}
	return 1;
}

void save_all_thread_info(void)
{
	struct task_struct *pts;
	struct thread_info *pti;
	struct thread_info_save *ptis = NULL, *ptis_ptr = NULL;

	struct rtc_time tm;

	asus_rtc_read_time(&tm);

	g_phonehang_log = (char *)PHONE_HANG_LOG_BUFFER;
	g_iPtr = 0;
	memset_nc(g_phonehang_log, 0, PHONE_HANG_LOG_SIZE);

	save_log("ASUSSlowg-%04d%02d%02d-%02d%02d%02d.txt  ---  ASUS_SW_VER : %s----------------------------------------------\r\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ASUS_SW_VER);
	save_log(" pID----ppID----NAME----------------SumTime---vruntime--SPri-NPri-State----------PmpCnt-binder----Waiting\r\n");


	if (ptis_head != NULL) {
		struct thread_info_save *ptis_next = ptis_head->pnext;
		struct thread_info_save *ptis_next_next;
		while (ptis_next) {
			ptis_next_next = ptis_next->pnext;
			kfree(ptis_next);
			ptis_next = ptis_next_next;
		}
		kfree(ptis_head);
		ptis_head = NULL;
	}

	if (ptis_head == NULL) {
		ptis_ptr = ptis_head = kmalloc(sizeof(struct thread_info_save), GFP_KERNEL);
		if (!ptis_head) {
			printk("kmalloc ptis_head failure\n");
			return;
		}
		memset(ptis_head, 0, sizeof(struct thread_info_save));
	}

	for_each_process(pts){
		pti = task_thread_info(pts);

		ptis = kmalloc(sizeof(struct thread_info_save), GFP_KERNEL);
		if (!ptis) {
			printk("kmalloc ptis failure\n");
			return;
		}
		memset(ptis, 0, sizeof(struct thread_info_save));

		save_log("-----------------------------------------------------\r\n");
		save_log(" %-7d", pts->pid);
		if (pts->parent)
			save_log("%-8d", pts->parent->pid);
		else
			save_log("%-8d", 0);

		save_log("%-20s", pts->comm);
		save_log("%lld.%06ld", SPLIT_NS(pts->se.sum_exec_runtime));
		save_log("     %lld.%06ld     ", SPLIT_NS(pts->se.vruntime));
		save_log("%-5d", pts->static_prio);
		save_log("%-5d", pts->normal_prio);
		save_log("%-15s", print_state((pts->state & TASK_REPORT) | pts->exit_state));
		save_log("%-6d", pti->preempt_count);

		if (pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL) {
			if (pti->pWaitingMutex->name) {
				save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
				printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
			} else {
				printk("pti->pWaitingMutex->name == NULL\r\n");
			}

			if (pti->pWaitingMutex->mutex_owner_asusdebug) {
				save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
				printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
			} else {
				printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");
			}

			if (pti->pWaitingMutex->mutex_owner_asusdebug->comm) {
				save_log(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
				printk(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
			} else {
				printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
			}
		}
		if (pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion != NULL) {
			if (pti->pWaitingCompletion->name)
				save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name);
			else
				printk("pti->pWaitingCompletion->name == NULL\r\n");
		}

		if (pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL) {
			struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
			if (temp)
				save_log("    RTMutex: Owned by pID(%d)", temp->pid);
			else
				printk("pti->pWaitingRTMutex->temp == NULL\r\n");
			if (temp->comm)
				save_log(" %s", temp->comm);
			else
				printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
		}


		save_log("\r\n");
		show_stack1(pts, NULL);

		save_log("\r\n");


		ptis->pid = pts->pid;
		ptis->pts = pts;
		ptis->sum_exec_runtime = pts->se.sum_exec_runtime;
		ptis->vruntime = pts->se.vruntime;

		ptis_ptr->pnext = ptis;
		ptis_ptr = ptis;

		if (!thread_group_empty(pts)) {
			struct task_struct *p1 = next_thread(pts);
			do {
				pti = task_thread_info(p1);

				ptis = kmalloc(sizeof(struct thread_info_save), GFP_KERNEL);
				if (!ptis) {
					printk("kmalloc ptis 2 failure\n");
					return;
				}
				memset(ptis, 0, sizeof(struct thread_info_save));

				ptis->pid = p1->pid;
				ptis->pts = p1;
				ptis->sum_exec_runtime = p1->se.sum_exec_runtime;
				ptis->vruntime = p1->se.vruntime;

				ptis_ptr->pnext = ptis;
				ptis_ptr = ptis;
				save_log(" %-7d", p1->pid);

				if (pts->parent)
					save_log("%-8d", p1->parent->pid);
				else
					save_log("%-8d", 0);

				save_log("%-20s", p1->comm);
				save_log("%lld.%06ld", SPLIT_NS(p1->se.sum_exec_runtime));
				save_log("     %lld.%06ld     ", SPLIT_NS(p1->se.vruntime));
				save_log("%-5d", p1->static_prio);
				save_log("%-5d", p1->normal_prio);
				save_log("%-15s", print_state((p1->state & TASK_REPORT) | p1->exit_state));
				save_log("%-6d", pti->preempt_count);

				if (pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL) {
					if (pti->pWaitingMutex->name) {
						save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
						printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
					} else {
						printk("pti->pWaitingMutex->name == NULL\r\n");
					}

					if (pti->pWaitingMutex->mutex_owner_asusdebug) {
						save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
						printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
					} else {
						printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");
					}

					if (pti->pWaitingMutex->mutex_owner_asusdebug->comm) {
						save_log(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
						printk(" %s", pti->pWaitingMutex->mutex_owner_asusdebug->comm);
					} else {
						printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
					}
				}

				if (pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion != NULL) {
					if (pti->pWaitingCompletion->name)
						save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name);
					else
						printk("pti->pWaitingCompletion->name == NULL\r\n");
				}

				if (pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL) {
					struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
					if (temp)
						save_log("    RTMutex: Owned by pID(%d)", temp->pid);
					else
						printk("pti->pWaitingRTMutex->temp == NULL\r\n");
					if (temp->comm)
						save_log(" %s", temp->comm);
					else
						printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
				}
				save_log("\r\n");
				show_stack1(p1, NULL);

				save_log("\r\n");

				p1 = next_thread(p1);
			} while (p1 != pts);
		}
	}
}

EXPORT_SYMBOL(save_all_thread_info);

void delta_all_thread_info(void)
{
	struct task_struct *pts;
	int ret = 0, ret2 = 0;

	struct rtc_time tm;

	asus_rtc_read_time(&tm);
	g_phonehang_log = (char *)PHONE_HANG_LOG_BUFFER;
	g_iPtr = 0;
	memset_nc(g_phonehang_log, 0, PHONE_HANG_LOG_SIZE);

	save_log("ASUSSlowg-%04d%02d%02d-%02d%02d%02d-delta.txt  ---  ASUS_SW_VER : %s----------------------------------------------\r\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ASUS_SW_VER);

	save_log("DELTA INFO----------------------------------------------------------------------------------------------\r\n");
	save_log(" pID----ppID----NAME----------------SumTime---vruntime--SPri-NPri-State----------PmpCnt----Waiting\r\n");
	for_each_process(pts){
		ret = find_thread_info(pts, 0);
		if (!thread_group_empty(pts)) {
			struct task_struct *p1 = next_thread(pts);
			ret2 = 0;
			do {
				ret2 += find_thread_info(p1, 0);
				p1 = next_thread(p1);
			} while (p1 != pts);
			if (ret2 && !ret)
				find_thread_info(pts, 1);
		}
		if (ret || ret2)
			save_log("-----------------------------------------------------\r\n\r\n-----------------------------------------------------\r\n");
	}
	save_log("\r\n\r\n\r\n\r\n");
}
EXPORT_SYMBOL(delta_all_thread_info);
///////////////////////////////////////////////////////////////////////////////////////////////////
void printk_buffer_rebase(void);
static mm_segment_t oldfs;
static void initKernelEnv(void)
{
	oldfs = get_fs();
	set_fs(KERNEL_DS);
}

static void deinitKernelEnv(void)
{
	set_fs(oldfs);
}

char messages[256];
void save_phone_hang_log(int delta)
{
	int file_handle;
	int ret;

	//---------------saving phone hang log if any -------------------------------
	g_phonehang_log = (char *)PHONE_HANG_LOG_BUFFER;
	printk("save_phone_hang_log PRINTK_BUFFER=%p, PHONE_HANG_LOG_BUFFER=%p\n", PRINTK_BUFFER_VA, PHONE_HANG_LOG_BUFFER);
	if (g_phonehang_log && ((strncmp(g_phonehang_log, "PhoneHang", 9) == 0) || (strncmp(g_phonehang_log, "ASUSSlowg", 9) == 0))) {
		printk("save_phone_hang_log-1\n");
		initKernelEnv();
		memset(messages, 0, sizeof(messages));
		strcpy(messages, ASUS_ASDF_BASE_DIR);
		if (delta)
			strncat(messages, g_phonehang_log, 29 + 6);
		else
			strncat(messages, g_phonehang_log, 29);
		file_handle = sys_open(messages, O_CREAT | O_WRONLY | O_SYNC, 0);
		printk("save_phone_hang_log-2 file_handle %d, name=%s\n", file_handle, messages);
		if (!IS_ERR((const void *)(ulong)file_handle)) {
			ret = sys_write(file_handle, (unsigned char *)g_phonehang_log, strlen(g_phonehang_log));
			sys_close(file_handle);
		}
		deinitKernelEnv();
	}
	if (g_phonehang_log && file_handle > 0 && ret > 0)
		g_phonehang_log[0] = 0;
}
EXPORT_SYMBOL(save_phone_hang_log);
void save_last_shutdown_log(char *filename)
{
	char *last_shutdown_log;
	int file_handle;
	unsigned long long t;
	unsigned long nanosec_rem;
    // ASUS_BSP +++
    char buffer[] = {"Kernel panic"};
    int i;
    // ASUS_BSP ---

	t = cpu_clock(0);
	nanosec_rem = do_div(t, 1000000000);
	last_shutdown_log = (char *)PRINTK_BUFFER_VA;
	sprintf(messages, ASUS_ASDF_BASE_DIR "LastShutdown_%lu.%06lu.txt",
		(unsigned long)t,
		nanosec_rem / 1000);

	initKernelEnv();
	file_handle = sys_open(messages, O_CREAT | O_RDWR | O_SYNC, 0);
	if (!IS_ERR((const void *)(ulong)file_handle)) {
		sys_write(file_handle, (unsigned char *)last_shutdown_log, PRINTK_BUFFER_SLOT_SIZE);
		sys_close(file_handle);
        // ASUS_BSP +++
        for(i=0; i<PRINTK_BUFFER_SLOT_SIZE; i++) {
            //
            // Check if it is kernel panic
            //
            if (strncmp((last_shutdown_log + i), buffer, strlen(buffer)) == 0) {
                ASUSEvtlog("[Reboot] Kernel panic\n");
                break;
            }
        }
        // ASUS_BSP ---
	} else {
		printk("[ASDF] save_last_shutdown_error: [%d]\n", file_handle);
	}
	deinitKernelEnv();
}

#if defined(CONFIG_QCOM_RTB)
extern struct msm_rtb_state msm_rtb;

int g_saving_rtb_log = 1;

void save_rtb_log(void)
{
	char *rtb_log;
	char rtb_log_path[256] = { 0 };
	int file_handle;
	unsigned long long t;
	unsigned long nanosec_rem;

	rtb_log = (char *)msm_rtb.rtb;
	t = cpu_clock(0);
	nanosec_rem = do_div(t, 1000000000);
	snprintf(rtb_log_path, sizeof(rtb_log_path) - 1, ASUS_ASDF_BASE_DIR "/rtb_%lu.%06lu.bin",
		 (unsigned long)t,
		 nanosec_rem / 1000);

	initKernelEnv();
	file_handle = sys_open(rtb_log_path, O_CREAT | O_RDWR | O_SYNC, 0);
	if (!IS_ERR((const void *)(ulong)file_handle)) {
		sys_write(file_handle, (unsigned char *)rtb_log, msm_rtb.size);
		sys_close(file_handle);
	} else {
		printk("[ASDF] save_rtb_log_error: [%d]\n", file_handle);
	}
	deinitKernelEnv();
}
#endif

void get_last_shutdown_log(void)
{
	ulong *printk_buffer_slot2_addr;

	printk_buffer_slot2_addr = (ulong *)PRINTK_BUFFER_SLOT2;
	printk("get_last_shutdown_log: printk_buffer_slot2=%p, value=0x%lx\n", printk_buffer_slot2_addr, *printk_buffer_slot2_addr);
	if (*printk_buffer_slot2_addr == (ulong)PRINTK_BUFFER_MAGIC)
		save_last_shutdown_log("LastShutdown");
	printk_buffer_rebase();
}
EXPORT_SYMBOL(get_last_shutdown_log);

extern int nSuspendInProgress;
static struct workqueue_struct *ASUSEvtlog_workQueue;
static int g_hfileEvtlog = -MAX_ERRNO;
static int g_bEventlogEnable = 1;
static char g_Asus_Eventlog[ASUS_EVTLOG_MAX_ITEM][ASUS_EVTLOG_STR_MAXLEN];
static int g_Asus_Eventlog_read = 0;
static int g_Asus_Eventlog_write = 0;
//[+++]Record the important power event
static struct workqueue_struct *ASUSErclog_workQueue;
static int g_hfileErclog = -MAX_ERRNO;
static char g_Asus_Erclog[ASUS_ERCLOG_MAX_ITEM][ASUS_ERCLOG_STR_MAXLEN];
static char g_Asus_Erclog_filelist[ASUS_ERCLOG_MAX_ITEM][ASUS_ERCLOG_FILENAME_MAXLEN];
static int g_Asus_Erclog_read = 0;
static int g_Asus_Erclog_write = 0;
//[---]Record the important power event

static void do_write_event_worker(struct work_struct *work);
static DECLARE_WORK(eventLog_Work, do_write_event_worker);
//[+++]Record the important power event
static void do_write_erc_worker(struct work_struct *work);
static DECLARE_WORK(ercLog_Work, do_write_erc_worker);
//[---]Record the important power event

/*ASUS-BBSP SubSys Health Record+++*/
static char g_SubSys_W_Buf[SUBSYS_W_MAXLEN];
static char g_SubSys_C_Buf[SUBSYS_C_MAXLEN]="0000-0000-0000-0000-0000";
static void do_write_subsys_worker(struct work_struct *work);
static void do_count_subsys_worker(struct work_struct *work);
static void do_delete_subsys_worker(struct work_struct *work);
static DECLARE_WORK(subsys_w_Work, do_write_subsys_worker);
static DECLARE_WORK(subsys_c_Work, do_count_subsys_worker);
static DECLARE_WORK(subsys_d_Work, do_delete_subsys_worker);
static struct completion SubSys_C_Complete;
/*ASUS-BBSP SubSys Health Record---*/

static struct mutex mA;
static struct mutex mA_erc;//Record the important power event
#define AID_SDCARD_RW 1015
static void do_write_event_worker(struct work_struct *work)
{
	char buffer[256];

	memset(buffer, 0, sizeof(char) * 256);

	if (IS_ERR((const void *)(ulong)g_hfileEvtlog)) {
		long size;

		g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH ".txt", O_CREAT | O_RDWR | O_SYNC, 0666);
		sys_chown(ASUS_EVTLOG_PATH ".txt", AID_SDCARD_RW, AID_SDCARD_RW);

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_8M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH "_old.txt");
			sys_rename(ASUS_EVTLOG_PATH ".txt", ASUS_EVTLOG_PATH "_old.txt");
			g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH ".txt", O_CREAT | O_RDWR | O_SYNC, 0666);
		}
        // ASUS_BSP +++
        if (warm_reset_value) {
    		snprintf(buffer, sizeof(buffer),
                "\n\n---------------System Boot----%s---------\n"
                "[Reboot] Warm reset Reason: %s ###### \n"
                "###### Bootup Reason: %s ######\n",
                ASUS_SW_VER,
                evtlog_warm_reset_reason,
                evtlog_bootup_reason);

        } else {
    		snprintf(buffer, sizeof(buffer),
                "\n\n---------------System Boot----%s---------\n"
                "[Shutdown] Power off Reason: %s ###### \n"
                "###### Bootup Reason: %s ######\n",
                ASUS_SW_VER,
                evtlog_poweroff_reason,
                evtlog_bootup_reason);
        }
        // ASUS_BSP ---
		sys_write(g_hfileEvtlog, buffer, strlen(buffer));
		sys_close(g_hfileEvtlog);
	}
	if (!IS_ERR((const void *)(ulong)g_hfileEvtlog)) {
		int str_len;
		char *pchar;
		long size;

		g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH ".txt", O_CREAT | O_RDWR | O_SYNC, 0666);
		sys_chown(ASUS_EVTLOG_PATH ".txt", AID_SDCARD_RW, AID_SDCARD_RW);

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_8M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH "_old.txt");
			sys_rename(ASUS_EVTLOG_PATH ".txt", ASUS_EVTLOG_PATH "_old.txt");
			g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH ".txt", O_CREAT | O_RDWR | O_SYNC, 0666);
		}

		while (g_Asus_Eventlog_read != g_Asus_Eventlog_write) {
			mutex_lock(&mA);
			str_len = strlen(g_Asus_Eventlog[g_Asus_Eventlog_read]);
			pchar = g_Asus_Eventlog[g_Asus_Eventlog_read];
			g_Asus_Eventlog_read++;
			g_Asus_Eventlog_read %= ASUS_EVTLOG_MAX_ITEM;
			mutex_unlock(&mA);

			if (pchar[str_len - 1] != '\n') {
				if (str_len + 1 >= ASUS_EVTLOG_STR_MAXLEN)
					str_len = ASUS_EVTLOG_STR_MAXLEN - 2;
				pchar[str_len] = '\n';
				pchar[str_len + 1] = '\0';
			}

			sys_write(g_hfileEvtlog, pchar, strlen(pchar));
			sys_fsync(g_hfileEvtlog);
		}
		sys_close(g_hfileEvtlog);
	}
}

extern struct timezone sys_tz;

void ASUSEvtlog(const char *fmt, ...)
{
	va_list args;
	char *buffer;

	if (g_bEventlogEnable == 0)
		return;
	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_lock(&mA);

	buffer = g_Asus_Eventlog[g_Asus_Eventlog_write];
	g_Asus_Eventlog_write++;
	g_Asus_Eventlog_write %= ASUS_EVTLOG_MAX_ITEM;

	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_unlock(&mA);

	memset(buffer, 0, ASUS_EVTLOG_STR_MAXLEN);
	if (buffer) {
		struct rtc_time tm;
		struct timespec ts;

		getnstimeofday(&ts);
		ts.tv_sec -= sys_tz.tz_minuteswest * 60;
		rtc_time_to_tm(ts.tv_sec, &tm);
		getrawmonotonic(&ts);
		sprintf(buffer, "(%ld)%04d-%02d-%02d %02d:%02d:%02d :", ts.tv_sec, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		va_start(args, fmt);
		vscnprintf(buffer + strlen(buffer), ASUS_EVTLOG_STR_MAXLEN - strlen(buffer), fmt, args);
		va_end(args);
		printk("%s", buffer);
		queue_work(ASUSEvtlog_workQueue, &eventLog_Work);
	} else {
		printk("ASUSEvtlog buffer cannot be allocated\n");
	}
}
EXPORT_SYMBOL(ASUSEvtlog);

//[+++]Record the important power event
static void do_write_erc_worker(struct work_struct *work)
{
    int str_len;
    char log_body[ASUS_ERCLOG_STR_MAXLEN];
    char filepath[ASUS_ERCLOG_FILENAME_MAXLEN];
    char filepath_old[ASUS_ERCLOG_FILENAME_MAXLEN];
    long size;
    int flag_read = -1;
    int flag_write = -1;

    while (g_Asus_Erclog_read != g_Asus_Erclog_write) {
        memset(log_body, 0, ASUS_ERCLOG_STR_MAXLEN);
        memset(filepath, 0, sizeof(char)*ASUS_ERCLOG_FILENAME_MAXLEN);
        memset(filepath_old, 0, sizeof(char)*ASUS_ERCLOG_FILENAME_MAXLEN);

        mutex_lock(&mA_erc);
        flag_read = g_Asus_Erclog_read;
        flag_write = g_Asus_Erclog_write;

        memcpy(log_body, g_Asus_Erclog[g_Asus_Erclog_read], ASUS_ERCLOG_STR_MAXLEN);
        snprintf(filepath, ASUS_ERCLOG_FILENAME_MAXLEN, "%s%s.txt", ASUS_ASDF_BASE_DIR, g_Asus_Erclog_filelist[g_Asus_Erclog_read]);
        snprintf(filepath_old, ASUS_ERCLOG_FILENAME_MAXLEN, "%s%s_old.txt", ASUS_ASDF_BASE_DIR, g_Asus_Erclog_filelist[g_Asus_Erclog_read]);
        memset(g_Asus_Erclog[g_Asus_Erclog_read], 0, ASUS_ERCLOG_STR_MAXLEN);
        memset(g_Asus_Erclog_filelist[g_Asus_Erclog_read], 0, ASUS_ERCLOG_FILENAME_MAXLEN);

        g_Asus_Erclog_read++;
        g_Asus_Erclog_read %= ASUS_ERCLOG_MAX_ITEM;
        mutex_unlock(&mA_erc);

        str_len = strlen(log_body);
		if(str_len == 0) continue;
        if (str_len > 0 && log_body[str_len - 1] != '\n' ) {
            if(str_len + 1 >= ASUS_ERCLOG_STR_MAXLEN)
                str_len = ASUS_ERCLOG_STR_MAXLEN - 2;
            log_body[str_len] = '\n';
            log_body[str_len + 1] = '\0';
        }

        pr_debug("flag_read = %d, flag_write = %d, filepath = %s\n", flag_read, flag_write, filepath);
        g_hfileErclog = sys_open(filepath, O_CREAT|O_RDWR|O_SYNC, 0666);
        sys_chown(filepath, AID_SDCARD_RW, AID_SDCARD_RW);

        if (!IS_ERR((const void *)(ulong)g_hfileEvtlog)) {
            size = sys_lseek(g_hfileErclog, 0, SEEK_END);
            if (size >= 5000) {    //limit 5KB each file
                sys_close(g_hfileErclog);
                sys_rmdir(filepath_old);
                sys_rename(filepath, filepath_old);
                g_hfileErclog = sys_open(filepath, O_CREAT|O_RDWR|O_SYNC, 0666);
            }

            sys_write(g_hfileErclog, log_body, strlen(log_body));
            sys_fsync(g_hfileErclog);
            sys_close(g_hfileErclog);

        }else{
            pr_err("sys_open %s IS_ERR error code: %d]\n", filepath, g_hfileEvtlog);
        }
    }
}

void ASUSErclog(const char * filename, const char *fmt, ...)
{
    va_list args;
    char *buffer;
    char *tofile;
    int flag_write = -1;

//    if (!in_interrupt() && !in_atomic() && !irqs_disabled())
        mutex_lock(&mA_erc);

    flag_write = g_Asus_Erclog_write;
    buffer = g_Asus_Erclog[g_Asus_Erclog_write];
    tofile = g_Asus_Erclog_filelist[g_Asus_Erclog_write];
    memset(buffer, 0, ASUS_EVTLOG_STR_MAXLEN);
    memset(tofile, 0, ASUS_ERCLOG_FILENAME_MAXLEN);
    g_Asus_Erclog_write++;
    g_Asus_Erclog_write %= ASUS_EVTLOG_MAX_ITEM;

//    if (!in_interrupt() && !in_atomic() && !irqs_disabled())
        mutex_unlock(&mA_erc);

    if (buffer) {
        struct rtc_time tm;
        struct timespec ts;

        getnstimeofday(&ts);
        ts.tv_sec -= sys_tz.tz_minuteswest * 60;
        rtc_time_to_tm(ts.tv_sec, &tm);
        getrawmonotonic(&ts);
        sprintf(buffer, "%04d%02d%02d%02d%02d%02d :", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

        va_start(args, fmt);
        vscnprintf(buffer + strlen(buffer), ASUS_ERCLOG_STR_MAXLEN - strlen(buffer), fmt, args);
        va_end(args);
		printk("%s", buffer);
        sprintf(tofile, "%s", filename);
        pr_debug("flag_write= %d, tofile = %s\n", flag_write, tofile);
        queue_work(ASUSErclog_workQueue, &ercLog_Work);
    } else {
        pr_err("[ASDF]ASUSErclog buffer cannot be allocated\n");
    }
}
EXPORT_SYMBOL(ASUSErclog);
//[---]Record the important power event
/*ASUS-BBSP SubSys Health Record+++*/
static void do_write_subsys_worker(struct work_struct *work)
{
	int hfile = -MAX_ERRNO;
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	hfile = sys_open(SUBSYS_HEALTH_MEDICAL_TABLE_PATH".txt", O_CREAT|O_WRONLY|O_SYNC, 0666);
	if(!IS_ERR((const void *)(ulong)hfile)) {
		if (sys_lseek(hfile, 0, SEEK_END) >= SZ_128K) {
			ASUSEvtlog("[SSR-Info] SubSys is versy ill\n");
			sys_close(hfile);
			sys_unlink(SUBSYS_HEALTH_MEDICAL_TABLE_PATH"_old.txt");
			sys_rename(SUBSYS_HEALTH_MEDICAL_TABLE_PATH".txt", SUBSYS_HEALTH_MEDICAL_TABLE_PATH"_old.txt");
			hfile = sys_open(SUBSYS_HEALTH_MEDICAL_TABLE_PATH".txt", O_CREAT|O_RDWR|O_SYNC, 0666);
		}
		sys_write(hfile, g_SubSys_W_Buf, strlen(g_SubSys_W_Buf));
		sys_fsync(hfile);
		sys_close(hfile);
	} else {
		ASUSEvtlog("[SSR-Info] Save SubSys Medical Table Error: [0x%x]\n", hfile);
		sys_unlink(SUBSYS_HEALTH_MEDICAL_TABLE_PATH".txt");/*Delete The File Which is Opened in Address Space Mismatch*/
	}
	set_fs(old_fs);	
}

static void do_count_subsys_worker(struct work_struct *work)
{
	int  hfile = -MAX_ERRNO;
	char r_buf[SUBSYS_R_MAXLEN];
	int  r_size = 0;
	int  index = 0;
	char keys[] = "[SSR]:";
	char *pch;
	char SubSysName[SUBSYS_NUM][10] = { "modem", "wcnss", "adsp", "venus", "a506_zap" };
	int  Counts[SUBSYS_NUM] = { 0 };/* MODEM, WCNSS, ADSP, VENUS, A506_ZAP */
	int  reboot_count = 0;
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	hfile = sys_open(SUBSYS_HEALTH_MEDICAL_TABLE_PATH".txt", O_CREAT|O_RDONLY|O_SYNC, 0444);
	if(!IS_ERR((const void *)(ulong)hfile)) {
		do {
			memset(r_buf, 0, sizeof(r_buf));
			r_size = sys_read(hfile, r_buf, sizeof(r_buf));
			if (r_size != 0) {
				/* count */
				pch = strstr(r_buf, keys);
				while (pch != NULL) {
					pch = pch + strlen(keys);
					for (index = 0 ; index < SUBSYS_NUM ; index++) {
						if (!strncmp(pch, SubSysName[index], strlen(SubSysName[index]))) {
							Counts[index]++;
							break;
						}
					}
					pch = strstr(pch, keys);
				}
			}
		} while (r_size != 0);

		sys_close(hfile);
	}

	hfile = sys_open(SUBSYS_HEALTH_MEDICAL_TABLE_PATH"_old.txt", O_RDONLY|O_SYNC, 0444);
	if(!IS_ERR((const void *)(ulong)hfile)) {
		do {
			memset(r_buf, 0, sizeof(r_buf));
			r_size = sys_read(hfile, r_buf, sizeof(r_buf));
			if (r_size != 0) {
				/* count */
				pch = strstr(r_buf, keys);
				while (pch != NULL) {
					pch = pch + strlen(keys);
					for (index = 0 ; index < SUBSYS_NUM ; index++) {
						if (!strncmp(pch, SubSysName[index], strlen(SubSysName[index]))) {
							Counts[index]++;
							break;
						}
					}
					pch = strstr(pch, keys);
				}
			}
		} while (r_size != 0);

		sys_close(hfile);
	}

	set_fs(old_fs);
	sprintf(g_SubSys_C_Buf, "%d-%d-%d-%d-%d", reboot_count, Counts[0], Counts[1], Counts[2], Counts[3]+Counts[4]);/* MODEM, WCNSS, ADSP, OTHERS(VENUS+A506_ZAP) */
	complete(&SubSys_C_Complete);
}

static void do_delete_subsys_worker(struct work_struct *work)
{
	sys_unlink(SUBSYS_HEALTH_MEDICAL_TABLE_PATH".txt");
	sys_unlink(SUBSYS_HEALTH_MEDICAL_TABLE_PATH"_old.txt");
	ASUSEvtlog("[SSR-Info] SubSys Medical Table Deleted\n");
}

void SubSysHealthRecord(const char *fmt, ...)
{
	va_list args;
	char *w_buf;
	struct rtc_time tm;
	struct timespec ts;

	memset(g_SubSys_W_Buf, 0 , sizeof(g_SubSys_W_Buf));
	w_buf = g_SubSys_W_Buf;

	getnstimeofday(&ts);
	ts.tv_sec -= sys_tz.tz_minuteswest * 60;
	rtc_time_to_tm(ts.tv_sec, &tm);
	sprintf(w_buf, "%04d-%02d-%02d %02d:%02d:%02d : ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	va_start(args, fmt);
	vscnprintf(w_buf + strlen(w_buf), sizeof(g_SubSys_W_Buf) - strlen(w_buf), fmt, args);
	va_end(args);
	/*printk("g_SubSys_W_Buf = %s", g_SubSys_W_Buf);*/

	queue_work(ASUSEvtlog_workQueue, &subsys_w_Work);
}
EXPORT_SYMBOL(SubSysHealthRecord);

static int SubSysHealth_proc_show(struct seq_file *m, void *v)
{
	unsigned long ret;

	queue_work(ASUSEvtlog_workQueue, &subsys_c_Work);/* Issue to count */

	ret = wait_for_completion_timeout(&SubSys_C_Complete, msecs_to_jiffies(1000));
	if (!ret)
		ASUSEvtlog("[SSR-Info] Timed out on query SubSys count\n");

	seq_printf(m, "%s\n", g_SubSys_C_Buf);
	return 0;
}

static int SubSysHealth_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, SubSysHealth_proc_show, NULL);
}

static ssize_t SubSysHealth_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char keyword[] = "clear";
	char tmpword[10];
	memset(tmpword, 0, sizeof(tmpword));

	/* no data be written Or Input size is too large to write our buffer */
	if ((!count) || (count > (sizeof(tmpword) - 1)))
		return -EINVAL;

	if (copy_from_user(tmpword, buf, count))
		return -EFAULT;

	if (strncmp(tmpword, keyword, strlen(keyword)) == 0) {
		queue_work(ASUSEvtlog_workQueue, &subsys_d_Work);
	}

	return count;
}

static const struct file_operations proc_SubSysHealth_operations = {
	.open = SubSysHealth_proc_open,
	.read = seq_read,
	.write = SubSysHealth_proc_write,
};
/*ASUS-BBSP SubSys Health Record---*/

static ssize_t evtlogswitch_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if (strncmp(buf, "0", 1) == 0) {
		ASUSEvtlog("ASUSEvtlog disable !!");
		printk("ASUSEvtlog disable !!\n");
		flush_work(&eventLog_Work);
		g_bEventlogEnable = 0;
	}
	if (strncmp(buf, "1", 1) == 0) {
		g_bEventlogEnable = 1;
		ASUSEvtlog("ASUSEvtlog enable !!");
		printk("ASUSEvtlog enable !!\n");
	}

	return count;
}
static ssize_t asusevtlog_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if (count > 256)
		count = 256;

	memset(messages, 0, sizeof(messages));
	if (copy_from_user(messages, buf, count))
		return -EFAULT;

	ASUSEvtlog("%s", messages);

	return count;
}

static int asusdebug_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int asusdebug_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t asusdebug_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	return 0;
}

#include <linux/reboot.h>
extern int rtc_ready;
int asus_asdf_set = 0;
static ssize_t asusdebug_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	u8 messages[256] = { 0 };

	if (count > 256)
		count = 256;
	if (copy_from_user(messages, buf, count))
		return -EFAULT;

	if (strncmp(messages, "panic", strlen("panic")) == 0) {
		panic("panic test");
	} else if (strncmp(messages, "dbg", strlen("dbg")) == 0) {
		g_user_dbg_mode = 1;
		printk("Kernel dbg mode = %d\n", g_user_dbg_mode);
	} else if (strncmp(messages, "ndbg", strlen("ndbg")) == 0) {
		g_user_dbg_mode = 0;
		printk("Kernel dbg mode = %d\n", g_user_dbg_mode);
	} else if (strncmp(messages, "get_asdf_log",
			   strlen("get_asdf_log")) == 0) {
#ifdef CONFIG_QCOM_RTB
		extern int g_saving_rtb_log;
#endif
		ulong *printk_buffer_slot2_addr;

		printk_buffer_slot2_addr = (ulong *)PRINTK_BUFFER_SLOT2;
		printk("[ASDF] printk_buffer_slot2_addr=%p, value=0x%lx\n", printk_buffer_slot2_addr, *printk_buffer_slot2_addr);

		if (!asus_asdf_set) {
			asus_asdf_set = 1;
			save_phone_hang_log(1);
			get_last_shutdown_log();
			printk("[ASDF] get_last_shutdown_log: printk_buffer_slot2_addr=%p, value=0x%lx\n", printk_buffer_slot2_addr, *printk_buffer_slot2_addr);
#ifdef CONFIG_QCOM_RTB
			if ((*printk_buffer_slot2_addr) == (ulong)PRINTK_BUFFER_MAGIC)
				save_rtb_log();
#endif
			if ((*printk_buffer_slot2_addr) == (ulong)PRINTK_BUFFER_MAGIC) {
				printk("[ASDF] after saving asdf log, then reboot\n");
				sys_sync();
				kernel_restart(NULL);
			}

			(*printk_buffer_slot2_addr) = (ulong)PRINTK_BUFFER_MAGIC;
		}
#ifdef CONFIG_QCOM_RTB
		g_saving_rtb_log = 0;
#endif
	} else if (strncmp(messages, "slowlog", strlen("slowlog")) == 0) {
		printk("start to gi chk\n");
		save_all_thread_info();
		save_phone_hang_log(0);
		msleep(5 * 1000);

		printk("start to gi delta\n");
		delta_all_thread_info();
		save_phone_hang_log(1);
		return count;
	}

	return count;
}

static const struct file_operations proc_evtlogswitch_operations = {
	.write	= evtlogswitch_write,
};
static const struct file_operations proc_asusevtlog_operations = {
	.write	= asusevtlog_write,
};
static const struct file_operations proc_asusdebug_operations = {
	.read		= asusdebug_read,
	.write		= asusdebug_write,
	.open		= asusdebug_open,
	.release	= asusdebug_release,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void asusdebug_early_suspend(struct early_suspend *h)
{
	entering_suspend = 1;
}

static void asusdebug_early_resume(struct early_suspend *h)
{
	entering_suspend = 0;
}
EXPORT_SYMBOL(entering_suspend);

struct early_suspend asusdebug_early_suspend_handler = {
	.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend	= asusdebug_early_suspend,
	.resume		= asusdebug_early_resume,
};
#endif

unsigned int asusdebug_enable = 0;
unsigned int readflag = 0;
static ssize_t turnon_asusdebug_proc_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
	char print_buf[32];
	unsigned int ret = 0, iret = 0;

	sprintf(print_buf, "asusdebug: %s\n", asusdebug_enable ? "off" : "on");
	ret = strlen(print_buf);
	iret = copy_to_user(buff, print_buf, ret);
	if (!readflag) {
		readflag = 1;
		return ret;
	} else {
		readflag = 0;
		return 0;
	}
}
static ssize_t turnon_asusdebug_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
	char messages[256];

	memset(messages, 0, sizeof(messages));
	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	if (strncmp(messages, "off", 3) == 0)
		asusdebug_enable = 0x11223344;
	else if (strncmp(messages, "on", 2) == 0)
		asusdebug_enable = 0;
	return len;
}
static struct file_operations turnon_asusdebug_proc_ops = {
	.read	= turnon_asusdebug_proc_read,
	.write	= turnon_asusdebug_proc_write,
};

void trigger_slowlog_work(struct work_struct *work)
{
	int ret = -1;
	char cmdpath[] = "/system/bin/recvkernelevt";
	char *argv[8] = {cmdpath, "slowlog",NULL};
	char *envp[] = {"HOME=/", "PATH=/sbin:/system/bin:/system/sbin:/vendor/bin", NULL};
	if(time_after(jiffies, trigger_slowlog_time)){
		printk("[Debug+++] slowlog  on userspace\n");
		ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
		printk("[Debug---] slowlog  on userspace, ret = %d\n", ret);
		trigger_slowlog_time = jiffies + 300 * HZ;
	}
	return;
}

static int __init proc_asusdebug_init(void)
{
	proc_create("asusdebug", S_IALLUGO, NULL, &proc_asusdebug_operations);
	proc_create("asusevtlog", S_IRWXUGO, NULL, &proc_asusevtlog_operations);
	proc_create("asusevtlog-switch", S_IRWXUGO, NULL, &proc_evtlogswitch_operations);
	proc_create("asusdebug-switch", S_IRWXUGO, NULL, &turnon_asusdebug_proc_ops);
	PRINTK_BUFFER_VA = ioremap(PRINTK_BUFFER_PA, PRINTK_BUFFER_SIZE);
	mutex_init(&mA);
	mutex_init(&mA_erc);//Record the important power event
	fake_mutex.owner = current;
	fake_mutex.mutex_owner_asusdebug = current;
	fake_mutex.name = " fake_mutex";
	strcpy(fake_completion.name, " fake_completion");
	fake_rtmutex.owner = current;

       	/*ASUS-BBSP SubSys Health Record+++*/
	proc_create("SubSysHealth", S_IRWXUGO, NULL, &proc_SubSysHealth_operations);
	init_completion(&SubSys_C_Complete);
	/*ASUS-BBSP SubSys Health Record---*/

	ASUSEvtlog_workQueue = create_singlethread_workqueue("ASUSEVTLOG_WORKQUEUE");
	ASUSErclog_workQueue  = create_singlethread_workqueue("ASUSERCLOG_WORKQUEUE");//Record the important power event
	
	INIT_WORK(&slow_work, trigger_slowlog_work);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&asusdebug_early_suspend_handler);
#endif

	return 0;
}
module_init(proc_asusdebug_init);
