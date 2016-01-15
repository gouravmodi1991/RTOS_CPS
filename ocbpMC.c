/******************************************************************************
 *
 * OCBP scheduler for RT-Xen 
 * by Dino and Gourav, 2015, Nanyang Technological University
 *
 * based on the code of Preemptive Global Earliest Deadline First  (EDF) 
 */

#include <xen/config.h>
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/domain.h>
#include <xen/delay.h>
#include <xen/event.h>
#include <xen/time.h>
#include <xen/perfc.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <asm/atomic.h>
#include <xen/errno.h>
#include <xen/trace.h>
#include <xen/cpu.h>
#include <xen/keyhandler.h>
#include <xen/trace.h>
#include <xen/guest_access.h>
/*
 * Default parameters: Period and budget in default is 10 and 4 ms, respectively
 */
#define RT_DEFAULT_PERIOD     		(MICROSECS(10))
#define RT_DEFAULT_BUDGET_LOW     	(MICROSECS(4))
#define RT_DEFAULT_BUDGET_HIGH     	(MICROSECS(6))
#define LOW_CRITICALITY			0
#define HIGH_CRITICALITY		1

/*
 * Useful macros
 */
#define RT_PRIV(_ops)     ((struct rt_private *)((_ops)->sched_data))
#define RT_VCPU(_vcpu)    ((struct rt_vcpu *)(_vcpu)->sched_priv)
#define RT_DOM(_dom)      ((struct rt_dom *)(_dom)->sched_priv)
#define RUNQ(_ops)              (&RT_PRIV(_ops)->runq)

/*
 * Flags
 */
/* RT_scheduled: Is this vcpu either running on, or context-switching off,
 * a phyiscal cpu?
 * + Accessed only with Runqueue lock held.
 * + Set when chosen as next in rt_schedule().
 * + Cleared after context switch has been saved in rt_context_saved()
 * + Checked in vcpu_wake to see if we can add to the Runqueue, or if we should
 *   set RT_delayed_runq_add
 * + Checked to be false in runq_insert.
 */
#define __RT_scheduled            1
#define RT_scheduled (1<<__RT_scheduled)
/* RT_delayed_runq_add: Do we need to add this to the Runqueueu once it'd done 
 * being context switching out?
 * + Set when scheduling out in rt_schedule() if prev is runable
 * + Set in rt_vcpu_wake if it finds RT_scheduled set
 * + Read in rt_context_saved(). If set, it adds prev to the Runqueue and
 *   clears the bit.
 *
 */
#define __RT_delayed_runq_add     2
#define RT_delayed_runq_add (1<<__RT_delayed_runq_add)

#define NO_OF_VCPUS 11 // Change it as per the generated instance
//#define NO_OF_VCPUS 3 // Change it as per the generated instance

/*
 * Debug only. Used to printout debug information
 */
#define printtime()\
	({s_time_t now = NOW(); \
	 printk("%u : %3ld.%3ldus : %-19s ",\
		 smp_processor_id(), now/MICROSECS(1), now%MICROSECS(1)/1000, __func__);} )

/*
 * Systme-wide private data, include a global RunQueue
 * The global lock is referenced by schedule_data.schedule_lock from all physical cpus.
 * It can be grabbed via vcpu_schedule_lock_irq()
 */
struct rt_private {
	spinlock_t lock;        /* The global coarse grand lock */
	struct list_head sdom;  /* list of availalbe domains, used for dump */
	struct list_head runq;  /* Ordered list of runnable VMs */
	cpumask_t cpus;         /* cpumask_t of available physical cpus */
	cpumask_t tickled;      /* another cpu in the queue already ticked this one */
	uint16_t criticality;
};

/*
 * Virtual CPU
 */
struct rt_vcpu {
	struct list_head runq_elem; /* On the runqueue list */
	struct list_head sdom_elem; /* On the domain VCPU list */

	/* Up-pointers */
	struct rt_dom *sdom;
	struct vcpu *vcpu;

	/* VCPU parameters, in milliseconds */
	s_time_t period;
	s_time_t budget_low;
	s_time_t budget_high;
	s_time_t deadline;
	uint16_t criticality_vcpu;
	uint16_t offl_flag;

	/* VCPU current infomation in nanosecond */
	long cur_budget;            /* current budget */
	s_time_t last_start;        /* last start time */
	s_time_t cur_deadline;      /* current deadline */

	unsigned flags;             /* mark __RT_scheduled, etc.. */
};

/*
 * Domain
 */
struct rt_dom {
	struct list_head vcpu;      /* link its VCPUs */
	struct list_head sdom_elem; /* link list on rt_priv */
	struct domain *dom;         /* pointer to upper domain */
};

/*
 * RunQueue helper functions
 */
	static int
__vcpu_on_runq(struct rt_vcpu *svc)
{
	return !list_empty(&svc->runq_elem);
}

	static struct rt_vcpu *
__runq_elem(struct list_head *elem)
{
	return list_entry(elem, struct rt_vcpu, runq_elem);
}

/*
 * Debug related code, dump vcpu/cpu information
 */
	static void
rt_dump_vcpu(struct rt_vcpu *svc)
{
	char *cpustr = keyhandler_scratch;
	if ( svc == NULL ) 
	{
		printk("NULL!\n");
		return;
	}
	cpumask_scnprintf(cpustr, sizeof(cpustr), svc->vcpu->cpu_hard_affinity);
	printk("OCBP:_dumpP: [%5d.%-2d] cpu %d, Period %"PRId64", Budget_low %"PRId64", Budget_high %"PRId64", Deadline %"PRId64",Criticality %d,Offline_flag %d, cur_b=%"PRId64",onR=%d runnable=%d cpu_hard_affinity=%s",
			svc->vcpu->domain->domain_id,
			svc->vcpu->vcpu_id,
			svc->vcpu->processor,
			svc->period,
			svc->budget_low,
			svc->budget_high,
			svc->deadline,
			svc->criticality_vcpu,
			svc->offl_flag,
			svc->cur_budget,
			__vcpu_on_runq(svc),
			vcpu_runnable(svc->vcpu),
			cpustr);
	memset(cpustr, 0, sizeof(char)*1024);
	cpumask_scnprintf(cpustr, sizeof(cpustr), cpupool_scheduler_cpumask(svc->vcpu->domain->cpupool));
	printk("cpupool=%s\n", cpustr);
}

	static void
rt_dump_pcpu(const struct scheduler *ops, int cpu)
{
	struct rt_vcpu *svc = RT_VCPU(curr_on_cpu(cpu));

	printtime();
	rt_dump_vcpu(svc);
}

/*
 * should not need lock here. only showing stuff 
 */
	static void
rt_dump(const struct scheduler *ops)
{
	struct list_head *iter_sdom, *iter_svc, *runq, *iter;
	struct rt_private *prv = RT_PRIV(ops);
	struct rt_vcpu *svc;
	int cpu = 0;
	int loop = 0;

	printtime();
	printk("OCBP:_dumpV Priority Scheme: OCBP\n");

	printk("PCPU info: \n");
	for_each_cpu(cpu, &prv->cpus) 
		rt_dump_pcpu(ops, cpu);

	printk("OCBP:_dumpV Global RunQueue info: \n");
	loop = 0;
	runq = RUNQ(ops);
	list_for_each( iter, runq ) 
	{
		svc = __runq_elem(iter);
		printk("\tOCBP:_dumpV RunQ no:%3d: ", ++loop);
		rt_dump_vcpu(svc);
	}

	printk("OCBP:_dumpV Domain info: \n");
	loop = 0;
	list_for_each( iter_sdom, &prv->sdom ) 
	{
		struct rt_dom *sdom;
		sdom = list_entry(iter_sdom, struct rt_dom, sdom_elem);
		printk("\t OCBP:_dumpV domain id: %d\n", sdom->dom->domain_id);

		list_for_each( iter_svc, &sdom->vcpu ) 
		{
			svc = list_entry(iter_svc, struct rt_vcpu, sdom_elem);
			printk("\t\tOCBP:_dumpV Loop.No%3d: ", ++loop);
			rt_dump_vcpu(svc);
		}
	}

	printk("\n");
}

	static inline void
__runq_remove(struct rt_vcpu *svc)
{
	if ( __vcpu_on_runq(svc) )
		list_del_init(&svc->runq_elem);
}

/* OCBP: Data structure to store prioritized VCPU */
struct vcpu_stack {
	struct rt_vcpu vcpu_data;
	struct vcpu_stack * next_vcpu;
};

struct vcpu_stack *first_vcpu = NULL, *temp;

/* OCBP: Funtion to store prioritized VCPU's */
int updateQueue(struct rt_vcpu vcpu_queue) 
{
	temp = xzalloc(struct vcpu_stack);
	temp->vcpu_data = vcpu_queue;
	temp->next_vcpu = first_vcpu;
	first_vcpu = temp;
	return 0;
}

static int glbl_flag = 0;

int gcd(s_time_t a,s_time_t b)
{
    if(b>a)
    return gcd(b,a);

    if(b==0)
    return a;

    return gcd(b,a%b);
}

int lcm(s_time_t a,s_time_t b)
{
    int k=gcd(a,b);

    a/=k;

    return a*b;
}

/* OCBP: Funtion to Calculate Hyperperiod */
s_time_t calculateLCM(struct rt_vcpu * vcpu_to_schedule, int no_of_vcpu)
{
	//s_time_t hyperperiod = 40000;
    	//s_time_t k = vcpu_to_schedule[0].period;
    	s_time_t hyperperiod = vcpu_to_schedule[0].period;
	
	no_of_vcpu--;

    	while(no_of_vcpu >= 1)
    	{
    	       hyperperiod = lcm(hyperperiod, vcpu_to_schedule[no_of_vcpu--].period);
    	}
    	//printk("Hyper-period of N numbers is: %ld\n",hyperperiod);
	return hyperperiod;
}

/* OCBP: Funtion to generate instances of VCPU's over Hyperperiod */
void findInstance(struct rt_vcpu * vcpu_to_schedule, int no_of_vcpu)
{
	s_time_t hyperperiod = 0;
	s_time_t vcpu_instance = 0;
	int count = 0;
	int lclcnt;
	int eachvcpu = no_of_vcpu;

	/* Calculate HyperPeriod (LCM) */
	hyperperiod = calculateLCM(vcpu_to_schedule, no_of_vcpu);

	for(count = 0; count < no_of_vcpu; count++){
		vcpu_instance = hyperperiod / vcpu_to_schedule[count].period;
		vcpu_instance -=1;
		
		for(lclcnt = 0; lclcnt < vcpu_instance; lclcnt++) {

			vcpu_to_schedule[eachvcpu+lclcnt]  			= vcpu_to_schedule[count];
			
			vcpu_to_schedule[eachvcpu+lclcnt].deadline  		= (vcpu_to_schedule[count].deadline +
										  ((lclcnt + 1)*vcpu_to_schedule[count].period));
		}
		eachvcpu += vcpu_instance;
	}
		//printk("%d\n", eachvcpu);
}


int calculatePriority(struct rt_vcpu * vcpu_to_schedule, int flagcnt) 
{
	int count;
	int loop_count = 0;
	int j_fxd_index;
	int vcpu_index;
	s_time_t total_wcet_time = 0; 
	if(flagcnt <= 2)
	{

		printk("OCBP:Cal: D:%ld,C:%d   ",vcpu_to_schedule[flagcnt].deadline, vcpu_to_schedule[flagcnt].criticality_vcpu);

		if(flagcnt == 2) {
				printtime();
			findInstance(vcpu_to_schedule, 3);

			for(j_fxd_index = 0; j_fxd_index < NO_OF_VCPUS ; j_fxd_index++) {
				for(vcpu_index = 0; vcpu_index < (NO_OF_VCPUS-loop_count) ; vcpu_index++) {

					if(vcpu_to_schedule[vcpu_index].criticality_vcpu == HIGH_CRITICALITY) {
						//printk("Entered HIGH Criticality\n");

						/* Check if the current vcpu can be assigned with LOW Priority*/
						for(count = vcpu_index + 1; count < (NO_OF_VCPUS-loop_count) ; count++) {
							total_wcet_time += vcpu_to_schedule[count].budget_high;
						}
						for(count = vcpu_index - 1; count >= 0; count--) {
							total_wcet_time += vcpu_to_schedule[count].budget_high;
						}

						if(total_wcet_time >= vcpu_to_schedule[vcpu_index].deadline) {
							//No time to execute the current vcpu
							//printk("Job %d cannot be assigned Low Priority\n", vcpu_index);	
						}
						else {
							//if(((vcpu_to_schedule[vcpu_index].deadline - total_wcet_time) - vcpu_to_schedule[vcpu_index].release) 
							if(((vcpu_to_schedule[vcpu_index].deadline - total_wcet_time)) 
									>= vcpu_to_schedule[vcpu_index].budget_high) {
								//printk("OCBP:_Cal: Job %d is assigned Low Priority\n", vcpu_index);	
								updateQueue(vcpu_to_schedule[vcpu_index]);
								break;
							}
							else {
								//printk("Job %d cannot be assigned Low Priority\n", vcpu_index);	
							}
						}
						//printk("Total WCET time = %ld\n", total_wcet_time);
						total_wcet_time = 0;

					}
					else {
						//printk("Entered LOW Criticality\n");

						/* Check if the current vcpu can be assigned with LOW Priority*/
						for(count = vcpu_index + 1; count < (NO_OF_VCPUS-loop_count) ; count++) {
							total_wcet_time += vcpu_to_schedule[count].budget_low;
						}
						for(count = vcpu_index - 1; count >= 0; count--) {
							total_wcet_time += vcpu_to_schedule[count].budget_low;
						}

						if(total_wcet_time >= vcpu_to_schedule[vcpu_index].deadline) {
							//No time to execute the current vcpu
							//printk("Job %d cannot be assigned Low Priority\n", vcpu_index);	
						}
						else {
							//if(((vcpu_to_schedule[vcpu_index].deadline - total_wcet_time)- vcpu_to_schedule[vcpu_index].release) 
							if(((vcpu_to_schedule[vcpu_index].deadline - total_wcet_time)) 
									>= vcpu_to_schedule[vcpu_index].budget_low) {
								//printk("OCBP:_Cal: Job %d is assigned Low Priority\n", vcpu_index);	
								updateQueue(vcpu_to_schedule[vcpu_index]);
								break;
							}
							else {
								//printk("Job %d cannot be assigned Low Priority\n", vcpu_index);	
							}
						}
						//printk("Total WCET time = %ld\n", total_wcet_time);
					}	
					total_wcet_time = 0;
				}
				/* Shifting the array index */
				if(vcpu_index == (NO_OF_VCPUS -1 -loop_count)) {
					/* No need to shift the arrays because of the last element*/
				}
				else {
					for(count = vcpu_index; count < (NO_OF_VCPUS-1-loop_count); count++) {
						vcpu_to_schedule[count] = vcpu_to_schedule[count+1];
					}
				}
				total_wcet_time = 0;
				loop_count++;
				//printk("loop count %d\n", loop_count);
			}
			printtime();
			glbl_flag = 1;
		}
	}
	return 0;
}

	static void
__runq_insert(const struct scheduler *ops, struct rt_vcpu *svc)
{
	struct list_head *runq = RUNQ(ops);
	struct list_head *iter;
	spinlock_t *schedule_lock;

	static int loop_count = 0;
	static struct rt_vcpu vcpu_prioritized[NO_OF_VCPUS];

	schedule_lock = per_cpu(schedule_data, svc->vcpu->processor).schedule_lock;
	ASSERT( spin_is_locked(schedule_lock) );

	/* Debug only */
	if ( __vcpu_on_runq(svc) )
	{
		rt_dump(ops);
	}
	ASSERT( !__vcpu_on_runq(svc) );

	/* OCBP Implementation */
	if(glbl_flag == 1)
	{
		while(first_vcpu != NULL)
		{
			vcpu_prioritized[loop_count] = first_vcpu->vcpu_data;
			printk("\nOCBP:RUNQ Deadline Time from Prioritized VCPU %ld\n",vcpu_prioritized[loop_count].deadline);
			first_vcpu = first_vcpu->next_vcpu;
			//list_add_tail(&vcpu_prioritized[loop_count].runq_elem, runq);
			//printk("OCBP:RUNQ: Inserted into Runq\n");
			loop_count++;
		}
		loop_count = 0;
		glbl_flag = 0;	
	}

	/* End */

	list_for_each(iter, runq) 
	{
		struct rt_vcpu * iter_svc = __runq_elem(iter);

		// svc still has budget 
		if ( svc->cur_budget > 0 ) 
		{
			//if ( iter_svc->cur_budget == 0 )
				break;
		} 
		else 
		{ // svc has no budget 
			if ( iter_svc->cur_budget == 0 )
				break;
		}

		/*
		// svc still has budget 
		if ( svc->cur_budget > 0 ) 
		{
		if ( iter_svc->cur_budget == 0 ||
		svc->cur_deadline <= iter_svc->cur_deadline )
		break;
		} 
		else 
		{ // svc has no budget 
		if ( iter_svc->cur_budget == 0 &&
		svc->cur_deadline <= iter_svc->cur_deadline )
		break;
		}
		 */
	}

	list_add_tail(&svc->runq_elem, iter);
}

/*
 * Init/Free related code
 */
	static int
rt_init(struct scheduler *ops)
{
	struct rt_private *prv = xzalloc(struct rt_private);

	if ( prv == NULL )
		return -ENOMEM;
	ops->sched_data = prv;
	spin_lock_init(&prv->lock);
	INIT_LIST_HEAD(&prv->sdom);
	INIT_LIST_HEAD(&prv->runq);
	
	/* OCBP: Initialised Criticality of the System to Low_Criticality*/
	prv->criticality = LOW_CRITICALITY;

	printtime();
	printk("\n");

	return 0;
}

	static void
rt_deinit(const struct scheduler *ops)
{
	struct rt_private *prv = RT_PRIV(ops);

	printtime();
	printk("\n");
	xfree(prv);
}

/* 
 * point per_cpu spinlock to the global system lock; all cpu have same global system lock 
 */
	static void *
rt_alloc_pdata(const struct scheduler *ops, int cpu)
{
	struct rt_private *prv = RT_PRIV(ops);

	cpumask_set_cpu(cpu, &prv->cpus);

	per_cpu(schedule_data, cpu).schedule_lock = &prv->lock;

	printtime();
	printk("%s total cpus: %d", __FUNCTION__, cpumask_weight(&prv->cpus));
	/* same as credit2, not a bogus pointer */
	return (void *)1;
}

	static void
rt_free_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
	struct rt_private * prv = RT_PRIV(ops);
	cpumask_clear_cpu(cpu, &prv->cpus);
	printtime();
	printk("%s cpu=%d\n", __FUNCTION__, cpu);
}

	static void *
rt_alloc_domdata(const struct scheduler *ops, struct domain *dom)
{
	unsigned long flags;
	struct rt_dom *sdom;
	struct rt_private * prv = RT_PRIV(ops);

	printtime();
	printk("OCBP:_domdata dom=%d\n", dom->domain_id);

	sdom = xzalloc(struct rt_dom);
	if ( sdom == NULL ) 
	{
		printk("%s, xzalloc failed\n", __func__);
		return NULL;
	}

	INIT_LIST_HEAD(&sdom->vcpu);
	INIT_LIST_HEAD(&sdom->sdom_elem);
	sdom->dom = dom;

	/* spinlock here to insert the dom */
	spin_lock_irqsave(&prv->lock, flags);
	list_add_tail(&sdom->sdom_elem, &(prv->sdom));
	spin_unlock_irqrestore(&prv->lock, flags);

	return (void *)sdom;
}

	static void
rt_free_domdata(const struct scheduler *ops, void *data)
{
	unsigned long flags;
	struct rt_dom *sdom = data;
	struct rt_private *prv = RT_PRIV(ops);

	printtime();
	printk("OCBP:_freedomdata dom=%d\n", sdom->dom->domain_id);

	spin_lock_irqsave(&prv->lock, flags);
	list_del_init(&sdom->sdom_elem);
	spin_unlock_irqrestore(&prv->lock, flags);
	xfree(data);
}

	static int
rt_dom_init(const struct scheduler *ops, struct domain *dom)
{
	struct rt_dom *sdom;

	printtime();
	printk("OCBP:_dominit dom=%d\n", dom->domain_id);

	/* IDLE Domain does not link on rt_private */
	if ( is_idle_domain(dom) ) 
		return 0;

	sdom = rt_alloc_domdata(ops, dom);
	if ( sdom == NULL ) 
	{
		printk("%s, failed\n", __func__);
		return -ENOMEM;
	}
	dom->sched_priv = sdom;

	return 0;
}

	static void
rt_dom_destroy(const struct scheduler *ops, struct domain *dom)
{
	printtime();
	printk("dom=%d\n", dom->domain_id);

	rt_free_domdata(ops, RT_DOM(dom));
}

	static void *
rt_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd)
{
	struct rt_vcpu *svc;
	s_time_t now = NOW();
	long count;

	/* Allocate per-VCPU info */
	svc = xzalloc(struct rt_vcpu);
	if ( svc == NULL ) 
	{
		printk("%s, xzalloc failed\n", __func__);
		return NULL;
	}

	INIT_LIST_HEAD(&svc->runq_elem);
	INIT_LIST_HEAD(&svc->sdom_elem);
	svc->flags = 0U;
	svc->sdom = dd;
	svc->vcpu = vc;
	svc->last_start = 0;            /* init last_start is 0 */

	svc->period = RT_DEFAULT_PERIOD;
	svc->deadline = RT_DEFAULT_PERIOD;

	if ( !is_idle_vcpu(vc) ) {
		svc->budget_low = RT_DEFAULT_BUDGET_LOW;
		svc->budget_high = RT_DEFAULT_BUDGET_HIGH;
	}

	count = (now/MICROSECS(svc->deadline)) + 1;
	/* sync all VCPU's start time to 0 */
	//svc->cur_deadline += count * MICROSECS(svc->period);
	svc->cur_deadline += count * MICROSECS(svc->deadline);

	svc->cur_budget = svc->budget_low*1000; /* counting in microseconds level */
	/* Debug only: dump new vcpu's info */
	printtime();
	rt_dump_vcpu(svc);

	return svc;
}

	static void
rt_free_vdata(const struct scheduler *ops, void *priv)
{
	struct rt_vcpu *svc = priv;

	/* Debug only: dump freed vcpu's info */
	printtime();
	rt_dump_vcpu(svc);
	xfree(svc);
}

/*
 * TODO: Do we need to add vc to the new Runqueue?
 * This function is called in sched_move_domain() in schedule.c
 * When move a domain to a new cpupool, 
 * may have to add vc to the Runqueue of the new cpupool
 */
	static void
rt_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
	struct rt_vcpu *svc = RT_VCPU(vc);

	/* Debug only: dump info of vcpu to insert */
	printtime();
	rt_dump_vcpu(svc);

	/* not addlocate idle vcpu to dom vcpu list */
	if ( is_idle_vcpu(vc) )
		return;


	list_add_tail(&svc->sdom_elem, &svc->sdom->vcpu);   /* add to dom vcpu list */
}

/*
 * TODO: same as rt_vcpu_insert()
 */
	static void
rt_vcpu_remove(const struct scheduler *ops, struct vcpu *vc)
{
	struct rt_vcpu * const svc = RT_VCPU(vc);
	struct rt_dom * const sdom = svc->sdom;

	printtime();
	rt_dump_vcpu(svc);

	BUG_ON( sdom == NULL );
	BUG_ON( __vcpu_on_runq(svc) );

	if ( !is_idle_vcpu(vc) ) 
		list_del_init(&svc->sdom_elem);
}

/* 
 * Pick a valid CPU for the vcpu vc
 * Valid CPU of a vcpu is intesection of vcpu's affinity and available cpus
 */
	static int
rt_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
	cpumask_t cpus;
	cpumask_t *online;
	int cpu;
	struct rt_private * prv = RT_PRIV(ops);

	online = cpupool_scheduler_cpumask(vc->domain->cpupool);
	cpumask_and(&cpus, &prv->cpus, online);
	cpumask_and(&cpus, &cpus, vc->cpu_hard_affinity);

	cpu = cpumask_test_cpu(vc->processor, &cpus)
		? vc->processor 
		: cpumask_cycle(vc->processor, &cpus);
	ASSERT( !cpumask_empty(&cpus) && cpumask_test_cpu(cpu, &cpus) );

	return cpu;
}

/*
 * Burn budget at microsecond level. 
 */
	static void
burn_budgets(const struct scheduler *ops, struct rt_vcpu *svc, s_time_t now) 
{
	s_time_t delta;
	long count = 0;
	struct rt_private * prv = RT_PRIV(ops);

	/* don't burn budget for idle VCPU */
	if ( is_idle_vcpu(svc->vcpu) ) 
	{
		return;
	}

	/* first time called for this svc, update last_start */
	if ( svc->last_start == 0 ) 
	{
		svc->last_start = now;
		return;
	}

	/*
	 * update deadline info: When deadline is in the past,
	 * it need to be updated to the deadline of the current period,
	 * and replenish the budget 
	 */
	delta = now - svc->cur_deadline;
	if ( delta >= 0 ) 
	{
		/*OCBP: Added criticality*/
		
		count = ( delta/MICROSECS(svc->deadline) ) + 1;
		svc->cur_deadline += count * MICROSECS(svc->deadline);

        	if(prv->criticality == LOW_CRITICALITY) 
		{
			svc->cur_budget = svc->budget_low * 1000;
		}
		else
			svc->cur_budget = svc->budget_high * 1000;
			
		/*
		count = ( delta/MICROSECS(svc->period) ) + 1;
		svc->cur_deadline += count * MICROSECS(svc->deadline);
		//svc->cur_deadline += count * MICROSECS(svc->period);
		svc->cur_budget = svc->budget_low * 1000;
		*/
		return;
	}

	/* burn at nanoseconds level */
	delta = now - svc->last_start;
	/* 
	 * delta < 0 only happens in nested virtualization;
	 * TODO: how should we handle delta < 0 in a better way? */
	if ( delta < 0 ) 
	{
		printk("%s, ATTENTION: now is behind last_start! delta = %ld for ",
				__func__, delta);
		rt_dump_vcpu(svc);
		svc->last_start = now;  /* update last_start */
		svc->cur_budget = 0;   /* FIXME: should we recover like this? */
		return;
	}

	if ( svc->cur_budget == 0 ) 
		return;

	svc->cur_budget -= delta;
	if ( svc->cur_budget < 0 ) 
		svc->cur_budget = 0;
}

/* 
 * RunQ is sorted. Pick first one within cpumask. If no one, return NULL
 * lock is grabbed before calling this function 
 */
	static struct rt_vcpu *
__runq_pick(const struct scheduler *ops, cpumask_t mask)
{
	struct list_head *runq = RUNQ(ops);
	struct list_head *iter;
	struct rt_vcpu *svc = NULL;
	struct rt_vcpu *iter_svc = NULL;
	cpumask_t cpu_common;
	cpumask_t *online;
	struct rt_private * prv = RT_PRIV(ops);

	list_for_each(iter, runq) 
	{
		iter_svc = __runq_elem(iter);

		/* mask is intersection of cpu_hard_affinity and cpupool and priv->cpus */
		online = cpupool_scheduler_cpumask(iter_svc->vcpu->domain->cpupool);
		cpumask_and(&cpu_common, online, &prv->cpus);
		cpumask_and(&cpu_common, &cpu_common, iter_svc->vcpu->cpu_hard_affinity);
		cpumask_and(&cpu_common, &mask, &cpu_common);
		if ( cpumask_empty(&cpu_common) )
			continue;

		if ( iter_svc->cur_budget <= 0 )
			continue;

		svc = iter_svc;
		break;
	}

	return svc;
}

/*
 * Update vcpu's budget and sort runq by insert the modifed vcpu back to runq
 * lock is grabbed before calling this function 
 */
	static void
__repl_update(const struct scheduler *ops, s_time_t now)
{
	struct list_head *runq = RUNQ(ops);
	struct list_head *iter;
	struct list_head *tmp;
	struct rt_vcpu *svc = NULL;
	struct rt_private * prv = RT_PRIV(ops);

	s_time_t diff;
	long count;

	list_for_each_safe(iter, tmp, runq) 
	{
		svc = __runq_elem(iter);

		diff = now - svc->cur_deadline;
		if ( diff > 0 ) 
		{
			count = ( diff/MICROSECS(svc->deadline) ) + 1;
			svc->cur_deadline += count * MICROSECS(svc->deadline);

        		if(prv->criticality == LOW_CRITICALITY) 
			{
				svc->cur_budget = svc->budget_low * 1000;
				__runq_remove(svc);
				__runq_insert(ops, svc);
			}
			else
			{
				if(svc->criticality_vcpu == LOW_CRITICALITY)
				{
					svc->cur_budget = 0;
					__runq_remove(svc);
					__runq_insert(ops, svc);
					printk(" OC__:LC :D ");
				}
				else
				{
					svc->cur_budget = svc->budget_high * 1000;
					__runq_remove(svc);
					__runq_insert(ops, svc);
					printk(" OC__:HC :D ");
				}
			}
	
			/*
			count = (diff/MICROSECS(svc->period)) + 1;
			//svc->cur_deadline += count * MICROSECS(svc->period);
			svc->cur_deadline += count * MICROSECS(svc->deadline);
			svc->cur_budget = svc->budget_low * 1000;

			//printk("OCBP:1 ");
			__runq_remove(svc);
			__runq_insert(ops, svc);
			*/
		}
	}
}

/* 
 * schedule function for rt scheduler.
 * The lock is already grabbed in schedule.c, no need to lock here 
 */
	static struct task_slice
rt_schedule(const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
	const int cpu = smp_processor_id();
	struct rt_private * prv = RT_PRIV(ops);
	struct rt_vcpu * const scurr = RT_VCPU(current);
	struct rt_vcpu * snext = NULL;
	struct task_slice ret = { .migrated = 0 };

	/* clear ticked bit now that we've been scheduled */
	if ( cpumask_test_cpu(cpu, &prv->tickled) )
		cpumask_clear_cpu(cpu, &prv->tickled);

	/* burn_budget would return for IDLE VCPU */
	burn_budgets(ops, scurr, now);

	__repl_update(ops, now);

	if ( tasklet_work_scheduled ) 
	{
		snext = RT_VCPU(idle_vcpu[cpu]);
		prv->criticality = LOW_CRITICALITY;
	} 
	else 
	{
		cpumask_t cur_cpu;
		cpumask_clear(&cur_cpu);
		cpumask_set_cpu(cpu, &cur_cpu);
		snext = __runq_pick(ops, cur_cpu);
		if ( snext == NULL ) {
			snext = RT_VCPU(idle_vcpu[cpu]);
			prv->criticality = LOW_CRITICALITY;
		}

		/* if scurr has higher priority and budget, still pick scurr */
		/*
		   if ( !is_idle_vcpu(current) &&
		   vcpu_runnable(current) &&
		   scurr->cur_budget > 0 &&
		   diff = now - svc->cur_deadline;
		   ( is_idle_vcpu(snext->vcpu) ||
		   scurr->cur_deadline <= snext->cur_deadline ) ) 
		   snext = scurr;
		 */

		/* OCBP: To check for the current and next VCPU */
		if ( !is_idle_vcpu(current) && vcpu_runnable(current) && scurr->cur_budget > 0 ) 
			snext = scurr;

	}

	if ( snext != scurr &&
			!is_idle_vcpu(current) &&
			vcpu_runnable(current) )
		set_bit(__RT_delayed_runq_add, &scurr->flags);


	snext->last_start = now;
	if ( !is_idle_vcpu(snext->vcpu) ) 
	{
		if ( snext != scurr ) 
		{
			__runq_remove(snext);
			set_bit(__RT_scheduled, &snext->flags);
		}
		if ( snext->vcpu->processor != cpu ) 
		{
			snext->vcpu->processor = cpu;
			ret.migrated = 1;
		}
	}

	ret.time = MILLISECS(1); /* sched quantum */
	ret.task = snext->vcpu;

	return ret;
}

/*
 * Remove VCPU from RunQ
 * The lock is already grabbed in schedule.c, no need to lock here 
 */
	static void
rt_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc)
{
	struct rt_vcpu * const svc = RT_VCPU(vc);

	BUG_ON( is_idle_vcpu(vc) );

	if ( curr_on_cpu(vc->processor) == vc ) 
	{
		cpu_raise_softirq(vc->processor, SCHEDULE_SOFTIRQ);
		return;
	}

	if ( __vcpu_on_runq(svc) ) 
	{
		__runq_remove(svc);
		printk("%s: vcpu should not on runq in vcpu_sleep()\n", __FUNCTION__);
		BUG();
	}

	clear_bit(__RT_delayed_runq_add, &svc->flags);
}

/*
 * Pick a vcpu on a cpu to kick out to place the running candidate
 * Called by wake() and context_saved()
 * We have a running candidate here, the kick logic is:
 * Among all the cpus that are within the cpu affinity
 * 1) if the new->cpu is idle, kick it. This could benefit cache hit
 * 2) if there are any idle vcpu, kick it.
 * 3) now all pcpus are busy, among all the running vcpus, pick lowest priority one
 *    if snext has higher priority, kick it.
 *
 * TODO:
 * 1) what if these two vcpus belongs to the same domain?
 *    replace a vcpu belonging to the same domain introduces more overhead
 *
 * lock is grabbed before calling this function 
 */
	static void
runq_tickle(const struct scheduler *ops, struct rt_vcpu *new)
{
	struct rt_private * prv = RT_PRIV(ops);
	struct rt_vcpu * latest_deadline_vcpu = NULL;    /* lowest priority scheduled */
	struct rt_vcpu * iter_svc;
	struct vcpu * iter_vc;
	int cpu = 0;
	cpumask_t not_tickled;
	cpumask_t *online;

	if ( new == NULL || is_idle_vcpu(new->vcpu) ) 
		return;

	online = cpupool_scheduler_cpumask(new->vcpu->domain->cpupool);
	cpumask_and(&not_tickled, online, &prv->cpus);
	cpumask_and(&not_tickled, &not_tickled, new->vcpu->cpu_hard_affinity);
	cpumask_andnot(&not_tickled, &not_tickled, &prv->tickled);

	/* 1) if new's previous cpu is idle, kick it for cache benefit */
	if ( is_idle_vcpu(curr_on_cpu(new->vcpu->processor)) ) 
	{
		cpumask_set_cpu(new->vcpu->processor, &prv->tickled);
		cpu_raise_softirq(new->vcpu->processor, SCHEDULE_SOFTIRQ);
		return;
	}

	/* 2) if there are any idle pcpu, kick it */
	/* The same loop also find the one with lowest priority */
	for_each_cpu(cpu, &not_tickled) 
	{
		iter_vc = curr_on_cpu(cpu);
		if ( is_idle_vcpu(iter_vc) ) 
		{
			cpumask_set_cpu(cpu, &prv->tickled);
			cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);
			return;
		}
		iter_svc = RT_VCPU(iter_vc);
		if ( latest_deadline_vcpu == NULL || 
				iter_svc->cur_deadline > latest_deadline_vcpu->cur_deadline )
			latest_deadline_vcpu = iter_svc;
	}

	/* 3) candicate has higher priority, kick out the lowest priority vcpu */
	/*OCBP: Only 1 VCPU is scheduled at a time since it is a unicore processor No need of eviction*/
	/*
	   if ( latest_deadline_vcpu != NULL && new->cur_deadline < latest_deadline_vcpu->cur_deadline ) 
	   {
	   cpumask_set_cpu(latest_deadline_vcpu->vcpu->processor, &prv->tickled);
	   cpu_raise_softirq(latest_deadline_vcpu->vcpu->processor, SCHEDULE_SOFTIRQ);
	   }
	 */
	return;
}

/* 
 * Should always wake up runnable vcpu, put it back to RunQ. 
 * Check priority to raise interrupt 
 * The lock is already grabbed in schedule.c, no need to lock here 
 * TODO: what if these two vcpus belongs to the same domain? 
 */
	static void
rt_vcpu_wake(const struct scheduler *ops, struct vcpu *vc)
{
	struct rt_vcpu * const svc = RT_VCPU(vc);
	s_time_t diff;
	s_time_t now = NOW();
	long count = 0;
	struct rt_private * prv = RT_PRIV(ops);
	struct rt_vcpu * snext = NULL;        /* highest priority on RunQ */

	BUG_ON( is_idle_vcpu(vc) );

	if ( unlikely(curr_on_cpu(vc->processor) == vc) ) 
		return;

	/* on RunQ, just update info is ok */
	if ( unlikely(__vcpu_on_runq(svc)) ) 
		return;

	/* If context hasn't been saved for this vcpu yet, we can't put it on
	 * the Runqueue. Instead, we set a flag so that it will be put on the Runqueue
	 * After the context has been saved. */
	if ( unlikely(test_bit(__RT_scheduled, &svc->flags)) ) 
	{
		set_bit(__RT_delayed_runq_add, &svc->flags);
		return;
	}

	/* update deadline info */
	diff = now - svc->cur_deadline;
	if ( diff >= 0 ) 
	{
		count = ( diff/MICROSECS(svc->deadline) ) + 1;
		//count = ( diff/MICROSECS(svc->period) ) + 1;
		svc->cur_deadline += count * MICROSECS(svc->deadline);
		//svc->cur_deadline += count * MICROSECS(svc->period);
		svc->cur_budget = svc->budget_low * 1000;
	}

	//printk("OCBP:3 ");
	__runq_insert(ops, svc);
	__repl_update(ops, now);
	snext = __runq_pick(ops, prv->cpus);    /* pick snext from ALL valid cpus */
	runq_tickle(ops, snext);

	return;
}

/* 
 * scurr has finished context switch, insert it back to the RunQ,
 * and then pick the highest priority vcpu from runq to run 
 */
	static void
rt_context_saved(const struct scheduler *ops, struct vcpu *vc)
{
	struct rt_vcpu * svc = RT_VCPU(vc);
	struct rt_vcpu * snext = NULL;
	struct rt_private * prv = RT_PRIV(ops);
	spinlock_t *lock = vcpu_schedule_lock_irq(vc);

	clear_bit(__RT_scheduled, &svc->flags);
	/* not insert idle vcpu to runq */
	if ( is_idle_vcpu(vc) ) 
		goto out;

	if ( test_and_clear_bit(__RT_delayed_runq_add, &svc->flags) && 
			likely(vcpu_runnable(vc)) ) 
	{

		//printk("OCBP:2 ");
		__runq_insert(ops, svc);
		__repl_update(ops, NOW());
		snext = __runq_pick(ops, prv->cpus);    /* pick snext from ALL cpus */
		runq_tickle(ops, snext);
	}
out:
	vcpu_schedule_unlock_irq(lock, vc);
}

/*
 * set/get each vcpu info of each domain
 */
	static int
rt_dom_cntl(
		const struct scheduler *ops, 
		struct domain *d, 
		struct xen_domctl_scheduler_op *op)
{
	xen_domctl_sched_rt_params_t *local_sched;
	struct rt_dom * const sdom = RT_DOM(d);
	struct list_head *iter;
	int vcpu_index = 0;
	int rc = -EINVAL;
	//int loop_count = 0;
	static int loop_index = 0;
	static struct rt_vcpu local_vcpu[NO_OF_VCPUS];
	//struct rt_vcpu vcpu_prioritized[3];

	struct rt_private * prv = RT_PRIV(ops);

	switch ( op->cmd )
	{
		case XEN_DOMCTL_SCHEDOP_getnumvcpus:
			op->u.rt.nr_vcpus = 0;
			list_for_each( iter, &sdom->vcpu ) 
				vcpu_index++;
			op->u.rt.nr_vcpus = vcpu_index;
			rc = 0;
			break;
		case XEN_DOMCTL_SCHEDOP_getinfo:
			/* for debug use, whenever adjust Dom0 parameter, do global dump */
			if ( d->domain_id == 0 ) 
				rt_dump(ops);

			op->u.rt.nr_vcpus = 0;
			list_for_each( iter, &sdom->vcpu ) 
				vcpu_index++;
			op->u.rt.nr_vcpus = vcpu_index;
			local_sched = xzalloc_array(xen_domctl_sched_rt_params_t, vcpu_index);
			vcpu_index = 0;
			list_for_each( iter, &sdom->vcpu ) 
			{
				struct rt_vcpu * svc = list_entry(iter, struct rt_vcpu, sdom_elem);

				local_sched[vcpu_index].budget_low = svc->budget_low;
				local_sched[vcpu_index].budget_high= svc->budget_high;
				local_sched[vcpu_index].deadline = svc->deadline;
				local_sched[vcpu_index].period = svc->period;
				local_sched[vcpu_index].criticality_vcpu = svc->criticality_vcpu;
				local_sched[vcpu_index].offl_flag = svc->offl_flag;
				local_sched[vcpu_index].index = vcpu_index;
				vcpu_index++;
			}
			copy_to_guest(op->u.rt.vcpu, local_sched, vcpu_index);
			rc = 0;
			break;
		case XEN_DOMCTL_SCHEDOP_putinfo:
			list_for_each( iter, &sdom->vcpu ) 
			{
				struct rt_vcpu * svc = list_entry(iter, struct rt_vcpu, sdom_elem);

				/* adjust per VCPU parameter */
				if ( op->u.rt.vcpu_index == svc->vcpu->vcpu_id ) 
				{ 
					vcpu_index = op->u.rt.vcpu_index;

					if ( vcpu_index < 0 ) 
						printk("OCBP:XEN_DOMCTL_SCHEDOP_putinfo: vcpu_index=%d\n",
								vcpu_index);
					else {
						printk("OCBP:XEN_DOMCTL_SCHEDOP_putinfo: "
								"vcpu_index=%d,period=%"PRId64",budget_low=%"PRId64",budget_high=%"PRId64",deadline=%"PRId64"\n",
								vcpu_index, op->u.rt.period, op->u.rt.budget_low, op->u.rt.budget_high,op->u.rt.deadline);
					}

					svc->period = op->u.rt.period;
					svc->budget_low = op->u.rt.budget_low;
					svc->budget_high = op->u.rt.budget_high;
					svc->deadline = op->u.rt.deadline;
					svc->criticality_vcpu = op->u.rt.criticality_vcpu;
					svc->offl_flag = op->u.rt.offl_flag;

					if ( vcpu_index >= 0 ) 
					{
						if(svc->offl_flag == 0)
						{
						printk("OBCP:1 LI %d VI %d  ", loop_index, vcpu_index);
						local_vcpu[loop_index] = *svc;
						calculatePriority(local_vcpu, loop_index);
						if(loop_index == 2)
						{
							loop_index = -1;
						}
						loop_index++;
						}
						else 
						{
							printk("\nOCBP:System in High Criticality mode\n");
							prv->criticality = HIGH_CRITICALITY;
						}
					}
					break;
				}
			}
			rc = 0;
			break;
	}

	return rc;
}

static struct rt_private _rt_priv;

const struct scheduler sched_rt_def = {
	.name           = "SMP RT Scheduler",
	.opt_name       = "rt",
	.sched_id       = XEN_SCHEDULER_RT_DS,
	.sched_data     = &_rt_priv,

	.dump_cpu_state = rt_dump_pcpu,
	.dump_settings  = rt_dump,
	.init           = rt_init,
	.deinit         = rt_deinit,
	.alloc_pdata    = rt_alloc_pdata,
	.free_pdata     = rt_free_pdata,
	.alloc_domdata  = rt_alloc_domdata,
	.free_domdata   = rt_free_domdata,
	.init_domain    = rt_dom_init,
	.destroy_domain = rt_dom_destroy,
	.alloc_vdata    = rt_alloc_vdata,
	.free_vdata     = rt_free_vdata,
	.insert_vcpu    = rt_vcpu_insert,
	.remove_vcpu    = rt_vcpu_remove,

	.adjust         = rt_dom_cntl,

	.pick_cpu       = rt_cpu_pick,
	.do_schedule    = rt_schedule,
	.sleep          = rt_vcpu_sleep,
	.wake           = rt_vcpu_wake,
	.context_saved  = rt_context_saved,
};

