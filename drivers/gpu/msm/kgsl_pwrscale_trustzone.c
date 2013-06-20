/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <mach/socinfo.h>
#include <mach/scm.h>

#include "kgsl.h"
#include "kgsl_pwrscale.h"
#include "kgsl_device.h"


static void tz_wake(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)

#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
#include <linux/module.h>
#endif

#define TZ_GOVERNOR_PERFORMANCE 0
#define TZ_GOVERNOR_ONDEMAND    1
#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
#define TZ_GOVERNOR_SIMPLE        2
#endif

struct tz_priv {
    int governor;
    unsigned int no_switch_cnt;
    unsigned int skip_cnt;
    struct kgsl_power_stats bin;
};
spinlock_t tz_lock;

/* FLOOR is 5msec to capture up to 3 re-draws
 * per frame for 60fps content.
 */
#define FLOOR                        5000
#define SWITCH_OFF                200
#define SWITCH_OFF_RESET_TH        40
#define SKIP_COUNTER                500
#define TZ_RESET_ID                0x3
#define TZ_UPDATE_ID                0x4

#ifdef CONFIG_MSM_SCM
/* Trap into the TrustZone, and call funcs there. */
static int __secure_tz_entry(u32 cmd, u32 val, u32 id)
{
    int ret;
    spin_lock(&tz_lock);
    __iowmb();
    ret = scm_call_atomic2(SCM_SVC_IO, cmd, val, id);
    spin_unlock(&tz_lock);
    return ret;
}
#else
static int __secure_tz_entry(u32 cmd, u32 val, u32 id)

{
	if (device->state != KGSL_STATE_NAP)
		kgsl_pwrctrl_pwrlevel_change(device,
					device->pwrctrl.default_pwrlevel);
}


/*** extern var ***/
bool gpu_idle;
short idle_counter;

//#define DEBUG

static ssize_t tz_governor_show(struct kgsl_device *device,
                                struct kgsl_pwrscale *pwrscale,
                                char *buf)
{
    struct tz_priv *priv = pwrscale->priv;
    int ret;
    
    if (priv->governor == TZ_GOVERNOR_ONDEMAND)
        ret = snprintf(buf, 10, "ondemand\n");
#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
    else if (priv->governor == TZ_GOVERNOR_SIMPLE)
        ret = snprintf(buf, 8, "simple\n");
#endif
    else
        ret = snprintf(buf, 13, "performance\n");
    
    return ret;
}

static ssize_t tz_governor_store(struct kgsl_device *device,
                                 struct kgsl_pwrscale *pwrscale,
                                 const char *buf, size_t count)
{
    char str[20];
    struct tz_priv *priv = pwrscale->priv;
    struct kgsl_pwrctrl *pwr = &device->pwrctrl;
    int ret;
    
    ret = sscanf(buf, "%20s", str);
    if (ret != 1)
        return -EINVAL;
    
    mutex_lock(&device->mutex);
    
    if (!strncmp(str, "ondemand", 8))
        priv->governor = TZ_GOVERNOR_ONDEMAND;
#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
    else if (!strncmp(str, "simple", 6))
        priv->governor = TZ_GOVERNOR_SIMPLE;
#endif
    else if (!strncmp(str, "performance", 11))
        priv->governor = TZ_GOVERNOR_PERFORMANCE;
    
    if (priv->governor == TZ_GOVERNOR_PERFORMANCE)
        kgsl_pwrctrl_pwrlevel_change(device, pwr->max_pwrlevel);
    
    mutex_unlock(&device->mutex);
    return count;
}

PWRSCALE_POLICY_ATTR(governor, 0644, tz_governor_show, tz_governor_store);


#define SAMPLE_TIME_MS 20

#define HISTORY_SIZE 6
#define GPU_IDLE_THRESHOLD 10

static void gpu_idle_detection(struct kgsl_device *device, int load)
{

	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if ((pwr->active_pwrlevel >= pwr->min_pwrlevel - 1) 
				&& (GPU_IDLE_THRESHOLD >= load))
	{
		if (idle_counter < HISTORY_SIZE)
			idle_counter += 1;
	}
	else if (idle_counter > 0)
		idle_counter -= 2;
	
	if (idle_counter >= HISTORY_SIZE)
		gpu_idle = true;
	else if (idle_counter <= 0)
		gpu_idle = false;

    struct tz_priv *priv = pwrscale->priv;
    if (device->state != KGSL_STATE_NAP &&
#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
        (priv->governor == TZ_GOVERNOR_ONDEMAND ||
         priv->governor == TZ_GOVERNOR_SIMPLE))
#else
        priv->governor == TZ_GOVERNOR_ONDEMAND)
#endif
        kgsl_pwrctrl_pwrlevel_change(device,
                                     device->pwrctrl.default_pwrlevel);

}

#define GO_HIGHSPEED_LOAD 90

static unsigned int interactive_load[4][2] = {
	{100,30},
	{60,25},
	{50,20},
	{40,0}};


unsigned int up_threshold(int gpu_state){
	return interactive_load[gpu_state][0]; }
unsigned int down_threshold(int gpu_state){
	return interactive_load[gpu_state][1]; }

#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
/* KGSL Simple GPU Governor */
/* Copyright (c) 2011-2013, Paul Reioux (Faux123). All rights reserved. */
static int default_laziness = 5;
module_param_named(simple_laziness, default_laziness, int, 0664);

static int ramp_up_threshold = 6000;
module_param_named(simple_ramp_threshold, ramp_up_threshold, int, 0664);

static int laziness;


static int interactive_governor(struct kgsl_device *device, int load)
{

	int val = 0;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if (load >= GO_HIGHSPEED_LOAD)
	{
		if (pwr->active_pwrlevel > pwr->max_pwrlevel)
		{
			val = -(pwr->active_pwrlevel);
		}
	}
	else if (load >= up_threshold(pwr->active_pwrlevel))
	{
		if (pwr->active_pwrlevel > pwr->max_pwrlevel)
			val = -1;
	}
	else if (load < down_threshold(pwr->active_pwrlevel))
	{
		if (pwr->active_pwrlevel < pwr->min_pwrlevel)
			val = 1;
	}

#ifdef DEBUG
	pr_info("------------------------------------------------");
	pr_info("GPU frequency:\t\t%d\n", 
		pwr->pwrlevels[pwr->active_pwrlevel].gpu_freq/1000000);
	pr_info("load:\t\t\t%d",load);
	pr_info("up_threshold:\t\t%u",
		up_threshold(pwr->active_pwrlevel));
	pr_info("down_threshold:\t\t%u",
		down_threshold(pwr->active_pwrlevel));
	pr_info("pwr->active_pwrlevel:\t%d",pwr->active_pwrlevel);
	pr_info("------------------------------------------------");
	if(gpu_idle){pr_info("GPU IDLE");}
	else{pr_info("GPU BUSY");}
	pr_info("Idle counter:\t\t%d",idle_counter);
	pr_info("------------------------------------------------");

    int val = 0;
    struct kgsl_pwrctrl *pwr = &device->pwrctrl;
    
    /* it's currently busy */
    if (idle_stat < ramp_up_threshold) {
        if (pwr->active_pwrlevel == 0)
            val = 0; /* already maxed, so do nothing */
        else if ((pwr->active_pwrlevel > 0) &&
                 (pwr->active_pwrlevel <= (pwr->num_pwrlevels - 1)))
            val = -1; /* bump up to next pwrlevel */
        /* idle case */
    } else {
        if ((pwr->active_pwrlevel >= 0) &&
            (pwr->active_pwrlevel < (pwr->num_pwrlevels - 1)))
            if (laziness > 0) {
                /* hold off for a while */
                laziness--;
                val = 0; /* don't change anything yet */
            } else {
                val = 1; /* above min, lower it */
                /* reset laziness count */
                laziness = default_laziness;
            }
            else if (pwr->active_pwrlevel == (pwr->num_pwrlevels - 1))
                val = 0; /* already @ min, so do nothing */
    }
    return val;
}

#endif

	return val;
}

static u64 time_stamp;
static unsigned long sum_total_time, sum_busy_time;

static void tz_idle(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{

	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_power_stats stats;
	unsigned long load = 0;
	int val = 0;
	bool idle_calc_ready = false;
	u64 now = ktime_to_ms(ktime_get());

	device->ftbl->power_stats(device, &stats);
	sum_total_time += (unsigned long)stats.total_time;
	sum_busy_time += (unsigned long)stats.busy_time;

	if (time_stamp < now)
	{
		if (sum_busy_time > 0 && sum_total_time > 0)
			load = (100 * sum_busy_time) / sum_total_time;
		else
			load = 0;
		idle_calc_ready = true;
		sum_total_time = sum_busy_time = 0;
		time_stamp = now + SAMPLE_TIME_MS;
		gpu_idle_detection(device, load);
	}

	if (idle_calc_ready)
		val = interactive_governor(device, load);

	if (val)
		kgsl_pwrctrl_pwrlevel_change(device,
					     pwr->active_pwrlevel + val);

    struct kgsl_pwrctrl *pwr = &device->pwrctrl;
    struct tz_priv *priv = pwrscale->priv;
    struct kgsl_power_stats stats;
    int val, idle;
    
    /* In "performance" mode the clock speed always stays
     the same */
    if (priv->governor == TZ_GOVERNOR_PERFORMANCE)
        return;
    
    device->ftbl->power_stats(device, &stats);
    priv->bin.total_time += stats.total_time;
    priv->bin.busy_time += stats.busy_time;
    /* Do not waste CPU cycles running this algorithm if
     * the GPU just started, or if less than FLOOR time
     * has passed since the last run.
     */
    if ((stats.total_time == 0) ||
        (priv->bin.total_time < FLOOR))
        return;
    
    /* If the GPU has stayed in turbo mode for a while, *
     * stop writing out values. */
    if (pwr->active_pwrlevel == 0) {
        if (priv->no_switch_cnt > SWITCH_OFF) {
            priv->skip_cnt++;
            if (priv->skip_cnt > SKIP_COUNTER) {
                priv->no_switch_cnt -= SWITCH_OFF_RESET_TH;
                priv->skip_cnt = 0;
            }
            return;
        }
        priv->no_switch_cnt++;
    } else {
        priv->no_switch_cnt = 0;
    }
    
    idle = priv->bin.total_time - priv->bin.busy_time;
    priv->bin.total_time = 0;
    priv->bin.busy_time = 0;
    idle = (idle > 0) ? idle : 0;
#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
    if (priv->governor == TZ_GOVERNOR_SIMPLE)
        val = simple_governor(device, idle);
    else
        val = __secure_tz_entry(TZ_UPDATE_ID, idle, device->id);
#else
    val = __secure_tz_entry(TZ_UPDATE_ID, idle, device->id);
#endif
    if (val) {
        kgsl_pwrctrl_pwrlevel_change(device,
                                     pwr->active_pwrlevel + val);
        //pr_info("TZ idle stat: %d, TZ PL: %d, TZ out: %d\n",
        //                idle, pwr->active_pwrlevel, val);
    }

}

static void tz_busy(struct kgsl_device *device,
	struct kgsl_pwrscale *pwrscale)
{
	device->on_time = ktime_to_us(ktime_get());
}

static void tz_sleep(struct kgsl_device *device,
	struct kgsl_pwrscale *pwrscale)
{
	time_stamp = ktime_to_ms(ktime_get()) + SAMPLE_TIME_MS;

	gpu_idle = true;
	idle_counter = HISTORY_SIZE;
}

#ifdef CONFIG_MSM_SCM
static int tz_init(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	return 0;
}
#else
static int tz_init(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	return -EINVAL;
}
#endif /* CONFIG_MSM_SCM */

static void tz_close(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
}

struct kgsl_pwrscale_policy kgsl_pwrscale_policy_tz = {
	.name = "trustzone",
	.init = tz_init,
	.busy = tz_busy,
	.idle = tz_idle,
	.sleep = tz_sleep,
	.wake = tz_wake,
	.close = tz_close
};
EXPORT_SYMBOL(kgsl_pwrscale_policy_tz);
