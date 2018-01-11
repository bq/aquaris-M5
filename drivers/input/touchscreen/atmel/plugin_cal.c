/*
 * Atmel maXTouch Touchscreen driver Plug in
 *
 * Copyright (C) 2013 Atmel Co.Ltd
 *
 * Author: Pitter Liao <pitter.liao@atmel.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

/****************************************************************  
	Pitter Liao add for macro for the global platform
		email:  pitter.liao@atmel.com 
		mobile: 13244776877
-----------------------------------------------------------------*/
#define PLUG_CAL_VERSION 0x0003
/*----------------------------------------------------------------
fixed some bugs
0.2
1 improved version 1:  
<a> fetch data from T37
<b> calibration condition
0.1
1 first version of CAL plugin: calibration recovery support
*/

#include "plug.h"
#include <linux/delay.h>

#define CR_FLAG_RESETING				(1<<0)
#define CR_FLAG_CALING					(1<<1)

#define CR_FLAG_RESET					(1<<4)
#define CR_FLAG_CAL						(1<<5)
#define CR_FLAG_RESUME					(1<<6)

#define CR_AUTO_SLEEP					(1<<16)

//more than 16 is high mask
#define CR_FLAG_FUNC_MOV			(1<<24)
#define CR_FLAG_FUNC_SE				(1<<25)
#define CR_FLAG_FUNC_SC				(1<<26)

#define CR_FLAG_CL_MASK_SHIFT			16

#define CR_FLAG_WORKAROUND_HALT			(1<<31)

#define CR_FLAG_MASK_LOW			(0x000000f0)
#define CR_FLAG_MASK_NORMAL		(0x00000f00)
#define CR_FLAG_MASK_FUNC			(0x0f000000)
#define CR_FLAG_MASK				(-1)

#define SC_FLAG_FUNC_PROX				(1<<16)
#define SC_FLAG_FUNC_TOUCH				(1<<17)
#define SC_FLAG_FUNC_HOVER				(1<<28)
#define SC_FLAG_MASK_FUNC			(0x000f0000)

#define INVALID_COUNT_VAL (-1)
struct sf_result{
	int count;
	int good;
	int bad;
	int pend;
	int sleep;
	int retry;
};

struct points_cache{
	struct point pos;
	int cnt;
	unsigned long ticks;
};

#define POINT_CACHE_SHIFT 4
#define POINT_CACHE (1<<POINT_CACHE_SHIFT)
#define POINT_CACHE_MASK  (POINT_CACHE - 1)

struct trace_cache{
	unsigned long flag;
	struct points_cache points[POINT_CACHE];
	int curr;

	int distance;
};

struct mv_observer {
	unsigned long flag;
	struct trace_cache trace[MAX_TRACE_POINTS];
};

struct se_debug_port{
	u8 mu_atchcalst_cnt;
	u8 mu_peak_tch_delta;
	s16 mu_touch_nodes;
	s16 mu_antitouch_nodes;
	u8 mu_cal_cnt;
	u8 mu_cal_state;
	u16 sct_cal_metric;
	u8 sct_cal_cnt;
	u8 sct_cal_state;
	s16 se_cal_metric;
	u8 se_cal_cnt;
	u8 se_cal_state;
	u8 key_atchcalst_cnt;
	u8 key_cal_cnt;
	u8 key_cal_state;
};

struct se_observer {
	unsigned long flag;

	struct sf_result cnt;
};

enum{
	//delta start
	SE,
	SC_TOUCH,
	SC_HOVER,
	SC_PROX,
	NUM_OBJ_TYPE,
	INVALID_SC = NUM_OBJ_TYPE
};

/*
static const char *sc_name[NUM_OBJ_TYPE] = {
	"SE",
	"SC_PROX",
	"SC_TOUCH",
	"SC_HOVER",
};
*/

struct sc_observer {
	unsigned long flag;

	struct sf_result cnt;
};

struct samples{
	unsigned long type;
	struct diagnostic_info command;
	s16 *buf;
	unsigned long check_time;
	int cnt;    //dec when used
	int curr;  //inc when checked
};

struct content_obj{
	struct list_head node;
	struct samples cont;
};

struct sampling_obj{
	struct content_obj data;
	struct list_head sublist;
};

struct rc_observer{
	unsigned long flag;

	unsigned long touch_id_list;

	struct point first[MAX_TRACE_POINTS];

	struct mv_observer mv;
	struct se_observer se;
	struct sc_observer sc; 

	void *obs_buf;

	struct list_head sampling_list;
};

struct mv_config {
	//struct rect area[NUM_CAL_CL_AREA];
	//struct point distance;
	int distance;

	unsigned long interval_sampling;
	unsigned long interval_valid;
};

struct se_config {
	int thld;
	int hysis;

	struct sf_result count;
};

enum{
    SC_NEG_K,  // Not use now
    SC_NEG,	//<= SC_NEG: recognize as negotive delta
    SC_NO,	//-SC_NO ~ SC_NO: recognize as no delta
    SC_POS,	// >= SC_POS: recognize as positive delta
    SC_ACCU,	//accumulate value:  <= SC_NEG  or  >= SC_POS
    SC_ACCU_K,  //accumulate value thld: if less than this value, recal immediately
    NUM_SC_CONDITION
};

struct sc_condition{
	int thld;
	int num;
};

struct sc_config {	
	struct sc_condition cond[NUM_OBJ_TYPE][NUM_SC_CONDITION];

	struct sf_result count;
}; 

enum {
   TICKS_FAST,
   TICKS_NORMAL,
   TICKS_SLOW,
   TICKS_STOP,
   NUM_TICKS,
};

struct rc_config{
	struct mv_config mv;
	struct se_config se;
	struct sc_config sc;

	unsigned long interval_next[NUM_OBJ_TYPE];
	unsigned long interval_ticks[NUM_TICKS];
	int check_count[NUM_OBJ_TYPE];
	int retry_count[NUM_OBJ_TYPE];
};

static struct samples *find_next_sample(struct list_head *head, int check_sign, unsigned long ticks);
static ssize_t rc_mv_show(struct plugin_cal *p, char *buf, size_t count);

static void plugin_cal_hook_t6(struct plugin_cal *p, u8 status)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_observer *obs = p->obs;

	if (status & (MXT_T6_STATUS_RESET|MXT_T6_STATUS_CAL)) {
		dev_dbg2(dev, "CR hook T6 0x%x\n", status);
		
		if (status & MXT_T6_STATUS_CAL) {
			set_and_clr_flag(CR_FLAG_CALING,
				0,&obs->flag);
		}
		
		if (status & MXT_T6_STATUS_RESET) {
			set_and_clr_flag(CR_FLAG_RESETING,
				CR_FLAG_MASK_NORMAL,&obs->flag);
		}
	}else{
		if(test_flag(CR_FLAG_RESETING,&obs->flag))
			set_and_clr_flag(CR_FLAG_RESET,
				CR_FLAG_RESETING,&obs->flag);
		if(test_flag(CR_FLAG_CALING,&obs->flag))
			set_and_clr_flag(CR_FLAG_CAL,
				CR_FLAG_CALING,&obs->flag);

			dev_dbg2(dev, "CR hook T6 end\n");
	}

	dev_info2(dev, "mxt rc flag=0x%lx %x\n",
		 obs->flag, status);
}

#define RC_ACTION_POINT_FIRST_TOUCH				(1<<0)
#define RC_ACTION_POINT_NEXT_TOUCH				(1<<1)
#define RC_ACTION_POINT_RELEASED					(1<<2)

#define RC_SUBFLAG_POINT_INVALID					(1<<0)

static void rc_mv_hook_t9_t100_points(struct plugin_cal *p, int id, int x, int y, unsigned long flag, unsigned long ticks)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct mv_config *mv_cfg = &cfg->mv;
	struct mv_observer *mv_obs = &obs->mv;
	struct trace_cache *tra;
	struct points_cache *pt;
	int idx;

	if (id >= MAX_TRACE_POINTS)
		return;

	tra = &mv_obs->trace[id];
	if (test_flag(RC_ACTION_POINT_FIRST_TOUCH | RC_ACTION_POINT_NEXT_TOUCH,&flag)) {	
		do {
			idx = tra->curr & POINT_CACHE_MASK;
			pt = &tra->points[idx];
			if (!pt->cnt) {
				pt->pos.x = x;
				pt->pos.y = y;
				pt->ticks = ticks;
				pt->cnt = 1;
				dev_dbg(dev, "[mxt]point (%d %d): (%d,%d) cnt %d time %ld #1\n",
					id, idx, pt->pos.x / pt->cnt, pt->pos.y / pt->cnt, pt->cnt, jiffies -pt->ticks);
			}else {
				if (time_after(ticks, pt->ticks + mv_cfg->interval_sampling)) {
					tra->curr++;
					idx = tra->curr & POINT_CACHE_MASK;
					pt = &tra->points[idx];
					pt->cnt = 0;
				}else{
					pt->pos.x += x;
					pt->pos.y += y;
					pt->cnt++;
					dev_dbg(dev, "[mxt]point (%d %d): (%d,%d) cnt %d time %ld #2\n",
						id, idx, pt->pos.x / pt->cnt, pt->pos.y / pt->cnt, pt->cnt, jiffies -pt->ticks);	
				}
			}
		}while(!pt->cnt);
	}

	//set_and_clr_flag(flag,0, &tra->flag);
}

static void rc_se_hook_t9_t100_points(struct plugin_cal *p, int id, int x, int y, unsigned long flag)
{
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct se_config *se_cfg = &cfg->se;
	struct se_observer *se_obs = &obs->se;

	se_obs->cnt.sleep = se_cfg->count.sleep;
}

static void rc_sc_hook_t9_t100_points(struct plugin_cal *p, int id, int x, int y, unsigned long flag)
{
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct sc_config *sc_cfg = &cfg->sc;
	struct sc_observer *sc_obs = &obs->sc;

	sc_obs->cnt.sleep = sc_cfg->count.sleep;
}

static int plugin_cal_hook_t9_t100(struct plugin_cal *p, int id, int x, int y, struct ext_info *in)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_observer *obs = p->obs;
	unsigned long flag = 0;
	unsigned long ticks = jiffies;

	if (!test_flag(CR_FLAG_MASK_FUNC, &obs->flag))
		return 0;

	if (id >= MAX_TRACE_POINTS)
		return 0;

	if (in->status & MXT_T9_T100_DETECT) {
		if (!__test_and_set_bit(id, &obs->touch_id_list)) {
			obs->first[id].x = x;
			obs->first[id].y = y;
			flag = RC_ACTION_POINT_FIRST_TOUCH;
		}else
			flag = RC_ACTION_POINT_NEXT_TOUCH;
	} else {
		if (__test_and_clear_bit(id, &obs->touch_id_list)) {
			flag = RC_ACTION_POINT_RELEASED;
		}
	}

	if (test_flag(CR_FLAG_FUNC_MOV, &obs->flag))
		rc_mv_hook_t9_t100_points(p, id, x, y, flag, ticks);

	if (test_flag(CR_FLAG_FUNC_SE, &obs->flag))
		rc_se_hook_t9_t100_points(p, id, x, y, flag);

	if (test_flag(CR_FLAG_FUNC_SC, &obs->flag))
		rc_sc_hook_t9_t100_points(p, id, x, y, flag);

	dev_dbg(dev,  "[mxt] tch status %x\n",in->status);

	return 0;
}

static int plugin_cal_hook_t9_t100_scraux(struct plugin_cal *p, struct scr_info *in)
{
	return 0;
}

static void plugin_cal_reset_slots_hook(struct plugin_cal *p)
{
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct se_config *se_cfg = &cfg->se;	
	struct sc_config *sc_cfg = &cfg->sc;
	struct sc_observer *sc_obs = &obs->sc;
	struct se_observer *se_obs = &obs->se;
	struct mv_observer *mv_obs = &obs->mv;

	obs->touch_id_list = 0;
	memset(mv_obs, 0, sizeof(*mv_obs));
	memset(&sc_obs->cnt, 0, sizeof(sc_obs->cnt));
	memset(&se_obs->cnt, 0, sizeof(se_obs->cnt));

	se_obs->cnt.sleep = se_cfg->count.sleep;
	sc_obs->cnt.sleep = sc_cfg->count.sleep;
}

static void rc_sampling_recovery(struct plugin_cal *p, int check_sign, unsigned long ticks)
{
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct samples *s;

	do {
		s = find_next_sample(&obs->sampling_list, check_sign, ticks);
		if (s) {
			WARN_ON(s->cnt);
			WARN_ON(s->type >= NUM_OBJ_TYPE);

			if (check_sign != cfg->check_count[s->type]) {
				s->cnt = cfg->check_count[s->type];
				s->curr = 0 /*cfg->retry_count[s->type]*/;
			}
			
			WARN_ON(s->cnt == check_sign);
			if (s->cnt == check_sign)
				break;
		}
	} while(s);
}

static int rc_handle_mv_message(struct plugin_cal *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct mv_config *mv_cfg = &cfg->mv;
	struct mv_observer *mv_obs = &obs->mv;
	struct trace_cache *tra;
	struct points_cache *pt;
	struct point src0,src1,dist;
	int dst0 = 0,dst1;
	int i,j,k;
	int ret = -EINVAL;  //returen 0 if moving
	
	for (i = 0; i <MAX_TRACE_POINTS; i++) {	
		tra = &mv_obs->trace[i];
		if (!tra->curr)
			continue;

		k = tra->curr & POINT_CACHE_MASK;  //last points
		pt = &tra->points[k];
		if (!pt->cnt) {
			dev_err(dev, "trace(%d) %d\n", i, k);
			rc_mv_show(p, NULL, 0);
			WARN_ON(1);
			continue;
		}

		dst0 = 0;
		src0.x = pt->pos.x /pt->cnt;
		src0.y = pt->pos.y /pt->cnt;	
		for (j = 0; j < (tra->curr & POINT_CACHE_MASK); j++) {
			k = (tra->curr - j - 1) & POINT_CACHE_MASK;
			pt = &tra->points[k];
			if (!pt->cnt)
				break;
		
			dev_dbg(dev, "[mxt]point (%d %d): (%d,%d) cnt %d time %ld #3\n",
				i, k, pt->pos.x / pt->cnt, pt->pos.y / pt->cnt, pt->cnt, jiffies -(pt->ticks + mv_cfg->interval_valid));

			if (time_before(jiffies, pt->ticks + mv_cfg->interval_valid)) {
				src1.x = pt->pos.x /pt->cnt;
				src1.y = pt->pos.y /pt->cnt;	

				dist.x = abs(src1.x - src0.x);
				dist.y = abs(src1.y - src0.y);

				dst1 = dist.x * dist.x + dist.y *dist.y;
				if (dst1 > dst0)
					dst0 = dst1;

				dev_info(dev, "trace(%d, %d) (%d %d)-(%d %d) mov %d %d time %ld\n",
					i,j, src0.x,src0.y,src1.x,src1.y,dst0,dst1,jiffies - pt->ticks);
			}
		}

		tra->distance = dst0;
		if (dst0 > mv_cfg->distance) {
			ret = 0;
			break;
		}
	}

	if (ret == 0)
		dev_info2(dev, "trace(%d) mov %d\n",i, dst0);

	return ret;
}

static int rc_handle_se_message(struct plugin_cal *p, const struct samples *s)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct se_config *se_cfg = &cfg->se;
	struct se_observer *se_obs = &obs->se;
	struct se_debug_port *se_dbg;
	int ret = 0;

	dev_dbg(dev, "rc_handle_se_message\n");

	se_dbg = (struct se_debug_port *)s->buf;
	if (se_dbg->se_cal_metric >= se_cfg->thld) {
		se_obs->cnt.bad++;
	}else if(se_dbg->se_cal_metric < se_cfg->hysis) {
		se_obs->cnt.bad = 0;
	}else {
		se_obs->cnt.pend++;
	}

	if (se_obs->cnt.bad/* || se_obs->cnt.pend*/) {
		if (se_obs->cnt.sleep < se_cfg->count.sleep)
			se_obs->cnt.sleep++;
	}else {
		if (se_obs->cnt.sleep > 0)
			se_obs->cnt.sleep--;
	}
		
	dev_info2(dev, "rc_handle_se_message se (%d %d %d) bad %d pend %d sleep %d\n", 
		se_dbg->se_cal_metric,se_dbg->se_cal_cnt,se_dbg->se_cal_state,
		se_obs->cnt.bad, se_obs->cnt.pend,se_obs->cnt.sleep);

	return ret;
}

static int rc_handle_sc_message(struct plugin_cal *p, const struct samples *s)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct sc_config *sc_cfg = &cfg->sc;
	struct sc_observer *sc_obs = &obs->sc;
	struct sc_condition *condi;
 	int val[NUM_SC_CONDITION];
	int i, channel;
	int ret = 0;

	WARN_ON(s->type >= NUM_OBJ_TYPE);

	dev_dbg(dev, "rc_handle_sc_message\n");

	memset(val, 0, sizeof(val));
	condi = sc_cfg->cond[s->type];
	channel = s->command.num >> 1;

	for (i = 0; i < channel; i++) {
		if (s->buf[i] <= condi[SC_NEG].thld) {
			val[SC_NEG]++;
			val[SC_ACCU] += s->buf[i];
		}else if(s->buf[i] >= -condi[SC_NO].thld && s->buf[i] <= condi[SC_NO].thld)
			val[SC_NO]++;
		else if(s->buf[i] > condi[SC_POS].thld) {
			val[SC_ACCU] += s->buf[i];
			val[SC_POS]++;
		}
	}

	if (val[SC_POS] >= condi[SC_POS].num && val[SC_ACCU] > condi[SC_ACCU_K].thld) {
		sc_obs->cnt.bad = 0;
		sc_obs->cnt.pend = 0;
		sc_obs->cnt.retry = 0;
	}else {
		if (val[SC_ACCU] <= condi[SC_ACCU_K].thld) {
			sc_obs->cnt.bad = sc_cfg->count.bad;  //recal immediately
			sc_obs->cnt.retry = 0;
		}else if (val[SC_ACCU] <= condi[SC_ACCU].thld) {
			 if(val[SC_NEG] > condi[SC_ACCU].num)
				sc_obs->cnt.bad++;
			else	if (val[SC_NO] >= condi[SC_NO].num) {
				if (obs->touch_id_list)
					sc_obs->cnt.pend++;
			}
			sc_obs->cnt.retry = 0;
		}else if (val[SC_ACCU] < condi[SC_NO].thld) {
			if (val[SC_NO] >= condi[SC_NO].num) {
				if (obs->touch_id_list)
					sc_obs->cnt.pend++;
			}
			sc_obs->cnt.retry = 0;
		}else {
			if(val[SC_NEG] >= condi[SC_NEG].num) {
				if (obs->touch_id_list)
					sc_obs->cnt.pend++;
				sc_obs->cnt.retry = 0;
			}else {				
				sc_obs->cnt.retry++;

				if (sc_obs->cnt.retry >= sc_cfg->count.retry) {				
					if (sc_obs->cnt.bad) {
						sc_obs->cnt.bad--;
						sc_obs->cnt.pend++;
					}else {
						sc_obs->cnt.pend--;
					}
					sc_obs->cnt.retry = 0;
				}
			}
		}

		if (sc_obs->cnt.bad || sc_obs->cnt.pend)
			ret = -EINVAL;
	}

	if (sc_obs->cnt.bad/* || sc_obs->cnt.pend*/) {
		if (sc_obs->cnt.sleep < sc_cfg->count.sleep)
			sc_obs->cnt.sleep++;
	}else {
		if (sc_obs->cnt.sleep > 0)
			sc_obs->cnt.sleep--;
	}

	sc_obs->cnt.count++;

	//if (ret) {
	//	dev_info2(dev, "rc_handle_sc_message sc(%ld) cnt (accu %d neg %d nor %d pos %d) count %d bad %d pend %d sleep %d retry %d touch(%lx)\n", 
	//		s->type, val[SC_ACCU], val[SC_NEG],	val[SC_NO],val[SC_POS],
	//		sc_obs->cnt.count, sc_obs->cnt.bad, sc_obs->cnt.pend,sc_obs->cnt.sleep,sc_obs->cnt.retry,
	//		obs->touch_id_list);
		print_dec16_buf(KERN_INFO, "s content", s->buf, s->command.num >> 1);
	//}
	return ret;
}

static int rc_handle_sleep(struct plugin_cal *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_observer *obs = p->obs;
	struct se_observer *se_obs = &obs->se;
	struct sc_observer *sc_obs = &obs->sc;
	int ret = 0;

	if (test_flag(CR_FLAG_FUNC_SE, &obs->flag))
		if (se_obs->cnt.sleep)
			ret = -EBUSY;

	if (test_flag(CR_FLAG_FUNC_SC, &obs->flag))
		if (sc_obs->cnt.sleep)
			ret = -EBUSY;

	dev_dbg(dev, "rc_handle_sleep sleep(%d,%d) ret %d\n", se_obs->cnt.sleep, sc_obs->cnt.sleep, ret);
	
	return ret;
}


int rc_aquire_samples(struct plugin_cal *p,  struct samples *s, bool send)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;	
	struct rc_config *cfg = p->cfg;

	struct diagnostic_block cmd_block;
	int ret = 0;

	WARN_ON(s->type >= NUM_OBJ_TYPE);

	memcpy(&cmd_block.diag, &s->command, sizeof(cmd_block.diag));
	cmd_block.interval = 2;
	cmd_block.count = 3;
	cmd_block.curr = s->curr;
	cmd_block.step = abs(cmd_block.diag.page) + 1;
	cmd_block.max_step = cmd_block.step + cfg->retry_count[s->type];
	cmd_block.ext = 0;
	cmd_block.buf = (u8 *)s->buf;

	dev_dbg(dev, "surface aquire sc (cmd %d, page %d, index %d, num %d) step(%d %d %d)interval(%d %d)\n",
		cmd_block.diag.cmd, cmd_block.diag.page, cmd_block.diag.index, cmd_block.diag.num,  
		cmd_block.curr, cmd_block.step,cmd_block.max_step,
		cmd_block.interval, cmd_block.count);

	ret = p->set_diagnostic_command_fast(p->dev, &cmd_block, send);
	if (ret == 0)
		ret = p->get_diagnostic_data_fast(p->dev, &cmd_block);
	else
		s->curr++;
	
	return ret;
}

static int rc_cal_handler(struct plugin_cal *p, const struct samples *s)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct sc_config *sc_cfg = &cfg->sc;
	struct sc_observer *sc_obs = &obs->sc;
	struct se_config *se_cfg = &cfg->se;
	struct se_observer *se_obs = &obs->se;

	int ret = 0;

	if (se_obs->cnt.bad >= se_cfg->count.bad)
		ret = -EINVAL;
	else if (sc_obs->cnt.bad >= sc_cfg->count.bad)
		ret = -EINVAL;
	else if (se_obs->cnt.pend + se_obs->cnt.bad >= se_cfg->count.pend)
		ret = -EINVAL;
	else if (sc_obs->cnt.pend + sc_obs->cnt.bad >= sc_cfg->count.pend)
		ret = -EINVAL;
	else {
		if ((se_obs->cnt.pend + se_obs->cnt.bad >= se_cfg->count.bad) ||
			(sc_obs->cnt.pend + sc_obs->cnt.bad >= sc_cfg->count.bad))
			ret = rc_handle_mv_message(p);
	}

	if (ret == -EINVAL) {
		dev_info(dev, "mxt rc_cal_handler set cal: se(%d %d) sc(%d %d)\n", 
			se_obs->cnt.bad,se_obs->cnt.pend,
			sc_obs->cnt.bad,sc_obs->cnt.pend);
		p->set_t6_cal(p->dev);
	}
	return ret;
}

static void plugin_cal_pre_process_messages(struct plugin_cal *p, unsigned long pl_flag)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_observer *obs = p->obs;
		
	if (test_flag(CR_FLAG_WORKAROUND_HALT,&obs->flag))
		return;

	dev_dbg2(dev, "mxt plugin_cal_pre_process_messages pl_flag=0x%lx flag=0x%lx\n",
		 pl_flag, obs->flag);
}

static long plugin_cal_post_process_messages(struct plugin_cal *p, unsigned long pl_flag)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;	
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct samples *s;
	int ret = 0;

	long ticks = jiffies;
	long interval = MAX_SCHEDULE_TIMEOUT;

	if (test_flag(CR_FLAG_WORKAROUND_HALT,&obs->flag))
		return interval;

	if (test_flag(CR_FLAG_CALING | CR_FLAG_RESETING,&obs->flag))
		return interval;

	dev_dbg(dev, "mxt rc pl_flag=0x%lx flag=0x%lx\n",
		 pl_flag, obs->flag);

	if (test_flag(CR_FLAG_CAL|CR_FLAG_RESET, &obs->flag)) {
		plugin_cal_reset_slots_hook(p);
	}

	s = find_next_sample(&obs->sampling_list, 1, ticks);
	if (!s) {
		rc_sampling_recovery(p, 0, 0);
		s = find_next_sample(&obs->sampling_list, 1, ticks);
	}

	if (s) {
		dev_dbg(dev, "mxt s type %lx cnt %d curr %ds\n", s->type, s->cnt, s->curr);
		WARN_ON(s->type >= NUM_OBJ_TYPE);
		WARN_ON(!s->cnt);
		ret = rc_aquire_samples(p, s, true);
		if (ret == -EAGAIN)
			;
		else if(ret == 0) {
			//print_dec16_buf(KERN_INFO, "s content", s->buf, s->command.num >> 1);
			s->check_time = ticks + cfg->interval_next[s->type];
			s->cnt--;
			if (s->type == SE) {
				ret = rc_handle_se_message(p, s);
			}else if (s->type == SC_TOUCH || s->type == SC_PROX) {
				ret = rc_handle_sc_message(p, s);
			}

			rc_cal_handler(p, s);
		}else/* if(ret == -EBUSY) */{
			s->cnt--;   //aquire failed
		}
	}

	if (ret == -EAGAIN)
		interval = cfg->interval_ticks[TICKS_FAST];
	else {
		ret = rc_handle_sleep(p);
		if (ret == 0) {
			interval = cfg->interval_ticks[TICKS_STOP];
		}else {
			if (s)
				interval = cfg->interval_ticks[TICKS_NORMAL];
			else
				interval = cfg->interval_ticks[TICKS_SLOW];
		}
	}

	clear_flag(CR_FLAG_MASK_LOW,&obs->flag);

	return interval;
}

static ssize_t rc_mv_show(struct plugin_cal *p, char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct mv_config *mv_cfg = &cfg->mv;
	struct mv_observer *mv_obs = &obs->mv;
	struct trace_cache *tra;
	struct points_cache *pt;

 	int offset = 0;
	int i,j;

	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]mv status: Flag=0x%lx\n", mv_obs->flag);

	dev_info(dev, "[mxt]mv distance: %d\n", mv_cfg->distance);

	dev_info(dev, "[mxt]mv interval: %ld %ld\n", 
		mv_cfg->interval_sampling,
		mv_cfg->interval_valid);

	for (i = 0; i < MAX_TRACE_POINTS; i++) {
		tra = &mv_obs->trace[i];
		if (!tra->curr)
			continue;

		dev_info(dev, "[mxt] trace %d curr %d distance %d\n",
			i, tra->curr, tra->distance);
		for (j = 0; j < POINT_CACHE; j++) {
			pt = &tra->points[j];
			if (!pt->cnt)
				continue;

			dev_info(dev, "[mxt]point %d: (%d,%d) cnt %d time %ld\n",
				j, pt->pos.x / pt->cnt, pt->pos.y / pt->cnt, pt->cnt, jiffies -pt->ticks);
		}
	}
	
	return offset;
}

static ssize_t rc_se_show(struct plugin_cal *p, char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct se_config *se_cfg = &cfg->se;
	struct se_observer *se_obs = &obs->se;
 	int offset = 0;

	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]se status: Flag=0x%08lx\n",
		se_obs->flag);

	if (count > 0) {
		offset += scnprintf(buf + offset, count - offset, "SE CFG: thld %d hysis %d bad %d pend %d sleep %d\n",
			se_cfg->thld, se_cfg->hysis,se_cfg->count.bad,se_cfg->count.pend,se_cfg->count.sleep);
	
		offset += scnprintf(buf + offset, count - offset, "SE obs: bad %d pend %d\n",
			se_obs->cnt.bad, se_obs->cnt.pend);
	}

	return offset;
}

static ssize_t rc_sc_show(struct plugin_cal *p, char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_config *cfg = p->cfg;
	struct rc_observer *obs = p->obs;
	struct sc_config *sc_cfg = &cfg->sc;
	struct sc_observer *sc_obs = &obs->sc;
 	int offset = 0;
	int i,j;

	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]se status: Flag=0x%08lx\n",
		sc_obs->flag);

	if (count > 0) {
		for (i = SE; i < NUM_OBJ_TYPE; i++) {
			for (j = SC_NEG; j < NUM_SC_CONDITION; j++) {
				offset += scnprintf(buf + offset, count - offset, "SC[%d-%d]: thld %d num %d\n",
					i, j, sc_cfg->cond[i][j].thld, sc_cfg->cond[i][j].num);
			}
		}
		offset += scnprintf(buf + offset, count - offset, "SC CFG: bad %d pend %d\n",
			sc_cfg->count.bad, sc_cfg->count.pend);
	
		offset += scnprintf(buf + offset, count - offset, "SC obs: bad %d pend %d\n",
			sc_obs->cnt.bad, sc_obs->cnt.pend);
	}


	return offset;
}


ssize_t plugin_cal_show(struct plugin_cal *p, char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_observer *obs = p->obs;
	int offset = 0;
	int i;

	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]cal status: Flag=0x%08lx\n",
		obs->flag);

	dev_info(dev, "[mxt]cal touch list %lx\n",
		obs->touch_id_list);

	for (i = 0; i < MAX_TRACE_POINTS; i++) {
		if (obs->first[i].x || obs->first[i].y)
			dev_info(dev, "[mxt]first[%d]: %d,%d\n",
				i,obs->first[i].x,obs->first[i].y);
	}

	offset += scnprintf(buf + offset, count - offset, "Func %lx\n",
		(obs->flag & CR_FLAG_MASK_FUNC) >> CR_FLAG_CL_MASK_SHIFT);

	offset += rc_se_show(p, buf + offset, count - offset);
	offset += rc_mv_show(p, buf + offset, count - offset);
	offset += rc_sc_show(p, buf + offset, count - offset);

	return offset;
}

int plugin_cal_store(struct plugin_cal *p, const char *buf, size_t count)
{
	printk(KERN_ERR "[mxt]plugin_cal_store: ------------------------- \n");

	if (!p->init)
		return 0;

	return count;
}

static int plugin_cal_debug_show(struct plugin_cal *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_observer *obs = p->obs;
  
	dev_info(dev, "[mxt]PLUG_CAL_VERSION: 0x%x\n",PLUG_CAL_VERSION);

	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]cal status: Flag=0x%08lx\n",
		obs->flag);

	rc_se_show(p, NULL, 0);
	rc_mv_show(p, NULL, 0);
	rc_sc_show(p, NULL, 0); 

	dev_info(dev, "[mxt]\n");

	return 0;
}

static int plugin_cal_debug_store(struct plugin_cal *p, const char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_config *cfg = p->cfg;
	struct rc_observer * obs = p->obs;
	struct se_config *se_cfg = &cfg->se;
	struct mv_config *mv_cfg = &cfg->mv;
	struct sc_config *sc_cfg = &cfg->sc;
	int offset,ofs,ret;
	char name[255];
	int config[10];
	
	dev_info(dev, "[mxt]rc store:%s\n",buf);

	if (!p->init)
		return 0;

	if (count <= 0)
		return 0;

	if (sscanf(buf, "status: Flag=0x%lx\n",
		&obs->flag) > 0) {
		dev_info(dev, "[mxt] OK\n");
	}else if (sscanf(buf, "enable %x\n",
		&config[0]) > 0) {
		config[0] <<= CR_FLAG_CL_MASK_SHIFT;
		config[0]  &= CR_FLAG_MASK_FUNC;
		set_and_clr_flag(config[0], CR_FLAG_MASK_FUNC, &obs->flag);
		dev_info(dev, "[mxt] set func %x\n", config[0]);
	}else{
		if (count > 4 && count < sizeof(name)) {
			ret = sscanf(buf, "%s %n", name, &offset);
			dev_info2(dev, "name %s, offset %d, ret %d\n",name,offset,ret);
			if (ret == 1) {
				if (strncmp(name, "rc", 2) == 0) {
					if (sscanf(buf + offset, "interval_next[%d]: %d%n", &config[0],&config[1],&ofs) == 2) {
						offset += ofs;
						if (config[0] < NUM_OBJ_TYPE)
							cfg->interval_next[config[0]] = config[1];
					}else if (sscanf(buf + offset, "interval_ticks[%d]: %d%n", &config[0],&config[1],&ofs) == 2) {
						offset += ofs;
						if (config[0] < NUM_OBJ_TYPE)
							cfg->interval_ticks[config[0]] = config[1];							
					}else if (sscanf(buf + offset, "count: %d,%d,%d,%d%n", &config[0],&config[1],&config[2],&config[3],&ofs) == 4) {
						offset += ofs;
						cfg->check_count[0] = config[0];
						cfg->check_count[1] = config[1];
						cfg->check_count[2] = config[2];
						cfg->check_count[3] = config[3];
					}else if(sscanf(buf + offset, "thld: %d %d%n", &config[0],&config[1],&ofs) == 2) {
						offset += ofs;
						se_cfg->thld = config[0];
						se_cfg->hysis = config[1];
					}else {
						dev_err(dev, "Unknow rc sc command: %s\n",buf + offset);
					}

				}else if (strncmp(name, "mv", 2) == 0) {
					if (sscanf(buf + offset, "internal: %d %d%n", &config[0],&config[1],&ofs) == 2) {
						mv_cfg->interval_sampling = config[0];
						mv_cfg->interval_valid = config[1];
						offset += ofs;
					}else if(sscanf(buf + offset, "dist: %d,%d%n", &config[0], &config[1],&ofs) == 2) {
						offset += ofs;
						mv_cfg->distance = config[0];
					}else {
						dev_err(dev, "Unknow rc mv command: %s\n",buf + offset);
					}	
				}else if (strncmp(name, "se", 2) == 0) {
					if (sscanf(buf + offset, "count: %d,%d,%d%n", &config[0],&config[1],&config[2],&ofs) == 3) {
						offset += ofs;
						se_cfg->count.bad = config[0];
						se_cfg->count.pend = config[1];
						se_cfg->count.sleep = config[2];
					}else if(sscanf(buf + offset, "thld: %d %d%n", &config[0],&config[1],&ofs) == 2) {
						offset += ofs;
						se_cfg->thld = config[0];
						se_cfg->hysis = config[1];
					}else {
						dev_err(dev, "Unknow rc sc command: %s\n",buf + offset);
					}

				}else if (strncmp(name, "sc", 2) == 0) {
					if(sscanf(buf + offset, "cond[%d-%d]: %d %d%n", &config[0],&config[1],&config[2],&config[3],&ofs) == 4) {
						offset += ofs;
						if (config[0] >= 0 && config[0] < NUM_OBJ_TYPE &&
							config[1] >= 0 && config[1] < NUM_SC_CONDITION) {
							sc_cfg->cond[config[0]][config[1]].thld = config[2];
							sc_cfg->cond[config[0]][config[1]].num = config[3];
						}
					}else if (sscanf(buf + offset, "count: %d,%d,%d,%d,%d,%d%n", &config[0],&config[1],&config[2],&config[3],&config[4],&config[5],&ofs) == 3) {
							offset += ofs;
							sc_cfg->count.count = config[0];
							sc_cfg->count.good = config[1];
							sc_cfg->count.bad = config[2];
							sc_cfg->count.pend = config[3];
							sc_cfg->count.sleep = config[4];
							sc_cfg->count.retry = config[5];
					}else{
						dev_err(dev, "Unknow rc sc command: %s\n",buf + offset);
					}
				}else {
					dev_err(dev, "Unknow rc command: %s\n",buf);
					return -EINVAL;
				}
			} else{
				dev_err(dev, "Unknow parameter, ret %d\n",ret);
			}
		}
	}
	
	return 0;
}

static int init_mov(struct plugin_cal *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct rc_config *cfg = p->cfg;
	struct mv_config *mv_cfg = &cfg->mv;
	struct point dst;

	dst.x =abs (dcfg->max_x >> 4);
	dst.y = abs(dcfg->max_y >> 4);
	
	mv_cfg->distance = dst.x*dst.x + dst.y*dst.y ;

	mv_cfg->interval_sampling = HZ /12;
	mv_cfg->interval_valid = HZ;

	return 0;
}

static void deinit_mov(struct plugin_cal *p)
{

}

static int init_se(struct plugin_cal *p)
{
	struct rc_config *cfg = p->cfg;
	struct se_config *se_cfg = &cfg->se;

	se_cfg->count.sleep = 5;
	se_cfg->thld = 25;
	se_cfg->hysis = 15;
	se_cfg->count.bad = 2;
	se_cfg->count.pend = 3;

	return 0;
}

static void deinit_se(struct plugin_cal *p)
{

}

static struct sc_condition sc_rc_condition[NUM_OBJ_TYPE][NUM_SC_CONDITION] = {
	{	//SE	
		{0, 0},		//SC_NEG_K
		{-15, 5},		//SC_NEG
		{10, 16},		//SC_NO
		{100, 1},		//SC_POS
		{-150,0},		//SC_ACCU
		{-350,1},		//SC_ACCU_K
	},
	{	//SC_TOUCH	
		{0, 0},		//SC_NEG_K
		{-15, 5},		//SC_NEG
		{10, 16},		//SC_NO
		{100, 1},		//SC_POS
		{-150,5},		//SC_ACCU
		{-350,0},		//SC_ACCU_K
	},
	{	//SC_HOVER	
		{0, 0},		//SC_NEG_K
		{-15, 5},		//SC_NEG
		{10, 16},		//SC_NO
		{100, 1},		//SC_POS
		{-150,5},		//SC_ACCU
		{-350,0},		//SC_ACCU_K
	},
	{	//SC_PROX	
		{0, 0},		//SC_NEG_K
		{-15, 5},		//SC_NEG
		{25, 12},		//SC_NO
		{35, 1},		//SC_POS
		{-100,5},		//SC_ACCU
		{-350,0},		//SC_ACCU_K
	},	
};

struct sf_result sc_rc_result = {
	-1,	//count
	-1,	//good
	2,	//bad
	4,	//pend
	5,	//sleep
	2,	//retry
};

static int init_sc(struct plugin_cal *p)
{
	struct rc_config *cfg = p->cfg;
	struct sc_config *sc_cfg = &cfg->sc;

	memcpy(sc_cfg->cond, sc_rc_condition, sizeof(sc_cfg->cond));
	memcpy(&sc_cfg->count, &sc_rc_result, sizeof(sc_cfg->count));

	return 0;
}

static void deinit_sc(struct plugin_cal *p)
{

}

static struct diagnostic_info diag_cmd_list[NUM_OBJ_TYPE] = {
	{MXT_T6_DEBUG_SE, 0, 0, 20},
	{MXT_T6_DEBUG_DELTA_SC, 0, 0, -1},
	{MXT_T6_DEBUG_DELTA_SC, 1, 0, -1},
	{MXT_T6_DEBUG_DELTA_SC, 2, 0, -1},
};

static int sc_check_count[NUM_OBJ_TYPE] = {
	INVALID_COUNT_VAL, 1, INVALID_COUNT_VAL, 1   //never set to ZERO
};

static int sc_retry_count[NUM_OBJ_TYPE] = {
	3, 3, 3, 3 
};

static struct samples *find_next_sample(struct list_head *head, int check_sign, unsigned long ticks)
{
	struct list_head *ghead,*chead,*gnode,*cnode;
	struct sampling_obj *sobj;
	struct content_obj *cobj;
	struct samples *s;

	//printk(KERN_INFO "[mxt] find list %p (sign %d, ticks %ld)\n",head,check_sign,ticks);

	ghead = head;
	list_for_each(gnode,ghead) {
		cobj = container_of(gnode, struct content_obj, node);
		sobj = container_of(cobj, struct sampling_obj, data);
		s = &cobj->cont;

		chead = &sobj->sublist;

		/*
		printk(KERN_INFO "[mxt] search sobj head %p sublist %p(%p,%p)\n",
			gnode,chead, chead->prev, chead->next);
		*/

		if (!list_empty(chead) ) {
			list_for_each(cnode,chead) {
				cobj = container_of(cnode, struct content_obj, node);
				s = &cobj->cont;
				/*
				printk(KERN_INFO "[mxt]#2 new node %p(%p,%p) head %p(%p,%p) %lx\n",
					&cobj->node, cobj->node.prev,cobj->node.next,
					chead,chead->prev,chead->next, s->type);		
				printk(KERN_INFO "[mxt] find list %p: sign %d(%d), ticks %ld(%ld)\n",
					head,check_sign,s->cnt, ticks,s->check_time);
				*/
				if (ticks) {
					if (time_before(ticks, s->check_time))
						continue;
				}
				if ((check_sign ^ s->cnt) >= 0) {
					if (!check_sign || !s->cnt) {
						if (check_sign == s->cnt)
							return s;
					}else
						return s;
				}
			}
		}
	}

	return NULL;
}

static int init_cal_obj_list(struct plugin_cal *p, struct list_head *head, int obj_size)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct rc_observer * obs = p->obs;
	struct sampling_obj *sobj;
	struct content_obj *cobj;
	int i,size,buf_size;
	void * buf;

	int ret = 0;

	INIT_LIST_HEAD(head);

	buf_size = obj_size;
	size = buf_size * NUM_OBJ_TYPE ;
	buf = kzalloc(size,	GFP_KERNEL);
	if (!buf) {
		dev_err(dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto release;
	}
	dev_info2(dev, "allocated CAL buffer at %p, node number %d\n",
			buf,size);
	obs->obs_buf = buf;

	//SE
	sobj = (struct sampling_obj *)create_new_node(head, sizeof(struct sampling_obj));
	if (!sobj) {
		dev_err(dev, "Failed to allocate memory for SE sampling list\n");
		ret = -ENOMEM;
		goto release;
	}
	INIT_LIST_HEAD(&sobj->sublist);
	sobj->data.cont.type = CR_FLAG_FUNC_SE;
	//SE sublist
	cobj = (struct content_obj *)create_new_node(&sobj->sublist, sizeof(struct content_obj));
	if (!cobj) {
		dev_err(dev, "Failed to allocate memory for SE list\n");
		ret = -ENOMEM;
		goto release;
	}
	cobj->cont.type = SE;
	cobj->cont.buf = buf;
	cobj->cont.check_time = jiffies;
	memcpy(&cobj->cont.command, &diag_cmd_list[SE], sizeof(cobj->cont.command));
	buf += buf_size;

	//SC
	sobj = (struct sampling_obj *)create_new_node(head, sizeof(struct sampling_obj));
	if (!sobj) {
		dev_err(dev, "Failed to allocate memory for SC sampling list\n");
		ret = -ENOMEM;
		goto release;
	}
	INIT_LIST_HEAD(&sobj->sublist);
	sobj->data.cont.type = CR_FLAG_FUNC_SC;
	//SC sublist
	for (i = SC_TOUCH; i < NUM_OBJ_TYPE; i++) {
		cobj = (struct content_obj *)create_new_node(&sobj->sublist, sizeof(struct content_obj));
		if (!cobj) {
			dev_err(dev, "Failed to allocate memory for SC %d list\n",
				i);
			ret = -ENOMEM;
			goto release;
		}
		cobj->cont.type = i;
		cobj->cont.buf = buf;
		cobj->cont.check_time = jiffies;		
		memcpy(&cobj->cont.command, &diag_cmd_list[i], sizeof(cobj->cont.command));
		if (dcfg->t113.ctrl == 5) {			
			cobj->cont.command.index = 0;
			cobj->cont.command.num = dcfg->s[SC_AA].y << 1;
		}else {
			cobj->cont.command.index = dcfg->s[SC].y << 1;
			cobj->cont.command.num = dcfg->s[SC_AA].x << 1;
		}
		buf += buf_size;
	}

release:
	return ret;
}

static void deinit_cal_obj_list(struct plugin_cal *p, struct list_head *head)
{
	struct list_head *ghead,*chead,*gnode,*cnode;
	struct sampling_obj *sobj;
	struct content_obj *cobj;

	if (!head)
		return;

	ghead = head;
	list_for_each(gnode,ghead) {

		printk(KERN_INFO "[mxt]#0 %p %p\n",gnode,ghead);

		printk(KERN_INFO "[mxt]#0 head %p(%p,%p)\n",
			ghead,ghead->prev,ghead->next);
	
		cobj = container_of(gnode, struct content_obj, node);
		sobj = container_of(cobj, struct sampling_obj, data);
		list_del(gnode);
		chead = &sobj->sublist;
		list_for_each(cnode,chead) {
			cobj = container_of(cnode, struct content_obj, node);
			list_del(cnode);
			kfree(cobj);
			cnode = chead;
		}
		kfree(sobj);
		gnode = ghead;
	}
}

static int init_rc_object(struct plugin_cal *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;	
	struct rc_config *cfg = p->cfg;
	struct rc_observer * obs = p->obs;
	int ret;

	ret = init_cal_obj_list(p, &obs->sampling_list ,dcfg->s[SC].x + dcfg->s[SC].y);
	if (ret) {
		dev_err(dev, "Failed to init_cal_obj_list %s\n",__func__);
		return -ENOMEM;
	}

	ret = init_mov(p);
	if (ret) {
		dev_err(dev, "Failed to init_mov %s\n",__func__);
		return -ENOMEM;
	}

	ret = init_se(p);
	if (ret) {
		dev_err(dev, "Failed to init_se %s\n",__func__);
		return -ENOMEM;
	}

	ret = init_sc(p);
	if (ret) {
		dev_err(dev, "Failed to init_sc %s\n",__func__);
		return -ENOMEM;
	}

	cfg->interval_next[SE] = HZ * 1;
	cfg->interval_next[SC_PROX] = HZ / 4;
	cfg->interval_next[SC_TOUCH] = HZ / 4;
	cfg->interval_next[SC_HOVER] = HZ / 4;

	cfg->interval_ticks[TICKS_FAST] = HZ / 32;
	cfg->interval_ticks[TICKS_NORMAL] = HZ / 4;
	cfg->interval_ticks[TICKS_SLOW] = HZ;
	cfg->interval_ticks[TICKS_STOP] = MAX_SCHEDULE_TIMEOUT;

	memcpy(&cfg->check_count[0], &sc_check_count[0], sizeof(cfg->check_count));
	memcpy(&cfg->retry_count[0], &sc_retry_count[0], sizeof(cfg->retry_count));

	set_flag(CR_FLAG_FUNC_MOV/*| CR_FLAG_FUNC_SE*/ | CR_FLAG_FUNC_SC, &obs->flag);

	return ret;
}

static int deinit_rc_object(struct plugin_cal *p)
{
	struct rc_observer * obs = p->obs;

	deinit_sc(p);
	deinit_se(p);
	deinit_mov(p);

	if (obs)
		deinit_cal_obj_list(p, &obs->sampling_list);
	return 0;
}

static int plugin_cal_init(struct plugin_cal *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;

	dev_info(dev, "%s: plugin cal version 0x%x\n", 
			__func__,PLUG_CAL_VERSION);

	p->obs = kzalloc(sizeof(struct rc_observer), GFP_KERNEL);
	if (!p->obs) {
		dev_err(dev, "Failed to allocate memory for rc observer\n");
		return -ENOMEM;
	}

	p->cfg = kzalloc(sizeof(struct rc_config), GFP_KERNEL);
	if (!p->cfg) {
		dev_err(dev, "Failed to allocate memory for rc cfg\n");
		kfree(p->obs);
		p->obs = NULL;
		return -ENOMEM;
	}

	return init_rc_object(p);
}

static void plugin_cal_deinit(struct plugin_cal *p)
{
	deinit_rc_object(p);

	if (p->obs) {
		kfree(p->obs);
		p->obs = NULL;
	}
	if (p->cfg) {
		kfree(p->cfg);
		p->cfg = NULL;
	}
}

static struct plugin_cal mxt_plugin_cal_if = 
{
	.init = plugin_cal_init,
	.deinit = plugin_cal_deinit,
	.start = NULL,
	.stop = NULL,
	.hook_t6 = plugin_cal_hook_t6,
	//.hook_t9 = plugin_cal_hook_t9_t100,
	.hook_t100 = plugin_cal_hook_t9_t100,
	.hook_t100_scraux = plugin_cal_hook_t9_t100_scraux,
	.hook_reset_slots = plugin_cal_reset_slots_hook,
	.pre_process = plugin_cal_pre_process_messages,
	.post_process = plugin_cal_post_process_messages,
	.show = plugin_cal_debug_show,
	.store = plugin_cal_debug_store,
};

int plugin_interface_cal_init(struct plugin_cal *p)
{
	memcpy(p, &mxt_plugin_cal_if, sizeof(struct plugin_cal));

	return 0;
}

