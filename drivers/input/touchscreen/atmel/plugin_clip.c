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
#define PLUG_CLIP_VERSION 0x0011
/*----------------------------------------------------------------
fixed some bugs
0.11
1 CL/PA/SUP/POS function
0.1
1 first version of clp plugin: CL/PID support
*/

#include "plug.h"
#include <linux/delay.h>

#define CLP_FLAG_RESETING				(1<<0)
#define CLP_FLAG_CALING					(1<<1)

#define CLP_FLAG_RESET					(1<<4)
#define CLP_FLAG_CAL						(1<<5)
#define CLP_FLAG_RESUME					(1<<6)

//more than 16 is high mask
#define CLP_FLAG_FUNC_PALM				(1<<16)
#define CLP_FLAG_FUNC_CLIP				(1<<17)
#define CLP_FLAG_FUNC_SUP				(1<<18)
#define CLP_FLAG_FUNC_POS				(1<<19)

#define CLP_FLAG_CL_MASK_SHIFT			16

#define CLP_FLAG_WORKAROUND_HALT			(1<<31)

#define CLP_FLAG_MASK_LOW			(0x000f0)
#define CLP_FLAG_MASK_NORMAL		(0x00f00)
#define CLP_FLAG_MASK_FUNC			(0xf0000)
#define CLP_FLAG_MASK				(-1)
 
struct cl_observer {
	unsigned long flag;
};

struct pa_observer {
	unsigned long flag;
};

struct sup_observer {
	unsigned long flag;

#define SHIFT_MAX_COUNT_IN_FILTER  3
#define MAX_COUNT_IN_FILTER (1<<SHIFT_MAX_COUNT_IN_FILTER)
	int sum;
	int count;
};

struct pos_observer {
	unsigned long flag;
	struct {
		unsigned long touch_id_list;
		int sum;
	}state[NUM_CLIP_POS_AREA];
};

struct clp_observer{
	unsigned long flag;

	unsigned long touch_id_list;
	struct point first[MAX_TRACE_POINTS];

	struct pa_observer pa;
	struct cl_observer cl[MAX_TRACE_POINTS];
	struct sup_observer sup[MAX_TRACE_POINTS]; 
	struct pos_observer pos;
};

struct cl_config {
	struct rect area[NUM_CLIP_CL_AREA];
	struct point distance;
};

struct pa_config {
	struct rect area[NUM_CLIP_PA_AREA];
	int thld;
	int thld_tch;
	int thld_atch;
	int intthld;
	int numtch;
};

enum {
	RESTICT_EDGE,
	RESTICT_NEAR,
	RESTICT_CENTER,
	NUM_RESTICTS,
};

enum {
	RANGE_THLD,
	RANGE_HYSIS,
	NUM_RANGE
};

struct sup_config {
	struct rect area[NUM_CLIP_SUP_AREA];
	struct point ratio[NUM_CLIP_SUP_AREA];  //width : height for each 
	u8 range[NUM_RANGE];
	u8 range_internal[NUM_RANGE];
}; 

struct pos_config {
	struct rect area[NUM_CLIP_POS_AREA];
};

struct clp_config{
	struct pa_config pa;
	struct cl_config cl;
	struct sup_config sup[NUM_RESTICTS]; 
	struct pos_config pos; 
};

static ssize_t clip_cl_show(struct plugin_clip *p, char *buf, size_t count);

static void plugin_clip_hook_t6(struct plugin_clip *p, u8 status)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_observer *obs = p->obs;

	if (status & (MXT_T6_STATUS_RESET|MXT_T6_STATUS_CAL)) {
		dev_dbg2(dev, "CLP hook T6 0x%x\n", status);
		
		if (status & MXT_T6_STATUS_CAL) {
			set_and_clr_flag(CLP_FLAG_CALING,
				0,&obs->flag);
		}
		
		if (status & MXT_T6_STATUS_RESET) {
			set_and_clr_flag(CLP_FLAG_RESETING,
				CLP_FLAG_MASK_NORMAL,&obs->flag);
		}
	}else{
		if(test_flag(CLP_FLAG_RESETING,&obs->flag))
			set_and_clr_flag(CLP_FLAG_RESET,
				CLP_FLAG_RESETING,&obs->flag);
		if(test_flag(CLP_FLAG_CALING,&obs->flag))
			set_and_clr_flag(CLP_FLAG_CAL,
				CLP_FLAG_CALING,&obs->flag);

			dev_dbg2(dev, "CLP hook T6 end\n");
	}

	dev_info2(dev, "mxt clp flag=0x%lx %x\n",
		 obs->flag, status);
}

#define CL_ACTION_POINT_FIRST_TOUCH				(1<<0)
#define CL_ACTION_POINT_NEXT_TOUCH				(1<<1)
#define CL_ACTION_POINT_RELEASED					(1<<2)

#define CL_SUBFLAG_POINT_INVALID					(1<<0)

/*
static int clip_pa_hook_t9_t100_points(struct plugin_clip *p, int id, int x, int y, u32 package, unsigned long flag)
{
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct pa_config *pa_cfg = &cfg->pa;
	struct pa_observer *pa_obs = &obs->pa;
	u8 area,width,height,internal;
	int area_sum = 0;
	int i,j;
	int ret = 0;

	area = package & 0xff;
	width = 0;
	height = 0;
	internal = 0;

	if (test_flag(CL_ACTION_POINT_FIRST_TOUCH|CL_ACTION_POINT_NEXT_TOUCH,&flag))
		pa_obs->area[id] = area;
	else if(test_flag(CL_ACTION_POINT_RELEASED,&flag)) 
		pa_obs->area[id] = 0;

	for (i = 0; i < MAX_TRACE_POINTS; i++) {
		for (j = 0; j < NUM_CLIP_PA_AREA; j++) {
			if (x >= pa_cfg->area[j].x0 && x <=  pa_cfg->area[j].x1 &&
				y >= pa_cfg->area[j].y0 && y <=  pa_cfg->area[j].y1) {
				area_sum += pa_obs->area[i] ;
			}
		}
	}

	pa_obs->sum = area_sum;
	if (area_sum >= pa_cfg->thld) {
		set_flag(CL_SUBFLAG_POINT_INVALID,&pa_obs->flag);
		ret = -ERANGE;
	}else {
		clear_flag(CL_SUBFLAG_POINT_INVALID,&pa_obs->flag);
	}

	return ret;
}
*/

static int clip_pa_hook_t9_t100_points_scraux(struct plugin_clip *p, struct scr_info *in, unsigned long flag)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct pa_config *pa_cfg = &cfg->pa;
	struct pa_observer *pa_obs = &obs->pa;
	int area;
	int ret = 0;

	area = in->area_tch + in->area_atch;	
	if (in->num_tch >=pa_cfg->numtch && 
		area >= pa_cfg->thld &&
		in->area_tch >= pa_cfg->thld_tch &&
		in->area_atch >= pa_cfg->thld_atch &&
		in->area_inttch >= pa_cfg->intthld) {
		if(!test_and_set_flag(CL_SUBFLAG_POINT_INVALID,&pa_obs->flag))
			ret = -ERANGE;
	}else {
		if (in->num_tch == 0)
			clear_flag(CL_SUBFLAG_POINT_INVALID,&pa_obs->flag);
	}

	dev_dbg(dev,  "[mxt] scr num %d(%d) thld %d(%d) %d(%d) %d(%d) %d(%d) flag=0x%lx ret %d\n",
		in->num_tch , pa_cfg->numtch, 
		area , pa_cfg->thld ,
		in->area_tch , pa_cfg->thld_tch ,
		in->area_atch , pa_cfg->thld_atch ,
		in->area_inttch , pa_cfg->intthld,
		pa_obs->flag,
		ret);

	return ret;
}

static int clip_cl_hook_t9_t100_points(struct plugin_clip *p, int id, int x, int y, unsigned long flag)
{
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct cl_config *cl_cfg = &cfg->cl;
	struct cl_observer *cl_obs = &obs->cl[id];
	struct point distance;
	int i;
	int ret = 0;

	if (test_flag(CL_ACTION_POINT_FIRST_TOUCH,&flag)) {
		for (i = 0; i < NUM_CLIP_CL_AREA; i++) {
			if (x >= cl_cfg->area[i].x0 && x <=  cl_cfg->area[i].x1 &&
				y >= cl_cfg->area[i].y0 && y <=  cl_cfg->area[i].y1) {
				//printk(KERN_INFO "[mxt] inrange\n");
				ret = -ERANGE;
				break;
			}
		}
		if (ret == -ERANGE)
			set_flag(CL_SUBFLAG_POINT_INVALID, &cl_obs->flag);
	}else  if(test_flag(CL_ACTION_POINT_NEXT_TOUCH,&flag)){
		//point out of the rectangle
		if (test_flag(CL_SUBFLAG_POINT_INVALID,&cl_obs->flag)) {
			for (i = 0; i < NUM_CLIP_CL_AREA; i++) {
				if (x >= cl_cfg->area[i].x0 && x <=  cl_cfg->area[i].x1 &&
					y >= cl_cfg->area[i].y0 && y <=  cl_cfg->area[i].y1) {
					ret = -ERANGE;
					break;
				}
			}
			if (ret == 0)
				clear_flag(CL_SUBFLAG_POINT_INVALID, &cl_obs->flag);
		}

		//distance match
		if (test_flag(CL_SUBFLAG_POINT_INVALID,&cl_obs->flag)) {
			if (cl_cfg->distance.x || cl_cfg->distance.y) {
				distance.x = x - obs->first[id].x;
				distance.x = abs(distance.x);
				distance.y = y - obs->first[id].y;
				distance.y = abs(distance.y);

				if (distance.x >= cl_cfg->distance.x &&
					distance.y >= cl_cfg->distance.y) 
					clear_flag(CL_SUBFLAG_POINT_INVALID,&cl_obs->flag);
			}
		}
	}else if(test_flag(CL_ACTION_POINT_RELEASED,&flag)) {
		clear_flag(CL_SUBFLAG_POINT_INVALID,&cl_obs->flag);
	}

	//printk(KERN_INFO "[mxt] flag %lx ret %d\n",cl_obs->flag,ret);

	if (test_flag(CL_SUBFLAG_POINT_INVALID,&cl_obs->flag))
		return -ERANGE;

	return 0;
}

static int clip_sup_hook_t9_t100_points(struct plugin_clip *p, int id, int x, int y, u8 area, unsigned long flag)
{
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct sup_config *sup_cfg = &cfg->sup[0];
	struct sup_observer *sup_obs = &obs->sup[id];
	int i,j;
	int area_avg;
	int ret = 0;

	if (test_flag(CL_ACTION_POINT_FIRST_TOUCH|CL_ACTION_POINT_NEXT_TOUCH,&flag)) {
		sup_obs->sum += area;
		sup_obs->count++;
		if (sup_obs->count)
			area_avg = sup_obs->sum /sup_obs->count;
		else
			area_avg = area;

		for (i = 0; i < NUM_RESTICTS; i++) {
			for (j = 0; j < NUM_CLIP_SUP_AREA; j++) {
				if (x >= sup_cfg[i].area[j].x0 && x <=  sup_cfg[i].area[j].x1 &&
					y >= sup_cfg[i].area[j].y0 && y <=  sup_cfg[i].area[j].y1) {
					if (test_flag(CL_SUBFLAG_POINT_INVALID,&sup_obs->flag)) {
						if (test_flag(CL_ACTION_POINT_NEXT_TOUCH,&flag)) {
							if (area_avg >= sup_cfg[i].range[RANGE_HYSIS])
								ret = -ERANGE;
						}
					}else {
						if (test_flag(CL_ACTION_POINT_FIRST_TOUCH,&flag)) {
							if (area_avg >= sup_cfg[i].range[RANGE_HYSIS] && i < RESTICT_CENTER)
								ret = -ERANGE;
						}else if (test_flag(CL_ACTION_POINT_NEXT_TOUCH,&flag)) {
							if (area_avg >= sup_cfg[i].range[RANGE_THLD] && i < RESTICT_CENTER)
								ret = -ERANGE;
						}
					}

					if (ret == -ERANGE) {
						set_flag(CL_SUBFLAG_POINT_INVALID, &sup_obs->flag);
						break;
					}
				}
			}
		}

		if (ret == 0)
			clear_flag(CL_SUBFLAG_POINT_INVALID, &sup_obs->flag);
	}else if(test_flag(CL_ACTION_POINT_RELEASED,&flag)) {
		sup_obs->sum = 0;
		sup_obs->count = 0;	
		clear_flag(CL_SUBFLAG_POINT_INVALID,&sup_obs->flag);
	}

	if (test_flag(CL_SUBFLAG_POINT_INVALID,&sup_obs->flag))
		ret = -ERANGE;

	return ret;
}

static int clip_pos_hook_t9_t100_points(struct plugin_clip *p, int id, int x, int y, unsigned long flag)
{
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct pos_config *pos_cfg = &cfg->pos;
	struct pos_observer *pos_obs = &obs->pos;
	int i;

	for (i = 0; i < NUM_CLIP_POS_AREA; i++) {
		if (test_flag(CL_ACTION_POINT_FIRST_TOUCH|CL_ACTION_POINT_NEXT_TOUCH,&flag)) {
			if (x >= pos_cfg->area[i].x0 && x <=  pos_cfg->area[i].x1 &&
				y >= pos_cfg->area[i].y0 && y <=  pos_cfg->area[i].y1) {
				if (!test_and_set_bit(id, &pos_obs->state[i].touch_id_list))
					pos_obs->state[i].sum++;
			}else {
				if (test_and_clear_bit(id, &pos_obs->state[i].touch_id_list))
					pos_obs->state[i].sum--;
			}
		}else {
			if (test_and_clear_bit(id, &pos_obs->state[i].touch_id_list))
				pos_obs->state[i].sum--;
		}
	}

	return 0;
}

/*
union vector_4{
	u8 val;
	struct comp {
		int x:4;
		int y:4;
	}comp;
};

static int clip_sup_hook_t9_packed_points(struct plugin_clip *p, int id, int x, int y, u8 *info, unsigned long flag)
{
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct sup_config *sup_cfg = &cfg->sup;
	struct sup_observer *sup_obs = &obs->sup[id];
	int i,j;
	int exp,area,width,height,vector_x,vector_y;
	const union vector_4 *vec;
	int area_avg;
	int ret = 0;

	if (test_flag(CL_ACTION_POINT_FIRST_TOUCH|CL_ACTION_POINT_NEXT_TOUCH,&flag)) {
		exp = (info[MSG_T9_T100_AREAHW0] >> 5) & 0x3;
		area = (info[MSG_T9_T100_AREAHW0] & 0x1f) << exp;
		width = info[MSG_T9_T100_AREAHW1]  >> 4;
		height = info[MSG_T9_T100_AREAHW1]  & 0xf;

		//TouchInputMapper::cookPointerData
		vec = &info[MSG_T9_T100_VEC];

		sup_obs->sum += area;
		sup_obs->count++;
		if (sup_obs->count)
			area_avg = sup_obs->sum /sup_obs->count;
		else
			area_avg = area;

		for (i = 0; i < NUM_RESTICTS; i++) {
			for (j = 0; j < NUM_CLIP_SUP_AREA; j++) {
				if (x >= sup_cfg[i].area[j].x0 && x <=  sup_cfg[i].area[j].x1 &&
					y >= sup_cfg[i].area[j].y0 && y <=  sup_cfg[i].area[j].y1) {
					if (test_flag(CL_SUBFLAG_POINT_INVALID,&sup_obs->flag)) {
						if (test_flag(CL_ACTION_POINT_NEXT_TOUCH,&flag)) {
							if (area_avg >= sup_cfg[i].range_min[i])
								ret = -ERANGE;
						}
					}else {
						if (test_flag(CL_ACTION_POINT_FIRST_TOUCH,&flag)) {
							if (area_avg >= sup_cfg[i].range_min[i] && i < RESTICT_CENTER)
								ret = -ERANGE;
						}else if (test_flag(CL_ACTION_POINT_NEXT_TOUCH,&flag)) {
							if (area_avg >= sup_cfg[i].range_max[i] && i < RESTICT_CENTER)
								ret = -ERANGE;
						}
					}

					if (ret == -ERANGE) {
						set_flag(CL_SUBFLAG_POINT_INVALID, &sup_obs->flag);
						break;
					}
				}
			}
		}

		if (ret == 0)
			clear_flag(CL_SUBFLAG_POINT_INVALID, &sup_obs->flag);
	}else if(test_flag(CL_ACTION_POINT_RELEASED,&flag)) {
		sup_obs->sum = 0;
		sup_obs->count = 0;	
		clear_flag(CL_SUBFLAG_POINT_INVALID,&sup_obs->flag);
	}

	if (test_flag(CL_SUBFLAG_POINT_INVALID,&sup_obs->flag))
		ret = -ERANGE;

	return ret;
}
*/

static int plugin_clip_hook_t9_t100(struct plugin_clip *p, int id, int x, int y, struct ext_info *in)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_observer *obs = p->obs;
	int cl,sup;
	unsigned long flag = 0;

	if (!test_flag(CLP_FLAG_MASK_FUNC, &obs->flag))
		return 0;

	if (id >= MAX_TRACE_POINTS)
		return 0;

	if (in->status & MXT_T9_T100_DETECT) {	
		if (!__test_and_set_bit(id, &obs->touch_id_list)) {
			obs->first[id].x = x;
			obs->first[id].y = y;
			flag = CL_ACTION_POINT_FIRST_TOUCH;
		}else
			flag = CL_ACTION_POINT_NEXT_TOUCH;
	} else {
		if (__test_and_clear_bit(id, &obs->touch_id_list)) {
			flag = CL_ACTION_POINT_RELEASED;
		}
	}

	cl = 0;
	if (test_flag(CLP_FLAG_FUNC_CLIP, &obs->flag)) {
		cl = clip_cl_hook_t9_t100_points(p, id, x, y, flag);
		if (cl ==  -ERANGE) {
			in->status &= ~MXT_T9_T100_DETECT;
		}
	}

	sup = 0;
	if (test_flag(CLP_FLAG_FUNC_SUP, &obs->flag)) {
		sup = clip_sup_hook_t9_t100_points(p, id, x, y, in->area, flag);
		if (sup ==  -ERANGE) {
			in->status &= ~MXT_T9_T100_DETECT;
		}
	}

	if (test_flag(CLP_FLAG_FUNC_POS, &obs->flag)) {
		clip_pos_hook_t9_t100_points(p, id, x, y, flag);
	}

	dev_dbg(dev,  "[mxt] tch status %x\n",in->status);

	if (cl || sup)
		return -ERANGE;

	return 0;
}

static int plugin_clip_hook_t9_t100_scraux(struct plugin_clip *p, struct scr_info *in)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_observer *obs = p->obs;
	int pa;
	unsigned long flag = 0;

	if (!test_flag(CLP_FLAG_MASK_FUNC, &obs->flag))
		return 0;

	pa = 0;
	if (test_flag(CLP_FLAG_FUNC_PALM, &obs->flag)) {
		pa = clip_pa_hook_t9_t100_points_scraux(p, in, flag);
		if (pa ==  -ERANGE)
			in->status |= MXT_SCRAUX_STS_SUP;
		/*   //don't change the flag if sup already set
		else
			in->status &= ~MXT_SCRAUX_STS_SUP;
		*/
	}

	dev_dbg(dev,  "[mxt] scr status %x pa %d\n",in->status, pa);

	if (pa)
		return -ERANGE;

	return 0;
}

static void plugin_clip_reset_slots_hook(struct plugin_clip *p)
{
	struct clp_observer *obs = p->obs;

	memset(&obs->touch_id_list, 0, sizeof(*obs) - offsetof(struct clp_observer,touch_id_list));  //clear data after touch_id_list
}

/*
static void clp_cl_reset_param(struct plugin_clip *p, int idx,unsigned long flag)
{
	struct clp_config *cfg = p->cfg;
	struct cl_config *cl_cfg = &cfg->cl;

	if (idx > 0 && idx < NUM_CLIP_CL_AREA)
		memset(&cl_cfg->area[idx], 0, sizeof(cl_cfg->area[idx]));
	else
		memset(&cl_cfg->area[0], 0, sizeof(cl_cfg->area));
}
*/

static ssize_t clip_pa_show(struct plugin_clip *p, char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct pa_config *pa_cfg = &cfg->pa;
	struct pa_observer *pa_obs = &obs->pa;
 	int offset = 0;
	int i;

	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]pa status: Flag=0x%08lx\n",
		pa_obs->flag);

	if (count > 0) {
		for (i = 0; i < NUM_CLIP_PA_AREA; i++) {
			offset += scnprintf(buf + offset, count - offset, "PA area[%d]: %d,%d %d,%d\n", i,
				pa_cfg->area[i].x0,pa_cfg->area[i].y0,
				pa_cfg->area[i].x1,pa_cfg->area[i].y1);
		}

		offset += scnprintf(buf + offset, count - offset, "PA thld: %d %d %d %d\n",
			pa_cfg->thld,pa_cfg->thld_tch,pa_cfg->thld_atch,pa_cfg->intthld);

		offset += scnprintf(buf + offset, count - offset, "numtch: %d\n",
			pa_cfg->numtch);

		offset += scnprintf(buf + offset, count - offset, "PA invalid: ");
		if (test_flag(CL_SUBFLAG_POINT_INVALID, &pa_obs->flag))
			offset += scnprintf(buf + offset, count - offset, "%d ", i);
		offset += scnprintf(buf + offset, count - offset, "\n");
	}

	return offset;
}

static ssize_t clip_cl_show(struct plugin_clip *p, char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct cl_config *cl_cfg = &cfg->cl;
	struct cl_observer *cl_obs = &obs->cl[0];
 	int offset = 0;
	int i;

	if (!p->init)
		return 0;

	for (i = 0; i < MAX_TRACE_POINTS; i++) {
		dev_info(dev, "[mxt]cl status[%d]: Flag=0x%08lx\n",
			i, cl_obs[i].flag);
	}
	
	if (count > 0) {
		for (i = 0; i < NUM_CLIP_CL_AREA; i++) {
			offset += scnprintf(buf + offset, count - offset, "CL area[%d]: %d,%d %d,%d\n", i,
				cl_cfg->area[i].x0,cl_cfg->area[i].y0,
				cl_cfg->area[i].x1,cl_cfg->area[i].y1);
		}

		offset += scnprintf(buf + offset, count - offset, "CL dist: %d,%d\n",
			cl_cfg->distance.x,cl_cfg->distance.y);

		offset += scnprintf(buf + offset, count - offset, "CL invalid: ");
		for (i = 0; i < MAX_TRACE_POINTS; i++) {
			if (test_flag(CL_SUBFLAG_POINT_INVALID, &cl_obs[i].flag))
				offset += scnprintf(buf + offset, count - offset, "%d ", i);
		}
		offset += scnprintf(buf + offset, count - offset, "\n");
	}

	return offset;
}

static ssize_t clip_sup_show(struct plugin_clip *p, char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct sup_config *sup_cfg = &cfg->sup[0];
	struct sup_observer *sup_obs = &obs->sup[0];
 	int offset = 0;
	int i,j;

	if (!p->init)
		return 0;

	for (i = 0; i < MAX_TRACE_POINTS; i++) {
		dev_info(dev, "[mxt]sup status[%d]: Flag=0x%08lx\n",
			i,sup_obs[i].flag);
	}

	if (count > 0) {
		for (i = 0; i < NUM_RESTICTS; i++) {
			for (j = 0; j< NUM_CLIP_SUP_AREA; j++) {
				offset += scnprintf(buf + offset, count, "SUP area[%d-%d]: %d,%d %d,%d\n", i,j,
					sup_cfg[i].area[j].x0,sup_cfg[i].area[j].y0,
					sup_cfg[i].area[j].x1,sup_cfg[i].area[j].y1);
			}
		}

		for (i = 0; i < NUM_RESTICTS; i++) {
			offset += scnprintf(buf + offset, count, "SUP range[%d]: %d %d\n",i,
				sup_cfg[i].range[RANGE_THLD],sup_cfg[i].range[RANGE_HYSIS]);
		}
		for (i = 0; i < NUM_RESTICTS; i++) {
			offset += scnprintf(buf + offset, count, "SUP range internal[%d]: %d %d\n",i,
				sup_cfg[i].range_internal[RANGE_THLD],sup_cfg[i].range_internal[RANGE_HYSIS]);
		}

		offset += scnprintf(buf + offset, count - offset, "SUP invalid: ");
		for (i = 0; i < MAX_TRACE_POINTS; i++) {
			if (test_flag(CL_SUBFLAG_POINT_INVALID, &sup_obs[i].flag))
				offset += scnprintf(buf + offset, count - offset, "%d ", i);
		}
		offset += scnprintf(buf + offset, count - offset, "\n");
	}

	return offset;
}

static ssize_t clip_pos_show(struct plugin_clip *p, char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_config *cfg = p->cfg;
	struct clp_observer *obs = p->obs;
	struct pos_config *pos_cfg = &cfg->pos;
	struct pos_observer *pos_obs = &obs->pos;
 	int offset = 0;
	int i;

	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]pos status: Flag=0x%08lx\n",
		pos_obs->flag);

	if (count > 0) {
		for (i = 0; i < NUM_CLIP_POS_AREA; i++) {
			offset += scnprintf(buf + offset, count - offset, "POS area[%d]: %d,%d %d,%d\n", i,
				pos_cfg->area[i].x0,pos_cfg->area[i].y0,
				pos_cfg->area[i].x1,pos_cfg->area[i].y1);

			offset += scnprintf(buf + offset, count - offset, "POS sts[%d]: %d,%lx\n",
				i, pos_obs->state[i].sum,pos_obs->state[i].touch_id_list);
		}
	}
	return offset;
}

static void plugin_clip_pre_process_messages(struct plugin_clip *p, unsigned long pl_flag)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_observer *obs = p->obs;
		
	if (test_flag(CLP_FLAG_WORKAROUND_HALT,&obs->flag))
		return;

	dev_dbg2(dev, "mxt plugin_clip_pre_process_messages pl_flag=0x%lx flag=0x%lx\n",
		 pl_flag, obs->flag);
}

static long plugin_clip_post_process_messages(struct plugin_clip *p, unsigned long pl_flag)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_observer *obs = p->obs;
	long interval = MAX_SCHEDULE_TIMEOUT;

	if (test_flag(CLP_FLAG_WORKAROUND_HALT,&obs->flag))
		return interval;

	if (test_flag(CLP_FLAG_RESETING,&obs->flag))
		return interval;

	dev_dbg(dev, "mxt clp pl_flag=0x%lx flag=0x%lx\n",
		 pl_flag, obs->flag);

	if (test_flag(CLP_FLAG_CAL|CLP_FLAG_RESET, &obs->flag)) {
		plugin_clip_reset_slots_hook(p);
	}

	clear_flag(CLP_FLAG_MASK_LOW,&obs->flag);

	return interval;
}

ssize_t plugin_clip_show(struct plugin_clip *p, char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_observer *obs = p->obs;
	int offset = 0;
	int i;

	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]clip status: Flag=0x%08lx\n",
		obs->flag);

	dev_info(dev, "[mxt]clip touch list %lx\n",
		obs->touch_id_list);

	for (i = 0; i < MAX_TRACE_POINTS; i++) {
		if (obs->first[i].x || obs->first[i].y)
			dev_info(dev, "[mxt]first[%d]: %d,%d\n",
				i,obs->first[i].x,obs->first[i].y);
	}

	offset += scnprintf(buf + offset, count - offset, "Func %lx\n",
		(obs->flag & CLP_FLAG_MASK_FUNC) >> CLP_FLAG_CL_MASK_SHIFT);

	offset += clip_pa_show(p, buf + offset, count - offset);
	offset += clip_cl_show(p, buf + offset, count - offset);
	offset += clip_sup_show(p, buf + offset, count - offset);
	offset += clip_pos_show(p, buf + offset, count - offset);

	return offset;
}

int plugin_clip_store(struct plugin_clip *p, const char *buf, size_t count)
{
	printk(KERN_ERR "[mxt]plugin_clip_store: ------------------------- \n");

	if (!p->init)
		return 0;

	return count;
}

ssize_t plugin_clip_tag_show(struct plugin_clip *p, char *buf, size_t count)
{
	struct clp_observer *obs = p->obs;
	struct pos_observer *pos_obs = &obs->pos;

	int offset = 0;
	int i;

	if (!p->init)
		return 0;

	for (i = 0; i < NUM_CLIP_POS_AREA; i++) {
		offset += scnprintf(buf + offset, count - offset, "POS sts[%d]: %d,%lx\n",
			i, pos_obs->state[i].sum,pos_obs->state[i].touch_id_list);
	}

	return offset;
}

static int plugin_clp_debug_show(struct plugin_clip *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_observer *obs = p->obs;
  
	dev_info(dev, "[mxt]PLUG_CLIP_VERSION: 0x%x\n",PLUG_CLIP_VERSION);

	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]clip status: Flag=0x%08lx\n",
		obs->flag);

	clip_pa_show(p, NULL, 0);
	clip_cl_show(p, NULL, 0);
	clip_sup_show(p, NULL, 0); 
	clip_pos_show(p, NULL, 0); 

	dev_info(dev, "[mxt]\n");

	return 0;
}

static int plugin_clp_debug_store(struct plugin_clip *p, const char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_config *cfg = p->cfg;
	struct clp_observer * obs = p->obs;
	struct pa_config *pa_cfg = &cfg->pa;
	struct cl_config *cl_cfg = &cfg->cl;
	struct sup_config *sup_cfg = &cfg->sup[0];
	struct pos_config *pos_cfg = &cfg->pos;
	int offset,ofs,ret;
	char name[255];
	int config[10];
	struct rect area;
	struct point distance;
	
	dev_info(dev, "[mxt]clp store:%s\n",buf);

	if (!p->init)
		return 0;

	if (count <= 0)
		return 0;

	if (sscanf(buf, "status: Flag=0x%lx\n",
		&obs->flag) > 0) {
		dev_info(dev, "[mxt] OK\n");
	}else if (sscanf(buf, "enable %x\n",
		&config[0]) > 0) {
		config[0] <<= CLP_FLAG_CL_MASK_SHIFT;
		config[0]  &= CLP_FLAG_MASK_FUNC;
		set_and_clr_flag(config[0], CLP_FLAG_MASK_FUNC, &obs->flag);
		dev_info(dev, "[mxt] set func %x\n", config[0]);
	}else{
		if (count > 4 && count < sizeof(name)) {
			ret = sscanf(buf, "%s %n", name, &offset);
			dev_info2(dev, "name %s, offset %d, ret %d\n",name,offset,ret);
			if (ret == 1) {
				if (strncmp(name, "pa", 2) == 0) {
					if (sscanf(buf + offset, "area[%d]: %d,%d %d,%d%n", &config[0],&config[1],&config[2],&config[3],&config[4],&ofs) == 5) {
						offset += ofs;
						if (config[0] >= 0 && config[0] < NUM_CLIP_CL_AREA) {
							area.x0 = config[1];
							area.y0 = config[2];
							area.x1 = config[3];
							area.y1 = config[4];

							memcpy(&pa_cfg->area[config[0]], &area, sizeof(struct rect));
						}
					}else if(sscanf(buf + offset, "thld: %d %d %d %d%n", &config[0],&config[1],&config[2],&config[3],&ofs) == 4) {
						offset += ofs;
						pa_cfg->thld = config[0];
						pa_cfg->thld_tch = config[1];
						pa_cfg->thld_atch = config[2];
						pa_cfg->intthld = config[3];
					}else if(sscanf(buf + offset, "numtch: %d%n", &config[0],&ofs) == 1) {
						offset += ofs;
						pa_cfg->numtch = config[0];
					}else {
						dev_err(dev, "Unknow clp sup command: %s\n",buf + offset);
					}
				}else if (strncmp(name, "cl", 2) == 0) {
					if (sscanf(buf + offset, "area[%d]: %d,%d %d,%d%n", &config[0],&config[1],&config[2],&config[3],&config[4],&ofs) == 5) {
						offset += ofs;
						if (config[0] >= 0 && config[0] < NUM_CLIP_CL_AREA) {
							area.x0 = config[1];
							area.y0 = config[2];
							area.x1 = config[3];
							area.y1 = config[4];

							memcpy(&cl_cfg->area[config[0]], &area, sizeof(struct rect));
						}
					}else if(sscanf(buf + offset, "dist: %d,%d%n", &config[0],&config[1],&ofs) == 2) {
						offset += ofs;
						distance.x = config[0];
						distance.y = config[1];
						memcpy(&cl_cfg->distance, &distance, sizeof(struct point));
					}else {
						dev_err(dev, "Unknow clp cl command: %s\n",buf + offset);
					}
				}else if (strncmp(name, "sup", 2) == 0) {
					if (sscanf(buf + offset, "area[%d-%d]: %d,%d %d,%d%n", &config[0],&config[1],&config[2],&config[3],&config[4],&config[5],&ofs) == 6) {
						offset += ofs;
						if (config[0] >= 0 && config[0] < NUM_RESTICTS &&
							config[1] >= 0 && config[1] < NUM_CLIP_CL_AREA) {
							area.x0 = config[2];
							area.y0 = config[3];
							area.x1 = config[4];
							area.y1 = config[5];

							memcpy(&sup_cfg[config[0]].area[config[1]], &area, sizeof(struct rect));
						}
					}else if(sscanf(buf + offset, "range internal[%d]: %d %d%n", &config[0],&config[1],&config[2],&ofs) == 3) {
						offset += ofs;
						if (config[0] >= 0 && config[0] < NUM_RESTICTS) {
							sup_cfg[config[0]].range_internal[RANGE_THLD] = config[1];
							sup_cfg[config[0]].range_internal[RANGE_HYSIS] = config[2];
						}
					}else if(sscanf(buf + offset, "range[%d]: %d %d%n", &config[0],&config[1],&config[2],&ofs) == 3) {
						offset += ofs;
						if (config[0] >= 0 && config[0] < NUM_RESTICTS) {
							sup_cfg[config[0]].range[RANGE_THLD] = config[1];
							sup_cfg[config[0]].range[RANGE_HYSIS] = config[2];
						}
					}else {
						dev_err(dev, "Unknow clp sup command: %s\n",buf + offset);
					}
				}else if (strncmp(name, "pos", 3) == 0) {
					if (sscanf(buf + offset, "area[%d]: %d,%d %d,%d%n", &config[0],&config[1],&config[2],&config[3],&config[4],&ofs) == 5) {
						offset += ofs;
						if (config[0] >= 0 && config[0] < NUM_CLIP_POS_AREA) {
							area.x0 = config[1];
							area.y0 = config[2];
							area.x1 = config[3];
							area.y1 = config[4];
				
							memcpy(&pos_cfg->area[config[0]], &area, sizeof(struct rect));
						}
					}else {
						dev_err(dev, "Unknow clp cl command: %s\n",buf + offset);
					}
				}else {
					dev_err(dev, "Unknow clp command: %s\n",buf);
					return -EINVAL;
				}
			} else{
				dev_err(dev, "Unknow parameter, ret %d\n",ret);
			}
		}
	}
	
	return 0;
}

int pa_area_list[NUM_CLIP_PA_AREA][4] = {
	{
		100,  //left
		50,  //up
		100,  //right
		50   //down
	}
};

static int init_pa(struct plugin_clip *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct clp_config *cfg = p->cfg;
	struct pa_config *pa_cfg = &cfg->pa;

	int i;

	for (i = 0; i < NUM_CLIP_PA_AREA; i++) {
		pa_cfg->area[i].x0 = pa_area_list[i][0];
		pa_cfg->area[i].y0 = pa_area_list[i][1];
		pa_cfg->area[i].x1 = dcfg->max_x -pa_area_list[i][2];
		pa_cfg->area[i].y1 = dcfg->max_y -pa_area_list[i][3];
	}

	pa_cfg->numtch = 1;

	pa_cfg->thld = 160;
	pa_cfg->thld_tch = 80;
	pa_cfg->thld_atch = 40;
	pa_cfg->intthld = 150;

	return 0;
}

static void deinit_pa(struct plugin_clip *p)
{

}


int cl_area_list[NUM_CLIP_CL_AREA] = {
	50,
	25,
	50,
	25
};

static int init_cl(struct plugin_clip *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct clp_config *cfg = p->cfg;
	struct cl_config *cl_cfg = &cfg->cl;

	cl_cfg->area[CL_LEFT].x0 = 0;
	cl_cfg->area[CL_LEFT].y0 = 0;
	cl_cfg->area[CL_LEFT].x1 = cl_area_list[CL_LEFT];
	cl_cfg->area[CL_LEFT].y1 = dcfg->max_y;

	cl_cfg->area[CL_UP].x0 = 0;
	cl_cfg->area[CL_UP].y0 = 0;
	cl_cfg->area[CL_UP].x1 = dcfg->max_x;
	cl_cfg->area[CL_UP].y1 = cl_area_list[CL_UP];

	cl_cfg->area[CL_RIGHT].x0 = dcfg->max_x - cl_area_list[CL_RIGHT];
	cl_cfg->area[CL_RIGHT].y0 = 0;
	cl_cfg->area[CL_RIGHT].x1 = dcfg->max_x;
	cl_cfg->area[CL_RIGHT].y1 = dcfg->max_y;

	cl_cfg->area[CL_DOWN].x0 = 0;
	cl_cfg->area[CL_DOWN].y0 = dcfg->max_y - cl_area_list[CL_DOWN];
	cl_cfg->area[CL_DOWN].x1 = dcfg->max_x;
	cl_cfg->area[CL_DOWN].y1 = dcfg->max_y;

	cl_cfg->distance.x = 0;
	cl_cfg->distance.y = 200;

	return 0;
}

static void deinit_cl(struct plugin_clip *p)
{

}

int sup_area_list[NUM_RESTICTS][NUM_CLIP_CL_AREA] = {
	{50, 50, 50, 50},
	{200, 200, 200, 200},
	{400, 400, 400, 400},
};

u8 sup_range_list[NUM_RESTICTS][NUM_RANGE] = {
	{12,10},
	{15,12},
	{30,20}
};

u8 sup_range_internal_list[NUM_RESTICTS][NUM_RANGE] = {
	{24,20},
	{30,24},
	{60,30}
};

struct point sup_ratio_list[NUM_RESTICTS] = {
	{5,3},
	{3,3},
	{5,2},
};

static int init_sup(struct plugin_clip *p)
{

	const struct mxt_config *dcfg = p->dcfg;
	struct clp_config *cfg = p->cfg;
	struct sup_config *sup_cfg = &cfg->sup[0];
	int i;

	for (i = 0; i < NUM_RESTICTS; i++) {

		sup_cfg[i].area[SUP_LEFT_UP].x0 = 0;
		sup_cfg[i].area[SUP_LEFT_UP].y0 = 0;
		sup_cfg[i].area[SUP_LEFT_UP].x1 = sup_area_list[i][SUP_LEFT_UP];
		sup_cfg[i].area[SUP_LEFT_UP].y1 = dcfg->max_y >> 2;

		sup_cfg[i].area[SUP_RIGHT_UP].x0 = dcfg->max_x - sup_area_list[i][SUP_RIGHT_UP];
		sup_cfg[i].area[SUP_RIGHT_UP].y0 = 0;
		sup_cfg[i].area[SUP_RIGHT_UP].x1 = dcfg->max_x;
		sup_cfg[i].area[SUP_RIGHT_UP].y1 = dcfg->max_y >> 2;

		sup_cfg[i].area[SUP_LEFT_DOWN].x0 = 0;
		sup_cfg[i].area[SUP_LEFT_DOWN].y0 = dcfg->max_y -(dcfg->max_y >> 2);
		sup_cfg[i].area[SUP_LEFT_DOWN].x1 = sup_area_list[i][SUP_LEFT_DOWN];
		sup_cfg[i].area[SUP_LEFT_DOWN].y1 = dcfg->max_y;

		sup_cfg[i].area[SUP_RIGHT_DOWN].x0 = dcfg->max_x - sup_area_list[i][SUP_RIGHT_UP];
		sup_cfg[i].area[SUP_RIGHT_DOWN].y0 = dcfg->max_y -(dcfg->max_y >> 2);
		sup_cfg[i].area[SUP_RIGHT_DOWN].x1 = dcfg->max_x;
		sup_cfg[i].area[SUP_RIGHT_DOWN].y1 = dcfg->max_y;

		memcpy(sup_cfg[i].range, sup_range_list[i], sizeof(sup_cfg[i].range));
		memcpy(sup_cfg[i].range_internal, sup_range_internal_list[i], sizeof(sup_cfg[i].range_internal));
	}

	return 0;
}

static void deinit_sup(struct plugin_clip *p)
{

}

int pos_area_list[NUM_CLIP_POS_AREA] = {
	100,
	100,
};

static int init_pos(struct plugin_clip *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct clp_config *cfg = p->cfg;
	struct pos_config *pos_cfg = &cfg->pos;

	pos_cfg->area[POS_LEFT].x0 = 0;
	pos_cfg->area[POS_LEFT].y0 = 0;
	pos_cfg->area[POS_LEFT].x1 = pos_area_list[POS_LEFT];
	pos_cfg->area[POS_LEFT].y1 = dcfg->max_y;

	pos_cfg->area[POS_RIGHT].x0 = dcfg->max_x - pos_area_list[POS_RIGHT];
	pos_cfg->area[POS_RIGHT].y0 = 0;
	pos_cfg->area[POS_RIGHT].x1 = dcfg->max_x;
	pos_cfg->area[POS_RIGHT].y1 = dcfg->max_y;

	return 0;
}

static void deinit_pos(struct plugin_clip *p)
{

}

static int init_clp_object(struct plugin_clip *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct clp_observer * obs = p->obs;
	int ret;

	ret = init_pa(p);
	if (ret) {
		dev_err(dev, "Failed to init_pa %s\n",__func__);
		return -ENOMEM;
	}

	ret = init_cl(p);
	if (ret) {
		dev_err(dev, "Failed to init_cl %s\n",__func__);
		return -ENOMEM;
	}

	ret = init_sup(p);
	if (ret) {
		dev_err(dev, "Failed to init_sup %s\n",__func__);
		return -ENOMEM;
	}

	ret = init_pos(p);
	if (ret) {
		dev_err(dev, "Failed to init_pos %s\n",__func__);
		return -ENOMEM;
	}

	set_flag(/*CLP_FLAG_FUNC_CLIP|CLP_FLAG_FUNC_SUP|CLP_FLAG_FUNC_PALM|CLP_FLAG_FUNC_POS*/0, &obs->flag);

	return ret;
}

static int deinit_clp_object(struct plugin_clip *p)
{
	deinit_pos(p);
	deinit_sup(p);
	deinit_cl(p);
	deinit_pa(p);

	return 0;
}

static int plugin_clip_init(struct plugin_clip *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;

	dev_info(dev, "%s: plugin clip clp version 0x%x\n", 
			__func__,PLUG_CLIP_VERSION);

	p->obs = kzalloc(sizeof(struct clp_observer), GFP_KERNEL);
	if (!p->obs) {
		dev_err(dev, "Failed to allocate memory for clp observer\n");
		return -ENOMEM;
	}

	p->cfg = kzalloc(sizeof(struct clp_config), GFP_KERNEL);
	if (!p->cfg) {
		dev_err(dev, "Failed to allocate memory for clp cfg\n");
		kfree(p->obs);
		p->obs = NULL;
		return -ENOMEM;
	}

	return init_clp_object(p);
}

static void plugin_clip_deinit(struct plugin_clip *p)
{
	deinit_clp_object(p);

	if (p->obs) {
		kfree(p->obs);
		p->obs = NULL;
	}
	if (p->cfg) {
		kfree(p->cfg);
		p->cfg = NULL;
	}
}

static struct plugin_clip mxt_plugin_clip_if = 
{
	.init = plugin_clip_init,
	.deinit = plugin_clip_deinit,
	.start = NULL,
	.stop = NULL,
	.hook_t6 = plugin_clip_hook_t6,
	//.hook_t9 = plugin_clip_hook_t9_t100,
	.hook_t100 = plugin_clip_hook_t9_t100,
	.hook_t100_scraux = plugin_clip_hook_t9_t100_scraux,
	.hook_reset_slots = plugin_clip_reset_slots_hook,
	.pre_process = plugin_clip_pre_process_messages,
	.post_process = plugin_clip_post_process_messages,
	.show = plugin_clp_debug_show,
	.store = plugin_clp_debug_store,
};

int plugin_interface_clip_init(struct plugin_clip *p)
{
	memcpy(p, &mxt_plugin_clip_if, sizeof(struct plugin_clip));

	return 0;
}

