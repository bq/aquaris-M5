/*
 *  pa12200001.h - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (c) 2013, All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __PA12200001_H__
#define __PA12200001_H__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>

/*Driver Parameters  */

/*pa12200001 als/ps Default*/  
#define PA12_ALS_TH_HIGH		35000
#define PA12_ALS_TH_LOW			30000
#define PA12_ALS_TH_RANGE		2000

#define PA12_PS_TH_HIGH			40
#define PA12_PS_TH_LOW			25

#define PA12_PS_TH_HIGH2		35//40//60
#define PA12_PS_TH_LOW2			20//25//45


#define PA12_PS_TH_MIN		  	0	// Minimun value
#define PA12_PS_TH_MAX 			255     // 8 bit MAX

#define PA12_PS_TH_BASE_HIGH 		30      
#define PA12_PS_TH_BASE_LOW		20

#define PA12_PS_NEAR_DISTANCE           0       //Near distance 0 cm
#define PA12_PS_FAR_DISTANCE            1       //Far distance 1 cm 

#define PA12_PS_OFFSET_DEFAULT  	5 	// for X-talk cannceling
#define PA12_PS_OFFSET_EXTRA  1
#define PA12_PS_OFFSET_MAX              50
#define PA12_PS_OFFSET_MIN              5
#define PA12_FAST_CAL			0
#define PA12_FAST_CAL_ONCE		1

#define PA12_ALS_GAIN			0 	// 0:125lux 1:1000lux 2:2000lux 3:10000lux 
#define PA12_LED_CURR			0 	// 0:150mA 1:100mA 2:50mA 3:25mA

#define PA12_PS_PRST			2	// 0:1point 1:2points 2:4points 3:8points (for INT)
#define PA12_ALS_PRST			0	// 0:1point 1:2points 2:4points 3:8points (for INT)

#define PA12_PS_SET			1	// 0:ALS interrupt only 1:PS interrupt only 3:BOTH interrupt *no use now
#define PA12_PS_MODE			3	// 0:OFFSET 3:NORMAL

#define PA12_INT_TYPE			1 	// 0:Window type 1:Hysteresis type for Auto Clear flag , if ALS use interrupt mode,should use windows mode 
#define PA12_PS_PERIOD			1	// 2:25 ms 3:50 ms
#define PA12_ALS_PERIOD			0	// 0 ms 


#define ALS_PS_INT		        0 	//gpio_to_irq(xxxx) GPIO Define
#define ALS_POLLING		  	1	// 0:INT Mode 1:Polling Mode
#define PS_POLLING			0	// 0:INT Mode 1:Polling Mode 	

#define ALS_POLLING_DELAY		200	//ms
#define PS_POLLING_DELAY		200     //ms

#define I2C_RETRY_TIMES			3       
#define I2C_RETRY_DELAY			10	//10ms

#define ALS_AVG_ENABLE			0	

#define USE_LIGHT_FEATURE

/*Driver Internel Use only */

#define PA12_I2C_ADDRESS        	0x1E  	//7 bit Address

/*pa12200001 als/ps sensor register map*/
#define REG_CFG0 			0X00  	// ALS_GAIN(D5-4),PS_ON(D1) ALS_ON(D0)
#define REG_CFG1 			0X01  	// LED_CURR(D5-4),PS_PRST(D3-2),ALS_PRST(D1-0)
#define REG_CFG2 			0X02  	// PS_MODE(D7-6),CLEAR(D4),INT_SET(D3-2),PS_INT(D1),ALS_INT(D0)
#define REG_CFG3			0X03  	// INT_TYPE(D6),PS_PERIOD(D5-3),ALS_PERIOD(D2-0)
#define REG_ALS_TL_LSB			0X04  	// ALS Threshold Low LSB
#define REG_ALS_TL_MSB			0X05  	// ALS Threshold Low MSB
#define REG_ALS_TH_LSB			0X06  	// ALS Threshold high LSB
#define REG_ALS_TH_MSB			0X07  	// ALS Threshold high MSB
#define REG_PS_TL			0X08  	// PS Threshold Low
#define REG_PS_TH			0X0A  	// PS Threshold High
#define REG_ALS_DATA_LSB		0X0B  	// ALS DATA
#define REG_ALS_DATA_MSB		0X0C  	// ALS DATA
#define REG_PS_DATA			0X0E  	// PS DATA
#define REG_PS_OFFSET			0X10  	// TBD
#define REG_PS_SET			0X11  	// 0x03

#define ALS_ACTIVE    		0x01
#define PS_ACTIVE    		0x02

#define ALS_INT_ACTIVE    	0x01
#define PS_INT_ACTIVE    	0x02

/*IOCTL Define*/
#define TXC_IOC_MAGIC 't'
#define PA12_IOCTL_PS_ENABLE      	_IOW(TXC_IOC_MAGIC,1,int)
#define PA12_IOCTL_PS_GET_DATA    	_IOR(TXC_IOC_MAGIC,2,int)       
#define PA12_IOCTL_PS_CALIBRATION       _IOR(TXC_IOC_MAGIC,3,int)

#define PA12_IOCTL_ALS_ENABLE	    	_IOW(TXC_IOC_MAGIC,6,int)
#define PA12_IOCTL_ALS_GET_DATA        	_IOR(TXC_IOC_MAGIC,7,int)

#define PAl2_VDD_MIN_UV  2500000
#define PAl2_VDD_MAX_UV  3600000


struct pa12200001_platform_data {
	int (*power_onoff)(int onoff);
	int irq;  /* proximity/light-sensor- external irq*/
	unsigned int ps_det_thld;
	unsigned int ps_hsyt_thld;
	unsigned int als_hsyt_thld;
};

#endif
