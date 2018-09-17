/**
 * Copyright (c) 2016-2017, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __IMX274_H__
#define __IMX274_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <media/nvc.h>
#include <media/nvc_image.h>

#define IMX274_IOCTL_SET_MODE			_IOW('o', 1, struct imx274_mode)
#define IMX274_IOCTL_GET_STATUS			_IOR('o', 2, __u8)
#define IMX274_IOCTL_SET_FRAME_LENGTH		_IOW('o', 3, __u32)
#define IMX274_IOCTL_SET_COARSE_TIME		_IOW('o', 4, __u32)
#define IMX274_IOCTL_SET_GAIN			_IOW('o', 5, __u16)
#define IMX274_IOCTL_GET_SENSORDATA		_IOR('o', 6, \
	 struct imx274_sensordata)
#define IMX274_IOCTL_SET_GROUP_HOLD		_IOW('o', 7, struct imx274_ae)
#define IMX274_IOCTL_SET_HDR_COARSE_TIME	_IOW('o', 8, struct imx274_hdr)
#define IMX274_IOCTL_SET_POWER			_IOW('o', 20, __u32)

#define IMX274_FRAME_LENGTH_ADDR_1		(0x30FA) /* VMAX, MSB */
#define IMX274_FRAME_LENGTH_ADDR_2		(0x30F9) /* VMAX */
#define IMX274_FRAME_LENGTH_ADDR_3		(0x30F8) /* VMAX, LSB */
#define IMX274_SVR_REG_MSB			(0x300F) /* SVR */
#define IMX274_SVR_REG_LSB			(0x300E) /* SVR */
#define IMX274_HMAX_REG_MSB			(0x30F7) /* HMAX */
#define IMX274_HMAX_REG_LSB			(0x30F6) /* HMAX */
#define IMX274_COARSE_TIME_ADDR_MSB		(0x300D) /* SHR */
#define IMX274_COARSE_TIME_ADDR_LSB		(0x300C) /* SHR */

#define IMX274_GROUP_HOLD_ADDR			(0x302D) /* REG HOLD */
#define IMX274_ANALOG_GAIN_ADDR_LSB		(0x300A) /* ANALOG GAIN LSB */
#define IMX274_ANALOG_GAIN_ADDR_MSB		(0x300B) /* ANALOG GAIN MSB */

/*
 * struct imx274_mode - imx274 mode structure
 * @xres: Image x resolutionn
 * @yres: Image y resolution
 * @frame_Length: iamge frame length
 * @coarse_time: Iamge coarse time
 * @coarse_time_short: Image short coarse time
 * @gain: Image gain
 * @hdr_en: Image HDR control
 */
struct imx274_mode {
	__u32 xres;
	__u32 yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

/*
 * struct imx274_hdr - Imx274 HDR structure
 * @coarse_time_long: Image long coarse time
 * @coarse_time_short: Image short coarse time
 */
struct imx274_hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

/*
 * struct imx274_ae - Imx274 ae structure
 * @frame_length: Image frame length
 * @frame_length_enable: Enable frame length control
 * @coarse_time: Image coarse time
 * @coarse_time_short: Image short coarse time
 * @coarse_time_enable: Enable coarse time control
 * @gain: Image gain
 * @gain_enable: Enable gain control
 */
struct imx274_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

#ifdef __KERNEL__
/*
 * struct imx274_power_rail - imx274 power rail structure
 * @dvdd: Pointer to dvdd regulator structure
 * @avdd: Pointer to avdd regulator structure
 * @iovdd: Pointer to iovdd regulator structure
 * @ext_reg1: Pointer to ext_reg1 regulator structure
 * @ext_reg2: Pointer to ext_reg2 regulator structure
 * @mclk: Pointer to mclk clk structure
 * @pwdn_gpio: Pwdn gpio
 * @cam1_gpio: Cam1 gpio
 * @reset_gpio: Reset gpio
 * @af_gpio: AF gpio
 */
struct imx274_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *ext_reg1;
	struct regulator *ext_reg2;
	struct clk *mclk;
	unsigned int pwdn_gpio;
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
};

/*
 * struct imx274_platform_data - imx274 platform structure
 * @mclk_name: Output mclk IO name
 * @cam1_gpio:
 * @reset_gpio: Gpio to reset the camera
 * @af_gpio: Gpio to control AF
 * @ext_reg:
 * @power_on: Function pointer to power on the camera
 * @power_off: Function pointer to power off the camera
 */
struct imx274_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
	bool ext_reg;
	int (*power_on)(struct imx274_power_rail *pw);
	int (*power_off)(struct imx274_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /* __IMX274_H__ */
