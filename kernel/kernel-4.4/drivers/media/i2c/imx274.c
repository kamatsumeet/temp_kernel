/*
 * imx274.c - imx274 sensor driver
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/camera_common.h>
#include <media/imx274.h>
#include "imx274_mode_tbls.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

/* differ to max coarse */
#define IMX274_MAX_COARSE_DIFF		(12)
/* minimum gain value */
#define IMX274_MIN_GAIN			(1)
/* maximum gain value */
#define IMX274_MAX_GAIN			(22)
/* gain shift bits */
#define IMX274_GAIN_SHIFT		(8)
/* minimum frame length */
#define IMX274_MIN_FRAME_LENGTH		(0x482)
/* maximum frame length */
#define IMX274_MAX_FRAME_LENGTH		(0x16893)
/* minimum exposure coarse */
#define IMX274_MIN_EXPOSURE_COARSE	(0x0001)
/* maximum exposure coarse */
#define IMX274_MAX_EXPOSURE_COARSE	\
	(IMX274_MAX_FRAME_LENGTH-IMX274_MAX_COARSE_DIFF)

/* default gain value */
#define IMX274_DEFAULT_GAIN		(IMX274_MIN_GAIN)
/* default frame length value */
#define IMX274_DEFAULT_FRAME_LENGTH	(0x11D2)
/* default exposure coarse value */
#define IMX274_DEFAULT_EXPOSURE_COARSE	\
	(IMX274_DEFAULT_FRAME_LENGTH-IMX274_MAX_COARSE_DIFF)

/* default mode */
#define IMX274_DEFAULT_MODE		(IMX274_MODE_3864X2174_RAW10)

/* default image output width */
#define IMX274_DEFAULT_WIDTH		(3864)
/* default image output height */
#define IMX274_DEFAULT_HEIGHT		(2174)
/* default image data format */
#define IMX274_DEFAULT_DATAFMT		(MEDIA_BUS_FMT_SRGGB10_1X10)
/* default output clk frequency for camera */
#define IMX274_DEFAULT_CLK_FREQ		(24000000)

/*
 * struct imx74 - imx274 structure
 * @power: Camera common power rail structure
 * @num_ctrls: The num of V4L2 control
 * @i2c_client: Pointer to I2C client
 * @subdev: Pointer to V4L2 subdevice structure
 * @pad: Media pad structure
 * @group_hold_prev: Group hold status
 * @group_hold_en: Enable/Disable group hold
 * @regmap: Pointer to regmap structure
 * @s_data: Pointer to camera common data structure
 * @p_data: Pointer to camera common pdata structure
 * @ctrls: Pointer to V4L2 control list
 */
struct imx274 {
	struct camera_common_power_rail	power;
	int				num_ctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;

	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

/*
 * struct sensror_regmap_config - sensor regmap config structure
 * @reg_bits: Sensor register address width
 * @val_bits: Sensor register value width
 * @cache_type: Cache type
 * @use_single_rw: Indicate only read or write a single time
 */
static const struct regmap_config sensor_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.cache_type	= REGCACHE_RBTREE,
	.use_single_rw	= true,
};

/*
 * Function declaration
 */
static int imx274_s_ctrl(struct v4l2_ctrl *ctrl);
static int imx274_set_gain(struct imx274 *priv, s32 val);
static int imx274_set_frame_length(struct imx274 *priv, s32 val);
static int imx274_set_coarse_time(struct imx274 *priv, s32 val);

/*
 * imx274 V4L2 control operator
 */
static const struct v4l2_ctrl_ops imx274_ctrl_ops = {
	.s_ctrl			= imx274_s_ctrl,
};

/*
 * V4L2 control configuration list
 * the control Items includes gain, exposure,
 * frame rate, group hold and HDR
 */
static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX274_MIN_GAIN,
		.max = IMX274_MAX_GAIN,
		.def = IMX274_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX274_MIN_FRAME_LENGTH,
		.max = IMX274_MAX_FRAME_LENGTH,
		.def = IMX274_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX274_MIN_EXPOSURE_COARSE,
		.max = IMX274_MAX_EXPOSURE_COARSE,
		.def = IMX274_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME_SHORT,
		.name = "Coarse Time Short",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX274_MIN_EXPOSURE_COARSE,
		.max = IMX274_MAX_EXPOSURE_COARSE,
		.def = IMX274_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &imx274_ctrl_ops,
		.id = V4L2_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
};

/*
 * imx274_calculate_frame_legnth_regs - Function for calculate frame length
 * register value
 * @regs: Pointer to imx274 reg structure
 * @frame_length: Frame length value
 *
 * This is used to calculate the frame length value for frame length register
 */
static inline void imx274_calculate_frame_length_regs(imx274_reg *regs,
						      u32 frame_length)
{
	regs->addr = IMX274_FRAME_LENGTH_ADDR_1;
	regs->val = (frame_length >> 16) & 0x01;
	(regs + 1)->addr = IMX274_FRAME_LENGTH_ADDR_2;
	(regs + 1)->val = (frame_length >> 8) & 0xff;
	(regs + 2)->addr = IMX274_FRAME_LENGTH_ADDR_3;
	(regs + 2)->val = (frame_length) & 0xff;
}

/*
 * imx274_calculate_coarse_time_regs - Function for calculate coarse time
 * register value
 * @regs: Pointer to imx274 reg structure
 * @coarse_time: Coarse time value
 *
 * This is used to get the coarse time value for coarse time register
 */
static inline void imx274_calculate_coarse_time_regs(imx274_reg *regs,
						     u32 coarse_time)
{
	regs->addr = IMX274_COARSE_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0x00ff;
	(regs + 1)->addr = IMX274_COARSE_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0x00ff;
}

/*
 * imx274_calculate_gain_regs - Function for calculate gain
 * register value
 * @regs: Pointer to imx274 reg structure
 * @gain: Gain value
 *
 * This is used to get the gain value for gain register
 */
static inline void imx274_calculate_gain_regs(imx274_reg *regs,
					      u16 gain)
{
	regs->addr = IMX274_ANALOG_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0x07;
	(regs + 1)->addr = IMX274_ANALOG_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

/*
 * imx274_read_reg - Function for reading register value
 * @s_data: Pointer to camera common data structure
 * @addr: Registr address
 * @val: Pointer to register value
 *
 * This function is used to read a register value for imx274
 *
 * Return: 0 on success, errors otherwise
 */
static inline int imx274_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

/*
 * imx274_write_reg - Function for writing register value
 * @s_data: Pointer to camera common data structure
 * @addr: Registr address
 * @val: Register value
 *
 * This function is used to write a register value for imx274
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err = 0;
	struct imx274 *priv = (struct imx274 *)s_data->priv;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

/*
 * regmap_util_write_table_8 - Function for writing register table
 * @priv: Pointer to imx274 structure
 * @table: Table containing register values
 *
 * This is used to write register table into sensor's reg map.
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_write_table(struct imx274 *priv,
				const imx274_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 IMX274_TABLE_WAIT_MS,
					 IMX274_TABLE_END);
}

/*
 * imx274_power_on - Function to power on the camera
 * @s_data: Pointer to camera common data
 *
 * This is used to power on imx274 camera board
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			pr_err("%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 1);

	usleep_range(300, 310);

	pw->state = SWITCH_ON;
	return 0;
}

/*
 * imx274_power_off - Function to power off the camera
 * @s_data: Pointer to camera common data
 *
 * This is used to power off imx274 camera board
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);

	if (priv->pdata->power_off) {
		err = priv->pdata->power_off(pw);
		if (err) {
			pr_err("%s failed.\n", __func__);
			return err;
		} else {
			goto power_off_done;
		}
	}

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

/*
 * imx274_power_put - Function to put power
 * @priv: Pointer to imx274 structure
 *
 * This is used to put power to tegra
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_power_put(struct imx274 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	if (unlikely(!pw))
		return -EFAULT;

	/* because imx274 does not get power from Tegra */
	/* so this function doing nothing */

	return 0;
}

/*
 * imx274_power_get - Function to get power
 * @priv: Pointer to imx274 structure
 *
 * This is used to get power from tegra
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_power_get(struct imx274 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(&priv->i2c_client->dev,
			"unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = priv->pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(&priv->i2c_client->dev,
				"unable to get parent clcok %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;

	}

	pw->state = SWITCH_OFF;
	return err;
}

/*
 * imx274_start_stream - Function for starting stream per mode index
 * @priv: Pointer to imx274 structure
 * @mode: Mode index value
 *
 * This is used to start steam per mode index.
 * mode = 0, start stream for sensor Mode 1: 3864x2174/raw10/60fps
 * mode = 1, start stream for sensor Mode 3: 1932x1094/raw10/60fps
 * mode = 2, start stream for sensor Mode 5: 1288x734/raw10/60fps
 * mode = 3, start stream for sensor Mode 6: 1288x546/raw10/240fps
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_start_stream(struct imx274 *priv, int mode)
{
	int err = 0;

	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_1]);
	if (err)
		return err;

	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_2]);
	if (err)
		return err;

	err = imx274_write_table(priv, mode_table[mode]);
	if (err)
		return err;

	msleep(20);
	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_3]);
	if (err)
		return err;

	msleep(20);
	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_4]);
	if (err)
		return err;

	return 0;
}

/*
 * imx274_s_stream - It is used to start/stop the streaming.
 * @sd: V4L2 Sub device
 * @enable: Flag (True / False)
 *
 * This function controls the start or stop of streaming for the
 * imx274 sensor.
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	struct v4l2_control control;
	int err;

	dev_dbg(&client->dev, "%s++\n", __func__);

	if (!enable)
		return imx274_write_table(priv,
			mode_table[IMX274_MODE_STOP_STREAM]);

	err = imx274_start_stream(priv, s_data->mode);
	if (err)
		goto exit;

	if (s_data->override_enable) {
		/* write list of override regs for the asking frame length, */
		/* coarse integration time, and gain.                       */
		control.id = V4L2_CID_GAIN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx274_set_gain(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error gain override\n", __func__);

		control.id = V4L2_CID_FRAME_LENGTH;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx274_set_frame_length(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error frame length override\n", __func__);

		control.id = V4L2_CID_COARSE_TIME;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx274_set_coarse_time(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error coarse time override\n", __func__);
	}

	return 0;
exit:
	dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

/*
 * imx274_get_fmt - Get the pad format
 * @sd: Pointer to V4L2 Sub device structure
 * @cfg: Pointer to sub device pad information structure
 * @fmt: Pointer to pad level media bus format
 *
 * This function is used to get the pad format information
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

/*
 * imx274_set_fmt - This is used to set the pad format
 * @sd: Pointer to V4L2 Sub device structure
 * @cfg: Pointer to sub device pad information structure
 * @format: Pointer to pad level media bus format
 *
 * This function is used to set the pad format
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

/*
 * imx274_g_input_status - This is used to get input status
 * @sd: Pointer to V4L2 Sub device structure
 * @status: Pointer to status
 *
 * This function is used to get input status
 *
 * Return: 0 on success
 */
static int imx274_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx274 *priv = (struct imx274 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

/*
 * Camera common sensor operations
 */
static struct camera_common_sensor_ops imx274_common_ops = {
	.power_on = imx274_power_on,
	.power_off = imx274_power_off,
	.write_reg = imx274_write_reg,
	.read_reg = imx274_read_reg,
};

/*
 * im274_set_group_hold - Function to hold the sensor register
 * @priv: Pinter to imx274 structure
 *
 * This is used to hold the imx274 sensor register
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_set_group_hold(struct imx274 *priv)
{
	int err;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		err = imx274_write_reg(priv->s_data,
				       IMX274_GROUP_HOLD_ADDR, 0x1);
		if (err)
			goto fail;
		priv->group_hold_prev = 1;
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		err = imx274_write_reg(priv->s_data,
				       IMX274_GROUP_HOLD_ADDR, 0x0);
		if (err)
			goto fail;
		priv->group_hold_prev = 0;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: Group hold control error\n", __func__);
	return err;
}

/*
 * imx274_to_real_gain - Function to translate to real gain
 * @rep: Input value
 * @shift: Shift bits
 *
 * This is used to translate from input value to real gain
 *
 * Return: Real gain value
 */
static u16 imx274_to_real_gain(u32 rep, int shift)
{
	u16 gain;
	int gain_int;
	int gain_dec;
	int min_int = (1 << shift);

	if (rep < IMX274_MIN_GAIN << IMX274_GAIN_SHIFT)
		rep = IMX274_MIN_GAIN << IMX274_GAIN_SHIFT;
	else if (rep > IMX274_MAX_GAIN << IMX274_GAIN_SHIFT)
		rep = IMX274_MAX_GAIN << IMX274_GAIN_SHIFT;

	gain_int = (int)(rep >> shift);
	gain_dec = (int)(rep & ~(0xffff << shift));

	rep = gain_int * min_int + gain_dec;
	rep = 2048 - (2048 * min_int) / rep;

	gain = (u16)rep;

	return gain;
}

/*
 * imx274_set_gain - Function called when setting analog gain
 * @priv: Pointer to device structure
 * @val: Value for gain
 *
 * Set the analog gain based on input value.
 * Range: [1, 22]
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_set_gain(struct imx274 *priv, s32 val)
{
	imx274_reg reg_list[2];
	int err;
	u16 gain;
	int i;

	dev_dbg(&priv->i2c_client->dev, "%s - val = %d\n", __func__, val);

	if (!priv->group_hold_prev)
		imx274_set_group_hold(priv);

	val = val << IMX274_GAIN_SHIFT;
	dev_dbg(&priv->i2c_client->dev, "input gain value: %d\n", val);

	/* translate value, used to be xxxxxxxx.xxxxxxxx format, */
	/* with 00000001.0000000 as gain of 1 */
	gain = imx274_to_real_gain((u32)val, IMX274_GAIN_SHIFT);

	imx274_calculate_gain_regs(reg_list, gain);
	dev_dbg(&priv->i2c_client->dev,
		"%s: gain %04x val: %04x\n", __func__, val, gain);

	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: GAIN control error\n", __func__);
	return err;
}

/*
 * imx274_set_frame_length - Function called when setting frame length
 * @priv: Pointer to device structure
 * @val: Variable for frame length (= VMAX, i.e. vertical drive period length)
 *
 * Set frame length based on input value.
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_set_frame_length(struct imx274 *priv, s32 val)
{
	imx274_reg reg_list[3];
	int err;
	u32 frame_length = 0;
	int i;
	u16 svr;
	u8 reg_val[2];

	dev_dbg(&priv->i2c_client->dev, "%s length = %d\n", __func__, val);

	if (!priv->group_hold_prev)
		imx274_set_group_hold(priv);

	frame_length = (u32)val;

	/* svr */
	err = imx274_read_reg(priv->s_data, IMX274_SVR_REG_LSB, &reg_val[0]);
	err |= imx274_read_reg(priv->s_data, IMX274_SVR_REG_MSB, &reg_val[1]);
	if (err)
		goto fail;
	svr = (reg_val[1] << 8) + reg_val[0];

	frame_length = frame_length / (svr + 1);

	imx274_calculate_frame_length_regs(reg_list, frame_length);
	dev_dbg(&priv->i2c_client->dev,
		"%s: val: %d\n", __func__, frame_length);

	for (i = 0; i < 3; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

/*
 * imx274_get_frame_length - Function for obtaining current frame length
 * @priv: Pointer to device structure
 * @val: Pointer to obainted value
 *
 * frame_length = vmax x (svr + 1), in unit of hmax.
 *
 * Return: 0 on success
 */
static int imx274_get_frame_length(struct imx274 *priv, s32 *val)
{
	int err;
	u16 svr;
	u32 vmax;
	u8 reg_val[3];

	/* svr */
	err = imx274_read_reg(priv->s_data, IMX274_SVR_REG_LSB, &reg_val[0]);
	err |= imx274_read_reg(priv->s_data, IMX274_SVR_REG_MSB, &reg_val[1]);
	if (err)
		goto fail;
	svr = (reg_val[1] << 8) + reg_val[0];

	/* vmax */
	err = imx274_read_reg(priv->s_data, IMX274_FRAME_LENGTH_ADDR_3,
			      &reg_val[0]);
	err |= imx274_read_reg(priv->s_data, IMX274_FRAME_LENGTH_ADDR_2,
			       &reg_val[1]);
	err |= imx274_read_reg(priv->s_data, IMX274_FRAME_LENGTH_ADDR_1,
			       &reg_val[2]);
	if (err)
		goto fail;
	vmax = ((reg_val[2] & 0x07) << 16) + (reg_val[1] << 8) + reg_val[0];

	*val = vmax * (svr + 1);
	return 0;
fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s : Get frame_length error\n", __func__);
	return err;
}

/*
 * imx274_clamp_coarse_time - Function to clamp coarse time
 * @priv: Pointer to imx274 structure
 * @val: Pointer to coarse time value
 *
 * This is used to clamp coarse time for imx274 sensor.
 *
 * Return: 0 one success, errors otherwise
 */
static int imx274_clamp_coarse_time(struct imx274 *priv, s32 *val)
{
	int err;
	s32 frame_length;

	err = imx274_get_frame_length(priv, &frame_length);
	if (err)
		goto fail;

	if (frame_length == 0)
		frame_length = IMX274_MIN_FRAME_LENGTH;

	*val = frame_length - *val; /* convert to raw shr */
	if (*val > frame_length - IMX274_MAX_COARSE_DIFF)
		*val = frame_length - IMX274_MAX_COARSE_DIFF;
	else if (*val < IMX274_MIN_EXPOSURE_COARSE)
		*val = IMX274_MIN_EXPOSURE_COARSE;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s : EXPOSURE control error\n", __func__);
	return err;
}

/*
 * imx274_set_coarse_time - Function called when setting SHR value
 * @priv: Pointer to imx274 structure
 * @val: Value for exposure time in number of line_length, or [HMAX]
 *
 * Set SHR value based on input value.
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_set_coarse_time(struct imx274 *priv, s32 val)
{
	imx274_reg reg_list[2];
	int err;
	s32 coarse_time;
	int i;

	dev_dbg(&priv->i2c_client->dev, "%s coarse time = %d\n", __func__, val);

	if (!priv->group_hold_prev)
		imx274_set_group_hold(priv);

	coarse_time = (s32)val;

	/* convert exposure_time to appropriate SHR value */
	err = imx274_clamp_coarse_time(priv, &coarse_time);
	if (err)
		goto fail;

	/* prepare SHR registers */
	imx274_calculate_coarse_time_regs(reg_list, coarse_time);

	/* write to SHR registers */
	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
				       reg_list[i].val);
		if (err)
			goto fail;
	}

fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s: COARSE_TIME control error\n", __func__);
	return err;
}

/*
 * imx274_verify_streaming - Function to verify sensor streaming
 * @priv: Pointer to imx274 structure
 *
 * This is used to verify imx274 sensor output steaming
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_verify_streaming(struct imx274 *priv)
{
	int err = 0;

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return err;

	err = imx274_s_stream(priv->subdev, true);
	if (err)
		goto error;

error:
	imx274_s_stream(priv->subdev, false);
	camera_common_s_power(priv->subdev, false);

	return err;
}

/*
 * imx274_s_ctrl - Function called for setting V4L2 control operations
 * @ctrl: Pointer to V4L2 control structure
 *
 * This is used to set V4L2 control operations
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx274 *priv =
		container_of(ctrl->handler, struct imx274, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		err = imx274_set_gain(priv, ctrl->val);
		break;
	case V4L2_CID_FRAME_LENGTH:
		err = imx274_set_frame_length(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME:
		err = imx274_set_coarse_time(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME_SHORT:
		err = imx274_set_coarse_time(priv, ctrl->val);
		break;
	case V4L2_CID_GROUP_HOLD:
		if (switch_ctrl_qmenu[ctrl->val] == SWITCH_ON) {
			priv->group_hold_en = true;
		} else {
			priv->group_hold_en = false;
			err = imx274_set_group_hold(priv);
		}
		break;
	case V4L2_CID_HDR_EN:
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

/*
 * imx274_ctrls_init - Function to initialize V4L2 controls
 * @priv: Pointer to imx274 structure
 *
 * This is used to initialize V4L2 controls for imx274 sensor
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_ctrls_init(struct imx274 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
		}
		priv->ctrls[i] = ctrl;
	}

	priv->num_ctrls = num_ctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

/*
 * imx274_open - Function to open camera device
 * @fh: Pointer to V4L2 subdevice structure
 *
 * This function does nothing
 *
 * Return: 0 on success
 */
static int imx274_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

/*
 * Media operations
 */
static const struct v4l2_subdev_video_ops imx274_subdev_video_ops = {
	.s_stream	= imx274_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status	= imx274_g_input_status,
};

static const struct v4l2_subdev_core_ops imx274_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static const struct v4l2_subdev_pad_ops imx274_subdev_pad_ops = {
	.set_fmt	= imx274_set_fmt,
	.get_fmt	= imx274_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size        = camera_common_enum_framesizes,
	.enum_frame_interval    = camera_common_enum_frameintervals,
};

static const struct v4l2_subdev_internal_ops imx274_subdev_internal_ops = {
	.open = imx274_open,
};

static const struct media_entity_operations imx274_media_ops = {
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = v4l2_subdev_link_validate,
#endif
};

static const struct v4l2_subdev_ops imx274_subdev_ops = {
	.core	= &imx274_subdev_core_ops,
	.video	= &imx274_subdev_video_ops,
	.pad	= &imx274_subdev_pad_ops,
};

static const struct of_device_id imx274_of_match[] = {
	{ .compatible = "nvidia,imx274" },
	{ },
};
MODULE_DEVICE_TABLE(of, imx274_of_match);

/*
 * im274_parse_dt - Function to parse device tree
 * @client: Pointer to I2C client structure
 *
 * This is used to parse imx274 device tree
 *
 * Return: Pointer to camera common pdata on success, NULL on error
 */
static struct camera_common_pdata *imx274_parse_dt(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(imx274_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, " Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	err = camera_common_parse_clocks(client, board_priv_pdata);
	if (err) {
		dev_err(&client->dev, "Failed to find clocks\n");
		goto error;
	}

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(&client->dev, "mclk not in DT\n");

	board_priv_pdata->reset_gpio = of_get_named_gpio(node,
			"reset-gpios", 0);

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

/*
 * imx274 probe - Function called for I2C driver
 * @client: Pointer to I2C client structure
 * @id: Pointer to I2C device id structure
 *
 * This is used to probe imx274 sensor
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct imx274 *priv;
	char debugfs_name[10];
	int err;

	dev_dbg(&client->dev, "Probing IMX274 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct imx274) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	priv->pdata = imx274_parse_dt(client);
	if (!priv->pdata) {
		dev_err(&client->dev, " unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops		= &imx274_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->i2c_client		= client;
	common_data->frmfmt		= &imx274_frmfmt[0];
	common_data->colorfmt		= camera_common_find_datafmt(
					  IMX274_DEFAULT_DATAFMT);
	common_data->power		= &priv->power;
	common_data->ctrls		= priv->ctrls;
	common_data->priv		= (void *)priv;
	common_data->numctrls		= ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts		= ARRAY_SIZE(imx274_frmfmt);
	common_data->def_mode		= IMX274_DEFAULT_MODE;
	common_data->def_width		= IMX274_DEFAULT_WIDTH;
	common_data->def_height		= IMX274_DEFAULT_HEIGHT;
	common_data->fmt_width		= common_data->def_width;
	common_data->fmt_height		= common_data->def_height;
	common_data->def_clk_freq	= IMX274_DEFAULT_CLK_FREQ;

	priv->i2c_client		= client;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &client->dev;

	err = imx274_power_get(priv);
	if (err)
		return err;

	err = camera_common_parse_ports(client, common_data);
	if (err) {
		dev_err(&client->dev, "Failed to find port info\n");
		return err;
	}
	sprintf(debugfs_name, "imx274_%c", common_data->csi_port + 'a');
	dev_dbg(&client->dev, "%s: name %s\n", __func__, debugfs_name);

	camera_common_create_debugfs(common_data, debugfs_name);

	v4l2_i2c_subdev_init(priv->subdev, client, &imx274_subdev_ops);

	err = imx274_ctrls_init(priv);
	if (err)
		return err;

	err = imx274_verify_streaming(priv);
	if (err)
		return err;

	priv->subdev->internal_ops = &imx274_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	priv->subdev->entity.ops = &imx274_media_ops;
	err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_dbg(&client->dev, "Detected IMX274 sensor\n");

	return 0;
}

/*
 * imx274_remove - Function called for I2C driver
 * @client: Pointer to I2C client structure
 *
 * This is used to remove imx274 sensor
 *
 * return: 0 one success
 */
static int imx274_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx274 *priv = (struct imx274 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	imx274_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

/*
 * Media related structure
 */
static const struct i2c_device_id imx274_id[] = {
	{ "imx274", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx274_id);

static struct i2c_driver imx274_i2c_driver = {
	.driver = {
		.name = "imx274",
		.owner = THIS_MODULE,
	},
	.probe = imx274_probe,
	.remove = imx274_remove,
	.id_table = imx274_id,
};

module_i2c_driver(imx274_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX274 sensor");
MODULE_AUTHOR("Leopard Imaging, Inc.");
MODULE_LICENSE("GPL v2");
