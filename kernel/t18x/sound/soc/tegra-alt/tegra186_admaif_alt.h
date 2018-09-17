/*
 * tegra186_admaif_alt.h - Additional defines for T186
 *
 * Copyright (c) 2015 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA186_ADMAIF_ALT_H__
#define __TEGRA186_ADMAIF_ALT_H__

#define TEGRA210_ADMAIF_CHANNEL_COUNT		20
#define TEGRA210_ADMAIF_XBAR_TX_ENABLE					0x500
#define TEGRA210_ADMAIF_GLOBAL_ENABLE					0xd00

#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT	0
#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_MASK	(0x3f << TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT)

#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_SHIFT	8
#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_MASK		(0x3f << TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_SHIFT)

#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_SHIFT	20
#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_MASK	(0x7ff << TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_SHIFT)

#define TEGRA210_ADMAIF_LAST_REG			0xd5f

#define ADMAIF_RX1_FIFO_CTRL_REG_DEFAULT 0x00000300
#define ADMAIF_RX2_FIFO_CTRL_REG_DEFAULT 0x00000304
#define ADMAIF_RX3_FIFO_CTRL_REG_DEFAULT 0x00000308
#define ADMAIF_RX4_FIFO_CTRL_REG_DEFAULT 0x0000030c
#define ADMAIF_RX5_FIFO_CTRL_REG_DEFAULT 0x00000210
#define ADMAIF_RX6_FIFO_CTRL_REG_DEFAULT 0x00000213
#define ADMAIF_RX7_FIFO_CTRL_REG_DEFAULT 0x00000216
#define ADMAIF_RX8_FIFO_CTRL_REG_DEFAULT 0x00000219
#define ADMAIF_RX9_FIFO_CTRL_REG_DEFAULT 0x0000021c
#define ADMAIF_RX10_FIFO_CTRL_REG_DEFAULT 0x0000021f
#define ADMAIF_RX11_FIFO_CTRL_REG_DEFAULT 0x00000222
#define ADMAIF_RX12_FIFO_CTRL_REG_DEFAULT 0x00000225
#define ADMAIF_RX13_FIFO_CTRL_REG_DEFAULT 0x00000228
#define ADMAIF_RX14_FIFO_CTRL_REG_DEFAULT 0x0000022b
#define ADMAIF_RX15_FIFO_CTRL_REG_DEFAULT 0x0000022e
#define ADMAIF_RX16_FIFO_CTRL_REG_DEFAULT 0x00000231
#define ADMAIF_RX17_FIFO_CTRL_REG_DEFAULT 0x00000234
#define ADMAIF_RX18_FIFO_CTRL_REG_DEFAULT 0x00000237
#define ADMAIF_RX19_FIFO_CTRL_REG_DEFAULT 0x0000023a
#define ADMAIF_RX20_FIFO_CTRL_REG_DEFAULT 0x0000023d

#define ADMAIF_TX1_FIFO_CTRL_REG_DEFAULT 0x02000300
#define ADMAIF_TX2_FIFO_CTRL_REG_DEFAULT 0x02000304
#define ADMAIF_TX3_FIFO_CTRL_REG_DEFAULT 0x02000308
#define ADMAIF_TX4_FIFO_CTRL_REG_DEFAULT 0x0200030c
#define ADMAIF_TX5_FIFO_CTRL_REG_DEFAULT 0x01800210
#define ADMAIF_TX6_FIFO_CTRL_REG_DEFAULT 0x01800213
#define ADMAIF_TX7_FIFO_CTRL_REG_DEFAULT 0x01800216
#define ADMAIF_TX8_FIFO_CTRL_REG_DEFAULT 0x01800219
#define ADMAIF_TX9_FIFO_CTRL_REG_DEFAULT 0x0180021c
#define ADMAIF_TX10_FIFO_CTRL_REG_DEFAULT 0x0180021f
#define ADMAIF_TX11_FIFO_CTRL_REG_DEFAULT 0x01800222
#define ADMAIF_TX12_FIFO_CTRL_REG_DEFAULT 0x01800225
#define ADMAIF_TX13_FIFO_CTRL_REG_DEFAULT 0x01800228
#define ADMAIF_TX14_FIFO_CTRL_REG_DEFAULT 0x0180022b
#define ADMAIF_TX15_FIFO_CTRL_REG_DEFAULT 0x0180022e
#define ADMAIF_TX16_FIFO_CTRL_REG_DEFAULT 0x01800231
#define ADMAIF_TX17_FIFO_CTRL_REG_DEFAULT 0x01800234
#define ADMAIF_TX18_FIFO_CTRL_REG_DEFAULT 0x01800237
#define ADMAIF_TX19_FIFO_CTRL_REG_DEFAULT 0x0180023a
#define ADMAIF_TX20_FIFO_CTRL_REG_DEFAULT 0x0180023d

#endif
