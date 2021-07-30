/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __SYN_DMA_REG_H__
#define __SYN_DMA_REG_H__

/*
 * DMA Register offset
 */
#define SYN_DMA_BUS_MODE				0x1000
#define SYN_DMA_TX_POLL_DEMAND				0x1004
#define SYN_DMA_RX_POLL_DEMAND				0x1008
#define SYN_DMA_RX_DESCRIPTOR_LIST_ADDRESS		0x100C
#define SYN_DMA_TX_DESCRIPTOR_LIST_ADDRESS		0x1010
#define SYN_DMA_STATUS					0x1014
#define SYN_DMA_OPERATION_MODE				0x1018
#define SYN_DMA_INT_ENABLE				0x101C
#define SYN_DMA_MISSED_FRAME_AND_BUFF_OVERFLOW_COUNTER	0x1020
#define SYN_DMA_RX_INTERRUPT_WATCHDOG_TIMER		0x1024
#define SYN_DMA_AXI_BUS_MODE				0x1028
#define SYN_DMA_AHB_OR_AXI_STATUS			0x102C

/*
 * SYN_DMA_OPERATION_MODE = 0x0018,		CSR6 - Dma Operation Mode Register
 */
enum syn_dma_operation_mode_reg {
	SYN_DMA_TX_START = 0x00002000,		/* (ST)Start/Stop transmission*/
	SYN_DMA_RX_START = 0x00000002,		/* (SR)Start/Stop reception   */
};

/*
 * SYN_DMA_STATUS = 0x0014,		CSR5 - Dma status Register
 */
enum syn_dma_status_reg {
	SYN_DMA_INT_NORMAL = 0x00010000,		/* (NIS)Normal interrupt summary */
	SYN_DMA_INT_ABNORMAL = 0x00008000,		/* (AIS)Abnormal interrupt summary */
	SYN_DMA_INT_BUS_ERROR = 0x00002000,		/* Fatal bus error (Abnormal) */
	SYN_DMA_INT_RX_STOPPED = 0x00000100,		/* Receive process stopped (Abnormal) */
	SYN_DMA_INT_RX_NO_BUFFER = 0x00000080,		/* RX buffer unavailable (Abnormal) */
	SYN_DMA_INT_RX_COMPLETED = 0x00000040,		/* Completion of frame RX (Normal) */
	SYN_DMA_INT_TX_UNDERFLOW = 0x00000020,		/* Transmit underflow (Abnormal) */
	SYN_DMA_INT_TX_STOPPED = 0x00000002,		/* TX process stopped (Abnormal) */
	SYN_DMA_INT_TX_COMPLETED = 0x00000001,		/* Transmit completed (Normal) */
};

/*
 * Values to initialize DMA registers
 */
enum syn_dma_init_values {
	SYN_DMA_INT_EN = SYN_DMA_INT_NORMAL
		| SYN_DMA_INT_ABNORMAL | SYN_DMA_INT_BUS_ERROR
		| SYN_DMA_INT_RX_NO_BUFFER | SYN_DMA_INT_RX_COMPLETED
		| SYN_DMA_INT_RX_STOPPED | SYN_DMA_INT_TX_UNDERFLOW
		| SYN_DMA_INT_TX_COMPLETED | SYN_DMA_INT_TX_STOPPED,
	SYN_DMA_INT_DISABLE = 0,
};

/*
 * syn_init_tx_desc_base()
 *	Programs the Dma Tx Base address with the starting address of the descriptor ring or chain.
 */
static inline void syn_init_tx_desc_base(struct nss_gmac_hal_dev *nghd, uint32_t tx_desc_dma)
{
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_TX_DESCRIPTOR_LIST_ADDRESS, tx_desc_dma);
}

/*
 * syn_init_rx_desc_base()
 *	Programs the Dma Rx Base address with the starting address of the descriptor ring or chain.
 */
static inline void syn_init_rx_desc_base(struct nss_gmac_hal_dev *nghd, uint32_t rx_desc_dma)
{
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_RX_DESCRIPTOR_LIST_ADDRESS, rx_desc_dma);
}

/*
 * syn_enable_dma_tx()
 *	Enable Rx GMAC operation
 */
static inline void syn_enable_dma_tx(struct nss_gmac_hal_dev *nghd)
{
	uint32_t data;

	data = hal_read_relaxed_reg(nghd->mac_base, SYN_DMA_OPERATION_MODE);
	data |= SYN_DMA_TX_START;
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_OPERATION_MODE, data);
}

/*
 * syn_disable_dma_tx()
 *	Disable Rx GMAC operation
 */
static inline void syn_disable_dma_tx(struct nss_gmac_hal_dev *nghd)
{
	uint32_t data;

	data = hal_read_relaxed_reg(nghd->mac_base, SYN_DMA_OPERATION_MODE);
	data &= ~SYN_DMA_TX_START;
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_OPERATION_MODE, data);
}

/*
 * syn_enable_dma_rx()
 *	Enable Rx GMAC operation
 */
static inline void syn_enable_dma_rx(struct nss_gmac_hal_dev *nghd)
{
	uint32_t data;

	data = hal_read_relaxed_reg(nghd->mac_base, SYN_DMA_OPERATION_MODE);
	data |= SYN_DMA_RX_START;
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_OPERATION_MODE, data);
}

/*
 * syn_disable_dma_rx()
 *	Disable Rx GMAC operation
 */
static inline void syn_disable_dma_rx(struct nss_gmac_hal_dev *nghd)
{
	uint32_t data;

	data = hal_read_relaxed_reg(nghd->mac_base, SYN_DMA_OPERATION_MODE);
	data &= ~SYN_DMA_RX_START;
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_OPERATION_MODE, data);
}

/*
 * syn_enable_dma_interrupt()
 *	Enables all DMA interrupts.
 */
static inline void syn_enable_dma_interrupt(struct nss_gmac_hal_dev *nghd)
{
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_INT_ENABLE, SYN_DMA_INT_EN);
}

/*
 * syn_disable_dma_interrupt()
 *	Disables all DMA interrupts.
 */
static inline void syn_disable_dma_interrupt(struct nss_gmac_hal_dev *nghd)
{
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_INT_ENABLE, SYN_DMA_INT_DISABLE);
}

/*
 * syn_resume_dma_tx
 *	Resumes the DMA Transmission.
 */
static inline void syn_resume_dma_tx(struct nss_gmac_hal_dev *nghd)
{
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_TX_POLL_DEMAND, 0);
}

/*
 * syn_resume_dma_rx
 *	Resumes the DMA Receive.
 */
static inline void syn_resume_dma_rx(struct nss_gmac_hal_dev *nghd)
{
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_RX_POLL_DEMAND, 0);
}

/*
 * syn_clear_dma_status()
 *	Clear all the pending dma interrupts.
 */
static inline void syn_clear_dma_status(struct nss_gmac_hal_dev *nghd)
{
	uint32_t data;

	data = hal_read_relaxed_reg(nghd->mac_base, SYN_DMA_STATUS);
	hal_write_relaxed_reg(nghd->mac_base, SYN_DMA_STATUS, data);
}

/*
 * syn_get_rx_missed
 *	Get Rx missed errors
 */
static inline uint32_t syn_get_rx_missed(struct nss_gmac_hal_dev *nghd)
{
	uint32_t missed_frame_buff_overflow;
	missed_frame_buff_overflow = hal_read_relaxed_reg(nghd->mac_base, SYN_DMA_MISSED_FRAME_AND_BUFF_OVERFLOW_COUNTER);
	return missed_frame_buff_overflow & 0xFFFF;
}

/*
 * syn_get_fifo_overflows
 *	Get FIFO overflows
 */
static inline uint32_t syn_get_fifo_overflows(struct nss_gmac_hal_dev *nghd)
{
	uint32_t missed_frame_buff_overflow;
	missed_frame_buff_overflow = hal_read_relaxed_reg(nghd->mac_base, SYN_DMA_MISSED_FRAME_AND_BUFF_OVERFLOW_COUNTER);
	return (missed_frame_buff_overflow >> 17) & 0x7ff;
}

#endif	// __SYN_DMA_REG_H__
