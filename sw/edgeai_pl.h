#ifndef EDGEAI_PL_H
#define EDGEAI_PL_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * edgeai_pl.h — PL register driver for pl_edgeai_top
 *
 * Maps to edgeai_ctrl_regs.v register layout:
 *   0x00  CTRL          W1S  [0]=inference_start [1]=soft_reset [2]=tensor_clear
 *   0x04  STATUS        RO   [0]=busy [1]=tensor_ready [2]=inference_done [3]=cam_frame_error
 *   0x08  WT_BASE_ADDR  RW   DDR base of weight blob
 *   0x0C  BIAS_BASE_ADDR RW  DDR base of bias blob
 *   0x10  FRAME_BUF_A   RW   Double-buffer frame A
 *   0x14  FRAME_BUF_B   RW   Double-buffer frame B
 *   0x18  TENSOR_DST    RW   DDR destination for 255-channel output tensor
 * ============================================================*/

/* Base address of AXI-Lite GP0 in Zynq PS memory map */
#ifndef EDGEAI_BASEADDR
#define EDGEAI_BASEADDR   0x43C00000UL
#endif

/* Register offsets */
#define EDGEAI_REG_CTRL         0x00U
#define EDGEAI_REG_STATUS       0x04U
#define EDGEAI_REG_WT_BASE      0x08U
#define EDGEAI_REG_BIAS_BASE    0x0CU
#define EDGEAI_REG_FRAME_BUF_A  0x10U
#define EDGEAI_REG_FRAME_BUF_B  0x14U
#define EDGEAI_REG_TENSOR_DST   0x18U

/* CTRL bits */
#define CTRL_INFERENCE_START    (1U << 0)
#define CTRL_SOFT_RESET         (1U << 1)
#define CTRL_TENSOR_CLEAR       (1U << 2)

/* STATUS bits */
#define STATUS_BUSY             (1U << 0)
#define STATUS_TENSOR_READY     (1U << 1)
#define STATUS_INFERENCE_DONE   (1U << 2)
#define STATUS_CAM_FRAME_ERROR  (1U << 3)

/* Timeout for polling loops (inference takes ~ms at 100 MHz) */
#define EDGEAI_POLL_TIMEOUT_MS  5000U

/* ============================================================
 * Public API
 * ============================================================*/

void     edgeai_init(uint32_t base_addr);
void     edgeai_configure(uint32_t wt_base, uint32_t bias_base,
                           uint32_t frame_a, uint32_t frame_b,
                           uint32_t tensor_dst);
void     edgeai_soft_reset(void);
int      edgeai_start_inference(void);   /* returns 0 on success, -1 on timeout */
uint32_t edgeai_read_status(void);
bool     edgeai_is_done(void);
bool     edgeai_tensor_ready(void);
void     edgeai_clear_tensor(void);

/* Low-level register access */
void     edgeai_reg_write(uint32_t offset, uint32_t value);
uint32_t edgeai_reg_read (uint32_t offset);

#endif /* EDGEAI_PL_H */
