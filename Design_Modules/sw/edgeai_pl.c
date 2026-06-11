#include "edgeai_pl.h"
#include <stddef.h>

/* Vitis/Xilinx bare-metal: use volatile pointer for MMIO */
#ifdef XPAR_XILTIMER_H
#  include "xtime_l.h"      /* XTime_GetTime() */
#  define MS_TO_COUNTS(ms)  ((XTime)(ms) * (COUNTS_PER_SECOND / 1000ULL))
#else
/* Fallback: pure iteration counter — calibrated for ~100 MHz */
#  define POLL_ITERS_PER_MS 100000U
#endif

static volatile uint32_t *pl_base = NULL;

/* ============================================================
 * Register access
 * ============================================================*/

void edgeai_reg_write(uint32_t offset, uint32_t value)
{
    pl_base[offset >> 2] = value;
}

uint32_t edgeai_reg_read(uint32_t offset)
{
    return pl_base[offset >> 2];
}

/* ============================================================
 * Public API
 * ============================================================*/

void edgeai_init(uint32_t base_addr)
{
    pl_base = (volatile uint32_t *)(uintptr_t)base_addr;
}

void edgeai_configure(uint32_t wt_base, uint32_t bias_base,
                       uint32_t frame_a, uint32_t frame_b,
                       uint32_t tensor_dst)
{
    edgeai_reg_write(EDGEAI_REG_WT_BASE,     wt_base);
    edgeai_reg_write(EDGEAI_REG_BIAS_BASE,   bias_base);
    edgeai_reg_write(EDGEAI_REG_FRAME_BUF_A, frame_a);
    edgeai_reg_write(EDGEAI_REG_FRAME_BUF_B, frame_b);
    edgeai_reg_write(EDGEAI_REG_TENSOR_DST,  tensor_dst);
}

void edgeai_soft_reset(void)
{
    edgeai_reg_write(EDGEAI_REG_CTRL, CTRL_SOFT_RESET);
    /* reset deasserts automatically — 1-cycle pulse from edgeai_ctrl_regs */
}

int edgeai_start_inference(void)
{
    edgeai_reg_write(EDGEAI_REG_CTRL, CTRL_INFERENCE_START);

#ifdef XPAR_XILTIMER_H
    XTime t_start, t_now;
    XTime_GetTime(&t_start);
    XTime deadline = t_start + MS_TO_COUNTS(EDGEAI_POLL_TIMEOUT_MS);
    while (!(edgeai_reg_read(EDGEAI_REG_STATUS) & STATUS_INFERENCE_DONE)) {
        XTime_GetTime(&t_now);
        if (t_now >= deadline) return -1;
    }
#else
    uint32_t iters = EDGEAI_POLL_TIMEOUT_MS * POLL_ITERS_PER_MS;
    while (iters--) {
        if (edgeai_reg_read(EDGEAI_REG_STATUS) & STATUS_INFERENCE_DONE)
            return 0;
    }
    return -1;
#endif
    return 0;
}

uint32_t edgeai_read_status(void)
{
    return edgeai_reg_read(EDGEAI_REG_STATUS);
}

bool edgeai_is_done(void)
{
    return (edgeai_reg_read(EDGEAI_REG_STATUS) & STATUS_INFERENCE_DONE) != 0;
}

bool edgeai_tensor_ready(void)
{
    return (edgeai_reg_read(EDGEAI_REG_STATUS) & STATUS_TENSOR_READY) != 0;
}

void edgeai_clear_tensor(void)
{
    edgeai_reg_write(EDGEAI_REG_CTRL, CTRL_TENSOR_CLEAR);
}
