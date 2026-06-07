/*
 * nms.h  —  Non-Maximum Suppression for EdgeAI Tiny-YOLO accelerator
 *
 * Matches RTL parameters from edgeai_defs.vh and tensor_output_buffer.v:
 *   GRID_SIZE       = 4  (img_dim at head output)
 *   GRID_SIZE_2     = 13 (second head — if you expand later)
 *   DETECT_CHANNELS = 255 = 5 anchors × (5 + NUM_CLASSES)
 *   NUM_CLASSES     = 46  → 5 * (5 + 46) = 255  ✓
 *
 * Tensor layout from tensor_output_buffer:
 *   Sequential write order:  [channel][row][col]
 *   Total bytes:  GRID_SIZE × GRID_SIZE × DETECT_CHANNELS = 4×4×255 = 4080
 *
 * YOLO output interpretation per cell (per anchor):
 *   [tx, ty, tw, th, obj_conf, cls0 … cls45]  — all INT8 dequantized to float
 *
 * AXI-Lite read interface maps to pl_edgeai_top ports:
 *   tensor_rd_en   → AXI read enable
 *   tensor_rd_addr → 16-bit word address
 *   tensor_rd_data → INT8 signed result (1 clock latency in RTL)
 */

#ifndef NMS_H
#define NMS_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 * Tensor geometry  (must match edgeai_defs.vh)
 * ========================================================= */
#define GRID_ROWS        4
#define GRID_COLS        4
#define NUM_ANCHORS      3
#define NUM_CLASSES      80
#define ATTRS_PER_ANCHOR (5 + NUM_CLASSES)   /* tx,ty,tw,th,obj + classes */
#define DETECT_CHANNELS  (NUM_ANCHORS * ATTRS_PER_ANCHOR)  /* 255 */
#define TENSOR_SIZE      (GRID_ROWS * GRID_COLS * DETECT_CHANNELS)  /* 4080 */

/* =========================================================
 * AXI-Lite base address of pl_edgeai_top tensor read port
 * Adjust to match your Vivado address map.
 * ========================================================= */
#define AXI_TENSOR_BASE  0x43C00000UL   /* example: GP0 slave */
#define AXI_RD_EN_OFF    0x00           /* write 1 to trigger a read */
#define AXI_RD_ADDR_OFF  0x04           /* 16-bit address register   */
#define AXI_RD_DATA_OFF  0x08           /* 8-bit signed result       */
#define AXI_RD_VALID_OFF 0x0C           /* 1 when rd_data is valid   */
#define AXI_TENSOR_READY 0x10           /* tensor_ready flag         */

/* =========================================================
 * Dequantization
 * INT8 range: −128 … +127  →  float via per-layer scale
 * These scales come from the same export script as rq_mult/rq_shift.
 * For the detection heads (L15, L18) scale ≈ 0.00784 (1/127).
 * ========================================================= */
#define DEQUANT_SCALE    0.00784313725f   /* 1 / 127.5 */

/* =========================================================
 * NMS hyper-parameters
 * ========================================================= */
#define CONF_THRESHOLD 0.05f
#define IOU_THRESHOLD    0.45f
#define MAX_DETECTIONS   64              /* hard cap on output boxes  */

/* =========================================================
 * Anchor prior sizes (w, h) in grid units — 5 anchors
 * Derived from COCO clusters for 4×4 grid at 128-px input.
 * Replace with your actual calibrated anchors.
 * ========================================================= */
static const float ANCHORS[NUM_ANCHORS][2] = {
    {1.08f, 1.19f},
    {3.42f, 4.41f},
    {6.63f, 11.38f}
};
/* =========================================================
 * Data structures
 * ========================================================= */

/* Raw INT8 tensor (flat, channel-major order matching RTL write pointer) */
typedef struct {
    int8_t data[TENSOR_SIZE];
} RawTensor;

/* One decoded prediction box (before NMS) */
typedef struct {
    float x;           /* centre x in pixels (0 … IMG_W)  */
    float y;           /* centre y in pixels (0 … IMG_H)  */
    float w;           /* box width  in pixels             */
    float h;           /* box height in pixels             */
    float obj_conf;    /* objectness confidence [0,1]      */
    float cls_conf;    /* class score [0,1]                */
    int   cls_id;      /* winning class index              */
    float score;       /* obj_conf × cls_conf              */
    bool  suppressed;  /* NMS suppression flag             */
} Detection;

/* Final NMS output */
typedef struct {
    Detection boxes[MAX_DETECTIONS];
    int       count;
} DetectionResult;

/* =========================================================
 * Public API
 * ========================================================= */

/*
 * axi_wait_tensor_ready — poll until tensor_ready is asserted by RTL.
 * Returns true when ready, false on timeout.
 */
bool axi_wait_tensor_ready(volatile uint32_t *axi_base, uint32_t timeout_us);

/*
 * axi_read_tensor — read all TENSOR_SIZE bytes from tensor_output_buffer
 * via the AXI-Lite read port.  Fills out->data[].
 */
void axi_read_tensor(volatile uint32_t *axi_base, RawTensor *out);

/*
 * decode_predictions — dequantize raw tensor, sigmoid/exp activations,
 * decode bx/by/bw/bh, apply threshold, fill det[] array.
 * Returns number of candidates (before NMS).
 */
int decode_predictions(const RawTensor *tensor,
                       float img_w, float img_h,
                       Detection det[], int det_cap);

/*
 * nms_filter — apply IoU-based greedy NMS in-place.
 * Returns number of surviving detections written into result.
 */
int nms_filter(Detection det[], int n_det, DetectionResult *result);

/*
 * run_nms_pipeline — high-level: wait → read AXI → decode → NMS → return.
 */
bool run_nms_pipeline(volatile uint32_t *axi_base,
                      float img_w, float img_h,
                      DetectionResult *result);

#endif /* NMS_H */
