#ifndef YOLO_DECODE_H
#define YOLO_DECODE_H

#include <stdint.h>

/* ============================================================
 * yolo_decode.h — Post-processing for Tiny-YOLOv3 L18 tensor
 *
 * Tensor layout from RTL (after scatter to DDR):
 *   GRID_H x GRID_W x NUM_ANCHORS x (5 + NUM_CLASSES)
 *   = 4 x 4 x 3 x 85 = 4080 INT8 values
 *
 * INT8 dequantization: float = INT8 * INT8_SCALE (1/16.0)
 * Anchors (original 416px, scaled to 128px input):
 *   (25, 25), (42, 52), (106, 98)  — large object set
 *
 * Box encoding (YOLO format, per grid cell):
 *   tx, ty, tw, th, obj_conf, class[0..79]
 *   bx = sigmoid(tx) + cx
 *   by = sigmoid(ty) + cy
 *   bw = anchor_w * exp(tw)
 *   bh = anchor_h * exp(th)
 * ============================================================*/

#define YOLO_GRID_W       4
#define YOLO_GRID_H       4
#define YOLO_NUM_ANCHORS  3
#define YOLO_NUM_CLASSES  80
#define YOLO_ENTRY_SIZE   (5 + YOLO_NUM_CLASSES)   /* 85 */
#define YOLO_TENSOR_SIZE  (YOLO_GRID_W * YOLO_GRID_H * YOLO_NUM_ANCHORS * YOLO_ENTRY_SIZE)  /* 4080 */
#define YOLO_IMG_W        128
#define YOLO_IMG_H        128

#define YOLO_INT8_SCALE   (1.0f / 16.0f)
#define YOLO_CONF_THRESH  0.25f
#define YOLO_NMS_THRESH   0.45f

/* Maximum detections before NMS */
#define YOLO_MAX_DETS     200

typedef struct {
    float x1, y1, x2, y2;  /* absolute pixel coords (0..IMG_W/H) */
    float confidence;       /* obj_conf * class_prob */
    int   class_id;         /* 0..79 */
} Detection;

/* ============================================================
 * Public API
 * ============================================================*/

/* Decode raw INT8 tensor from DDR into detection array.
 * tensor: pointer to YOLO_TENSOR_SIZE INT8 values (channel-major, scattered).
 * dets:   output buffer, caller provides YOLO_MAX_DETS entries.
 * Returns number of detections above conf_thresh (before NMS). */
int yolo_decode(const int8_t *tensor, Detection *dets);

/* Helper: sigmoid */
float yolo_sigmoid(float x);

#endif /* YOLO_DECODE_H */
