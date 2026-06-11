#ifndef NMS_H
#define NMS_H

#include "yolo_decode.h"

/* ============================================================
 * nms.h — Greedy class-aware Non-Maximum Suppression
 *
 * Algorithm:
 *   For each class c:
 *     1. Collect all detections with class_id == c.
 *     2. Sort by confidence descending.
 *     3. Greedily keep highest-confidence box; suppress any box
 *        with IoU > NMS_THRESH against it.
 *     4. Repeat for next un-suppressed box.
 * ============================================================*/

/* Run NMS in-place. Returns new count of surviving detections.
 * dets: array of n_dets detections (modified in place).
 * iou_thresh: typically YOLO_NMS_THRESH (0.45). */
int nms_run(Detection *dets, int n_dets, float iou_thresh);

/* Compute IoU of two boxes */
float nms_iou(const Detection *a, const Detection *b);

#endif /* NMS_H */
