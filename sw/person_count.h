#ifndef PERSON_COUNT_H
#define PERSON_COUNT_H

#include "yolo_decode.h"

/* ============================================================
 * person_count.h — Filter post-NMS detections for persons
 *
 * COCO class 0 = "person"
 * ============================================================*/

/* Count detections with class_id == 0 (person).
 * dets: post-NMS detection array.
 * n:    number of surviving detections.
 * Returns person count. */
int person_count(const Detection *dets, int n);

#endif /* PERSON_COUNT_H */
