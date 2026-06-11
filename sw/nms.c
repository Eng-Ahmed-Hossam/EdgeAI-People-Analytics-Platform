#include "nms.h"
#include <string.h>

float nms_iou(const Detection *a, const Detection *b)
{
    float ix1 = a->x1 > b->x1 ? a->x1 : b->x1;
    float iy1 = a->y1 > b->y1 ? a->y1 : b->y1;
    float ix2 = a->x2 < b->x2 ? a->x2 : b->x2;
    float iy2 = a->y2 < b->y2 ? a->y2 : b->y2;

    float iw = ix2 - ix1;
    float ih = iy2 - iy1;
    if (iw <= 0.0f || ih <= 0.0f) return 0.0f;

    float inter = iw * ih;
    float area_a = (a->x2 - a->x1) * (a->y2 - a->y1);
    float area_b = (b->x2 - b->x1) * (b->y2 - b->y1);
    float uni    = area_a + area_b - inter;
    if (uni <= 0.0f) return 0.0f;
    return inter / uni;
}

/* Simple insertion sort descending by confidence */
static void sort_dets_desc(Detection *dets, int n)
{
    for (int i = 1; i < n; i++) {
        Detection key = dets[i];
        int j = i - 1;
        while (j >= 0 && dets[j].confidence < key.confidence) {
            dets[j + 1] = dets[j];
            j--;
        }
        dets[j + 1] = key;
    }
}

int nms_run(Detection *dets, int n_dets, float iou_thresh)
{
    if (n_dets <= 0) return 0;

    /* suppressed[i]=1 means box i is removed */
    static uint8_t suppressed[YOLO_MAX_DETS];
    memset(suppressed, 0, (size_t)n_dets);

    /* Process one class at a time to keep class boundaries intact */
    for (int cls = 0; cls < YOLO_NUM_CLASSES; cls++) {
        /* Collect indices belonging to this class, already globally sorted */
        for (int i = 0; i < n_dets; i++) {
            if (suppressed[i] || dets[i].class_id != cls) continue;
            for (int j = i + 1; j < n_dets; j++) {
                if (suppressed[j] || dets[j].class_id != cls) continue;
                if (nms_iou(&dets[i], &dets[j]) > iou_thresh)
                    suppressed[j] = 1;
            }
        }
    }

    /* Sort by confidence first so greedy NMS is correct */
    sort_dets_desc(dets, n_dets);

    /* Re-run NMS after sorting (index-based pass above may be order-sensitive) */
    memset(suppressed, 0, (size_t)n_dets);
    for (int cls = 0; cls < YOLO_NUM_CLASSES; cls++) {
        for (int i = 0; i < n_dets; i++) {
            if (suppressed[i] || dets[i].class_id != cls) continue;
            for (int j = i + 1; j < n_dets; j++) {
                if (suppressed[j] || dets[j].class_id != cls) continue;
                if (nms_iou(&dets[i], &dets[j]) > iou_thresh)
                    suppressed[j] = 1;
            }
        }
    }

    /* Compact surviving detections */
    int out = 0;
    for (int i = 0; i < n_dets; i++) {
        if (!suppressed[i])
            dets[out++] = dets[i];
    }
    return out;
}
