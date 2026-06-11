#include "yolo_decode.h"
#include <math.h>
#include <string.h>

/* Anchors for large-object detector, pre-scaled to 128/416 */
static const float anchors[YOLO_NUM_ANCHORS][2] = {
    {81.0f * 128.0f / 416.0f,  82.0f * 128.0f / 416.0f},
    {135.0f * 128.0f / 416.0f, 169.0f * 128.0f / 416.0f},
    {344.0f * 128.0f / 416.0f, 319.0f * 128.0f / 416.0f}
};

float yolo_sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

/* ============================================================
 * Tensor layout:
 *   The RTL outputs L18 in "channel-major" after scatter:
 *   channel c at spatial (y, x) → index [c * GRID_H * GRID_W + y * GRID_W + x]
 *
 *   YOLO channel mapping (255 channels total):
 *     anchor a, entry e → channel = a * YOLO_ENTRY_SIZE + e
 *   where e: 0=tx, 1=ty, 2=tw, 3=th, 4=obj, 5..84=class[0..79]
 * ============================================================*/

static inline float dequant(int8_t v)
{
    return (float)v * YOLO_INT8_SCALE;
}

static inline int tensor_idx(int ch, int y, int x)
{
    return ch * (YOLO_GRID_H * YOLO_GRID_W) + y * YOLO_GRID_W + x;
}

int yolo_decode(const int8_t *tensor, Detection *dets)
{
    int n_dets = 0;

    for (int gy = 0; gy < YOLO_GRID_H; gy++) {
        for (int gx = 0; gx < YOLO_GRID_W; gx++) {
            for (int a = 0; a < YOLO_NUM_ANCHORS; a++) {
                int base_ch = a * YOLO_ENTRY_SIZE;

                /* Objectness confidence */
                float obj_raw = dequant(tensor[tensor_idx(base_ch + 4, gy, gx)]);
                float obj_conf = yolo_sigmoid(obj_raw);
                if (obj_conf < YOLO_CONF_THRESH) continue;

                /* Find best class */
                float best_prob = 0.0f;
                int   best_cls  = 0;
                for (int c = 0; c < YOLO_NUM_CLASSES; c++) {
                    float cls_raw  = dequant(tensor[tensor_idx(base_ch + 5 + c, gy, gx)]);
                    float cls_prob = yolo_sigmoid(cls_raw);
                    if (cls_prob > best_prob) {
                        best_prob = cls_prob;
                        best_cls  = c;
                    }
                }

                float score = obj_conf * best_prob;
                if (score < YOLO_CONF_THRESH) continue;

                /* Box decode */
                float tx = dequant(tensor[tensor_idx(base_ch + 0, gy, gx)]);
                float ty = dequant(tensor[tensor_idx(base_ch + 1, gy, gx)]);
                float tw = dequant(tensor[tensor_idx(base_ch + 2, gy, gx)]);
                float th = dequant(tensor[tensor_idx(base_ch + 3, gy, gx)]);

                float cell_w = (float)YOLO_IMG_W / (float)YOLO_GRID_W;
                float cell_h = (float)YOLO_IMG_H / (float)YOLO_GRID_H;

                float bx = (yolo_sigmoid(tx) + (float)gx) * cell_w;
                float by = (yolo_sigmoid(ty) + (float)gy) * cell_h;
                float bw = anchors[a][0] * expf(tw);
                float bh = anchors[a][1] * expf(th);

                if (n_dets >= YOLO_MAX_DETS) break;
                Detection *d = &dets[n_dets++];
                d->x1         = bx - bw * 0.5f;
                d->y1         = by - bh * 0.5f;
                d->x2         = bx + bw * 0.5f;
                d->y2         = by + bh * 0.5f;
                d->confidence = score;
                d->class_id   = best_cls;
            }
            if (n_dets >= YOLO_MAX_DETS) break;
        }
        if (n_dets >= YOLO_MAX_DETS) break;
    }

    return n_dets;
}
