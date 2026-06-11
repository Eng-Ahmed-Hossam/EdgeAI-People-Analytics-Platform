/*
 * main.c — Vitis bare-metal application for Zynq EdgeAI Location Intelligence
 *
 * Execution flow per frame:
 *   1. Wait for OV7670 frame-ready signal (GPIO or interrupt).
 *   2. Trigger PL inference (weight/bias already loaded at init).
 *   3. Poll inference_done.
 *   4. Read detection tensor from DDR.
 *   5. YOLO decode → NMS → person count.
 *   6. Publish count via UART/MQTT stub.
 */

#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "sleep.h"

#include "edgeai_pl.h"
#include "yolo_decode.h"
#include "nms.h"
#include "person_count.h"

/* ============================================================
 * Memory map (must match Vivado address editor)
 * ============================================================*/
#define WT_DDR_BASE       0x00210000UL  /* weight blob  (~2.5 MB) */
#define BIAS_DDR_BASE     0x00440000UL  /* bias blob    (~8.4 KB) */
#define FRAME_BUF_A       0x00200000UL  /* 128×128 RGB565 frame A */
#define FRAME_BUF_B       0x00208000UL  /* 128×128 RGB565 frame B */
#define TENSOR_DDR_BASE   0x00443000UL  /* L18 output tensor (4080 bytes) */

#define EDGEAI_AXI_BASE   0x43C00000UL  /* AXI-Lite GP0 base in PS map */

/* ============================================================
 * Working buffers (on-stack is fine for 4 KB tensor)
 * ============================================================*/
static int8_t    tensor_buf[YOLO_TENSOR_SIZE];
static Detection dets[YOLO_MAX_DETS];

/* ============================================================
 * MQTT stub — replace with lwIP MQTT or paho client
 * ============================================================*/
static void mqtt_publish_count(int count)
{
    /* Placeholder: send over UART for now */
    xil_printf("[MQTT] person_count=%d\r\n", count);
}

/* ============================================================
 * main
 * ============================================================*/
int main(void)
{
    xil_printf("\r\n=== EdgeAI Location Intelligence — Starting ===\r\n");

    /* Invalidate D-cache so DDR writes from PL are visible to ARM */
    Xil_DCacheInvalidate();

    /* Initialise PL driver */
    edgeai_init(EDGEAI_AXI_BASE);
    edgeai_configure(WT_DDR_BASE, BIAS_DDR_BASE,
                     FRAME_BUF_A, FRAME_BUF_B, TENSOR_DDR_BASE);

    /* Issue a soft reset to clear any stale PL state */
    edgeai_soft_reset();
    usleep(10);

    xil_printf("PL configured. Entering inference loop...\r\n");

    uint32_t frame_count = 0;

    while (1) {
        /* --------------------------------------------------------
         * Step 1: Wait for OV7670 frame.
         * In production: poll a GPIO "frame_valid" or wait for
         * a VDMA frame-done interrupt here.
         * For now: a short sleep simulates camera cadence (~15 fps).
         * --------------------------------------------------------*/
        usleep(66666);   /* ~15 fps */

        /* --------------------------------------------------------
         * Step 2: Trigger inference
         * --------------------------------------------------------*/
        int rc = edgeai_start_inference();
        if (rc != 0) {
            xil_printf("[ERROR] Inference timeout on frame %u\r\n", frame_count);
            edgeai_soft_reset();
            continue;
        }

        /* --------------------------------------------------------
         * Step 3: Flush D-cache before reading tensor from DDR
         * --------------------------------------------------------*/
        Xil_DCacheInvalidateRange(TENSOR_DDR_BASE, YOLO_TENSOR_SIZE);

        /* --------------------------------------------------------
         * Step 4: Copy tensor from DDR to local buffer
         * --------------------------------------------------------*/
        const int8_t *ddr_tensor = (const int8_t *)(uintptr_t)TENSOR_DDR_BASE;
        for (int i = 0; i < YOLO_TENSOR_SIZE; i++)
            tensor_buf[i] = ddr_tensor[i];

        /* --------------------------------------------------------
         * Step 5: Post-process
         * --------------------------------------------------------*/
        int n = yolo_decode(tensor_buf, dets);
        n = nms_run(dets, n, YOLO_NMS_THRESH);
        int persons = person_count(dets, n);

        /* --------------------------------------------------------
         * Step 6: Publish
         * --------------------------------------------------------*/
        xil_printf("[Frame %4u] dets=%d  persons=%d\r\n",
                    frame_count, n, persons);
        mqtt_publish_count(persons);

        /* Clear tensor ready flag for next inference */
        edgeai_clear_tensor();
        frame_count++;
    }

    return 0;
}
