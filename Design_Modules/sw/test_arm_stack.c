/*
 * test_arm_stack.c — PC host harness for the ARM post-processing stack.
 *
 * Compiles and runs yolo_decode + nms + person_count on a PC (x86/gcc),
 * with NO Zynq/XSDK dependency, against a golden L18 tensor hex file.
 *
 * The tensor hex file is one byte per line in hex (00..ff), 4080 lines,
 * channel-major [c][y][x] — exactly the format written by
 * sim/run_golden_real_images.py (and the RTL reorder/scatter output).
 *
 * Usage:
 *   test_arm <tensor.hex> [expected_person_count]
 *
 * Build (MinGW / WSL / Linux):
 *   gcc test_arm_stack.c yolo_decode.c nms.c person_count.c -I. -lm -o test_arm
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "yolo_decode.h"
#include "nms.h"
#include "person_count.h"

/* Load a tensor hex file: one hex byte per line, interpreted as signed int8. */
static int load_tensor_hex(const char *path, int8_t *tensor, int max_len)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "ERROR: cannot open '%s'\n", path);
        return -1;
    }
    char line[64];
    int n = 0;
    while (n < max_len && fgets(line, sizeof(line), f)) {
        /* skip blank lines */
        if (line[0] == '\n' || line[0] == '\r' || line[0] == '\0')
            continue;
        unsigned int byte = 0;
        if (sscanf(line, "%x", &byte) != 1)
            continue;
        tensor[n++] = (int8_t)(byte & 0xFF);   /* two's-complement int8 */
    }
    fclose(f);
    return n;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <tensor.hex> [expected_person_count]\n", argv[0]);
        return 2;
    }
    const char *path = argv[1];
    int expected = (argc >= 3) ? atoi(argv[2]) : -1;

    static int8_t    tensor[YOLO_TENSOR_SIZE];
    static Detection dets[YOLO_MAX_DETS];

    int got = load_tensor_hex(path, tensor, YOLO_TENSOR_SIZE);
    if (got < 0)
        return 1;
    if (got != YOLO_TENSOR_SIZE) {
        fprintf(stderr, "WARNING: read %d values, expected %d\n",
                got, YOLO_TENSOR_SIZE);
    }

    printf("==============================================================\n");
    printf(" ARM post-processing stack — PC validation\n");
    printf("   tensor: %s  (%d bytes)\n", path, got);
    printf("==============================================================\n");

    int raw = yolo_decode(tensor, dets);
    printf("  Raw detections (pre-NMS): %d\n", raw);
    for (int i = 0; i < raw && i < 5; i++)
        printf("    [cls %2d] conf=%.3f  box=(%.1f,%.1f,%.1f,%.1f)\n",
               dets[i].class_id, dets[i].confidence,
               dets[i].x1, dets[i].y1, dets[i].x2, dets[i].y2);

    int kept = nms_run(dets, raw, YOLO_NMS_THRESH);
    printf("  Post-NMS detections:     %d\n", kept);

    int persons = person_count(dets, kept);
    printf("  Person count:            %d\n", persons);

    if (expected >= 0) {
        int pass = (persons == expected);
        printf("  Expected:                %d  -> %s\n",
               expected, pass ? "PASS" : "FAIL");
        printf("==============================================================\n");
        return pass ? 0 : 1;
    }
    printf("==============================================================\n");
    return 0;
}
