# Makefile — EdgeAI NMS
#
# Targets:
#   make test        — native selftest (no hardware needed)
#   make arm         — cross-compile for Zynq PS (ARM Cortex-A9)
#   make clean

CC_NATIVE = gcc
CC_ARM    = arm-linux-gnueabihf-gcc

CFLAGS_COMMON = -O2 -Wall -Wextra -std=c11
CFLAGS_TEST   = $(CFLAGS_COMMON) -DNMS_SELFTEST -DNMS_VERBOSE -lm
CFLAGS_ARM    = $(CFLAGS_COMMON) -mfpu=neon-fp16 -mfloat-abi=hard \
                -mcpu=cortex-a9 -lm

SRCS = nms.c

.PHONY: test arm clean

test: $(SRCS) nms.h
	$(CC_NATIVE) $(CFLAGS_TEST) -o nms_selftest $(SRCS)
	./nms_selftest

arm: $(SRCS) nms.h axi_mmap.h
	$(CC_ARM) $(CFLAGS_ARM) -o nms_arm $(SRCS)

clean:
	rm -f nms_selftest nms_arm *.o
