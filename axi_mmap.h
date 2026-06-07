/*
 * axi_mmap.h  —  /dev/mem helper to map pl_edgeai_top AXI slave into PS userspace
 *
 * Usage (Linux):
 *   #include "axi_mmap.h"
 *   volatile uint32_t *axi = axi_open(AXI_TENSOR_BASE, 0x1000);
 *   run_nms_pipeline(axi, 128.f, 128.f, &result);
 *   axi_close(axi, 0x1000);
 */

#ifndef AXI_MMAP_H
#define AXI_MMAP_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

/*
 * axi_open  —  mmap a physical AXI slave region.
 *
 * phys_addr : AXI slave base (from Vivado address editor, e.g. 0x43C00000)
 * size      : mapping size in bytes (e.g. 0x1000 = 4 KB)
 *
 * Returns a volatile uint32_t pointer aligned to phys_addr, or NULL on error.
 * The caller owns the pointer and must call axi_close() when done.
 */
static inline volatile uint32_t *axi_open(uintptr_t phys_addr, size_t size)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("axi_open: open /dev/mem");
        return NULL;
    }

    void *vaddr = mmap(NULL, size,
                       PROT_READ | PROT_WRITE,
                       MAP_SHARED,
                       fd, (off_t)phys_addr);
    close(fd);

    if (vaddr == MAP_FAILED) {
        perror("axi_open: mmap");
        return NULL;
    }

    return (volatile uint32_t *)vaddr;
}

/*
 * axi_close  —  unmap previously opened AXI region.
 */
static inline void axi_close(volatile uint32_t *vaddr, size_t size)
{
    if (vaddr) munmap((void *)vaddr, size);
}

#endif /* AXI_MMAP_H */
