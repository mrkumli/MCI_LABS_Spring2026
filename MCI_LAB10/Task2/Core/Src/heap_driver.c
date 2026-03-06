#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

// Students should be provided the above code (includes and defines) and the function declarations in this file.
// They can figure out the rest.

// Allocation bitmap: 0 = free, 1 = used
uint8_t allocation_bitmap[BLOCK_COUNT];
// Add you code below

void heap_init() {
    for (int i = 0; i < BLOCK_COUNT; i++) {
        allocation_bitmap[i] = 0;
    }
}

void* heap_alloc(size_t size){
    int blocks = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    int availBlocks = 0;
    for (int i = 0; i < BLOCK_COUNT; i++) {
        if (i >= BLOCK_COUNT) {
            break;
        }
        if (allocation_bitmap[i] == 0){
            availBlocks++;
            if (availBlocks == blocks) {
                int firstBlock = i - blocks + 1;
                for (int j = firstBlock; j <= i; j++) {
                    allocation_bitmap[j] = 1;
                }
                return (void*)(HEAP_START_ADDR + firstBlock * BLOCK_SIZE);
            }
        } else {
            availBlocks = 0;
        }
    }
    return NULL;
}

void heap_free(void* ptr) {
    if (ptr == NULL) {
        return;
    }
    int blockIndex = ((uint8_t*)ptr - HEAP_START_ADDR) / BLOCK_SIZE;
    for (int i = blockIndex; i < BLOCK_COUNT; i++) {
        if (allocation_bitmap[i] == 1) {
            allocation_bitmap[i] = 0;
        } else {
            break;
        }
    }
}