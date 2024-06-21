/*
 ****************************************************************************
 *
 *                  UNIVERSITY OF WATERLOO ECE 350 RTOS LAB
 *
 *                     Copyright 2020-2022 Yiqing Huang
 *                          All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice and the following disclaimer.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************
 */

/**************************************************************************//**
 * @file        k_mem.c
 * @brief       Kernel Memory Management API C Code
 *
 * @version     V1.2021.01.lab2
 * @authors     Yiqing Huang
 * @date        2021 JAN
 *
 * @note        skeleton code
 *
 *****************************************************************************/

#include "k_inc.h"
#include <cstddef>
#include "k_mem.h"

/*---------------------------------------------------------------------------
The memory map of the OS image may look like the following:
                   RAM1_END-->+---------------------------+ High Address
                              |                           |
                              |                           |
                              |       MPID_IRAM1          |
                              |   (for user space heap  ) |
                              |                           |
                 RAM1_START-->|---------------------------|
                              |                           |
                              |  unmanaged free space     |
                              |                           |
&Image$$RW_IRAM1$$ZI$$Limit-->|---------------------------|-----+-----
                              |         ......            |     ^
                              |---------------------------|     |
                              |                           |     |
                              |---------------------------|     |
                              |                           |     |
                              |      other data           |     |
                              |---------------------------|     |
                              |      PROC_STACK_SIZE      |  OS Image
              g_p_stacks[2]-->|---------------------------|     |
                              |      PROC_STACK_SIZE      |     |
              g_p_stacks[1]-->|---------------------------|     |
                              |      PROC_STACK_SIZE      |     |
              g_p_stacks[0]-->|---------------------------|     |
                              |   other  global vars      |     |
                              |                           |  OS Image
                              |---------------------------|     |
                              |      KERN_STACK_SIZE      |     |                
   g _k_stacks[MAX_TASKS-1]-->|---------------------------|     |
                              |                           |     |
                              |     other kernel stacks   |     |                              
                              |---------------------------|     |
                              |      KERN_STACK_SIZE      |  OS Image
              g_k_stacks[2]-->|---------------------------|     |
                              |      KERN_STACK_SIZE      |     |                      
              g_k_stacks[1]-->|---------------------------|     |
                              |      KERN_STACK_SIZE      |     |
              g_k_stacks[0]-->|---------------------------|     |
                              |   other  global vars      |     |
                              |---------------------------|     |
                              |        TCBs               |  OS Image
                      g_tcbs->|---------------------------|     |
                              |        global vars        |     |
                              |---------------------------|     |
                              |                           |     |          
                              |                           |     |
                              |        Code + RO          |     |
                              |                           |     V
                 IRAM1_BASE-->+---------------------------+ Low Address
    
---------------------------------------------------------------------------*/ 

/*
 *===========================================================================
 *                            GLOBAL VARIABLES
 *===========================================================================
 */
// kernel stack size, referred by startup_a9.s
const U32 g_k_stack_size = KERN_STACK_SIZE;
// task proc space stack size in bytes, referred by system_a9.c
const U32 g_p_stack_size = PROC_STACK_SIZE;

// task kernel stacks
U32 g_k_stacks[MAX_TASKS][KERN_STACK_SIZE >> 2] __attribute__((aligned(8)));

// task process stack (i.e. user stack) for tasks in thread mode
// remove this bug array in your lab2 code
// the user stack should come from MPID_IRAM2 memory pool
//U32 g_p_stacks[MAX_TASKS][PROC_STACK_SIZE >> 2] __attribute__((aligned(8)));
U32 g_p_stacks[NUM_TASKS][PROC_STACK_SIZE >> 2] __attribute__((aligned(8)));

/*
 *===========================================================================
 *                            DEFINES
 *===========================================================================
 */
#define MAX_ORDER   IRAM1_SIZE_LOG2 /*log2(IRAM1/IRAM2 SIZE)*/
#define HEADER_SIZE sizeof(header_t) /* for alignment */

/*
 *===========================================================================
 *                            DATA STRUCTURES
 *===========================================================================
 */
typedef struct header
{
    U32  size;
    BOOL is_free;
    U32  order;
    header_t *prev;
    header_t *next;
}header_t;

typedef struct block
{
    header_t *header;
}block_t;

typedef struct freeList
{
    header_t *head;
}freeList_t;

/*
 *===========================================================================
 *                            FUNCTIONS
 *===========================================================================
 */

/* note list[n] is for blocks with order of n */
mpool_t k_mpool_create (int algo, U32 start, U32 end)
{
    mpool_t mpid;
    U32 total_size;
    U32 max_order;
    header_t *first_header;
    block_t *first_block;
    static freeList_t free_list[MAX_ORDER + 1];

#ifdef DEBUG_0
    printf("k_mpool_init: algo = %d\r\n", algo);
    printf("k_mpool_init: RAM range: [0x%x, 0x%x].\r\n", start, end);
#endif /* DEBUG_0 */    
    
    if (algo != BUDDY ) {
        errno = EINVAL;
        return RTX_ERR;
    }
    
    if ( start == RAM1_START) {
        mpid = MPID_IRAM1;
    } else if ( start == RAM2_START) {
        mpid = MPID_IRAM2;
    } else {
        errno = EINVAL;
        return RTX_ERR;
    }

    // 1) Calculate the total size of the memory pool
    total_size = end - start;

    // 2) Verify block size isn’t too small
    if (total_size < MIN_BLK_SIZE)
    {
        errno = EINVAL;
        return RTX_ERR;
    }

    // 3) Align the total_size to the nearest power of 2 and determine max_order
    total_size = prv_align_to_power_of_two(total_size);
    max_order = prv_calculate_max_order(total_size);

    // 4) Create header for the first block
    first_header = (header_t *)start;  // Assume header is at the start of the memory pool
    first_header->size = total_size;
    first_header->is_free = TRUE;
    first_header->order = max_order;
    first_header->prev = NULL;
    first_header->next = NULL;

    // 5) Create first block
    first_block = (block_t *)(start + HEADER_SIZE);
    first_block->header = first_header;

    // 6) Initialize free list
    for (int i = 0; i <= MAX_ORDER; i++) {
        free_list[i].head = NULL;
    }
    free_list[max_order].head = first_header;

    return mpid;
}

// Aligns the value to the nearest power of two
static U32 prv_align_to_power_of_two(U32 val) {
    U32 power_of_two = 1;
    while (power_of_two < val) {
        power_of_two <<= 1;
    }
    return power_of_two;
}

// Calculates the maximum order of the bit tree based on the size
static U32 prv_calculate_max_order(U32 val) {
    U32 order = 0;
    while (val >>= 1) {
        order++;
    }
    return order;
}

void *k_mpool_alloc (mpool_t mpid, size_t size)
{
    U32 actual_size = 0;
    header_t *suitable_block;

#ifdef DEBUG_0
    printf("k_mpool_alloc: mpid = %d, size = %d, 0x%x\r\n", mpid, size, size);
#endif /* DEBUG_0 */
    // return ((void *) IRAM2_BASE);
    
    if (size == 0)
    {
        return NULL;
    }

    // 1) Calculate the size of memory block to use
    actual_size = prv_align_to_power_of_two(size + HEADER_SIZE);

    // 2) Verify size of memory block
    if (actual_size > IRAM1_SIZE)
    {
        errno = ENOMEM;
        return NULL;
    }
    
    // 3) Find the smallest free block of sufficient size
    suitable_block = find_smallest_free_block(actual_size);
    if (!suitable_block)
    {
        return NULL;
    }
    
    // 4) Allocate memory block selected and Update free list
    // a) If a block larger than the requested size is used split them
    header_t *allocated_block = suitable_block;
    if(suitable_block->size > actual_size)
    {
        //Split the block
        U32 buddy_order = prv_calculate_max_order(actual_size) - 1;
        U32 buddy_size = allocated_block->size/2;

        //Create buddy block
        header_t *buddy_block = (header_t*)(allocated_block + buddy_size);
        buddy_block->order = buddy_order;
        buddy_block->size = buddy_size;
        buddy_block->is_free = TRUE;

        //Adjust pointers
        buddy_block->prev = NULL;
        buddy_block->next = free_list[buddy_order].head;

        if (free_list[buddy_order].head)
        {
            free_list[buddy_order].head->prev = buddy_block;
        }
        free_list[buddy_order].head = buddy_block;

        //Update allocated block
        allocated_block->size = buddy_size;
        allocated_block->order = buddy_order;
    } 
    // b) Mark allocated memory block as used
    allocated_block->is_free = FALSE;

    // c) Remove allocated block from free list
    if (allocated_block->prev)
    {
        allocated_block->prev->next = allocated_block->next;
    }
    if (allocated_block->next)
    {
        allocated_block->next->prev = allocated_block->prev;
    }
    if (free_list[allocated_block->order].head == allocated_block)
    {
        free_list[allocated_block->order].head = allocated_block->next;
    }
    
    return (void*)(allocated_block + HEADER_SIZE);
}


static header_t* find_smallest_free_block(U32 actual_size)
{
    U32 block_order = prv_calculate_max_order(actual_size);
    for (size_t i = block_order; i <= MAX_ORDER; i++)
    {
        if (!free_list[i].head)
        {
            return free_list[i].head;
        }
    }
    return NULL;
}

int k_mpool_dealloc(mpool_t mpid, void *ptr)
{
    header_t *block;
    header_t *buddy_block;

#ifdef DEBUG_0
    printf("k_mpool_dealloc: mpid = %d, ptr = 0x%x\r\n", mpid, ptr);
#endif /* DEBUG_0 */
    
    // 1) Verify ptr argument
    if (!ptr)
    {
        return RTX_OK;
    }

    // 2) Verify mpid argument
    if ((mpid != MPID_IRAM1) && (mpid != MPID_IRAM2))
    {
        errno = EINVAL;
        return RTX_ERR;
    }

    // 3) Check if ptr is within the memory pool range
    if (mpid == MPID_IRAM1)
    {
        if ((ptr < (void *)RAM1_START) || (ptr > (void *)RAM1_END))
        {
            errno = EFAULT;
            return RTX_ERR;
        }
    
    }else
    {
        if ((&ptr < (void *)RAM2_START) || (&ptr > (void *)RAM2_END))
        {
            errno = EFAULT;
            return RTX_ERR;
        }
    }
    
    
    // 4) Get the memory block header location
    block = (header_t*)(ptr-HEADER_SIZE)
    
    // 5) Check if the block is already free
    if (block->is_free)
    {
        return RTX_OK;
    }
    
    // 6) Mark the block as free
    block->is_free = TRUE;

    // 7) Attempt to merge with the buddy block
    merge_blocks(block);
    
    return RTX_OK; 
}

static void merge_blocks(header_t* block)
{
    U32 buddy_order = block->order;
    U32 buddy_address = block ^ block->size;
    header_t *buddy_block = (header_t*) buddy_address;

    //Check if buddy exists and is free
    if ((buddy_block->order = buddy_order) && (buddy_block->is_free))
    {
        //Remove buddy from free list
        if (buddy_block->prev)
        {
            buddy_block->prev->next = buddy_block->next;
        }
        if (buddy_block->next)
        {
            buddy_block->next->prev = buddy_block->prev;
        }
        if (free_list[buddy_order].head)
        {
            free_list[buddy_order].head = buddy_block->next;
        }
        
        //Merge blocks
        if (block < buddy_address)
        {
            block->size *= 2;
            block->order++;
        }else
        {
            buddy_block->size *= 2;
            buddy_block->order++;
            block = buddy_block;
        }
        
        merge_blocks(block);
    }
    
    else // Add block alone to free list if no merge happened
    {
        block->next = free_list[block->order].head;
        block->prev = NULL;
        if (free_list[block->order].head)
        {
            free_list[block->order].head->prev = block;
        }        
        free_list[block->order].head = block;
    }
}

int k_mpool_dump (mpool_t mpid)
{
    U32 num_free_blocks = 0;
    header_t *curr_block;
#ifdef DEBUG_0
    printf("k_mpool_dump: mpid = %d\r\n", mpid);
#endif /* DEBUG_0 */
    
    // 1) Verify mpid argument
    if ((mpid != MPID_IRAM1) && ((mpid != MPID_IRAM2)))
    {
        return 0;
    }
    
    // 2) Iterate through each free list by order
    for (U32 i = 0; i <= MAX_ORDER; i++)
    {
        curr_block = free_list[i].head;

        //Iterate through the free list
        while (curr_block)
        {
            printf("0x%08x: 0x%02x\n", (U32)curr_block, curr_block->size);
            num_free_blocks++;
            curr_block = curr_block->next;
        }
    }

    // 3) Print summary
    printf("\n%d free memory block(s) found", num_free_blocks);

    return num_free_blocks;
}

int k_mem_init(int algo)
{
#ifdef DEBUG_0
    printf("k_mem_init: algo = %d\r\n", algo);
#endif /* DEBUG_0 */
        
    if ( k_mpool_create(algo, RAM1_START, RAM1_END) < 0 ) {
        return RTX_ERR;
    }
    
    if ( k_mpool_create(algo, RAM2_START, RAM2_END) < 0 ) {
        return RTX_ERR;
    }
    
    return RTX_OK;
}

/**
 * @brief allocate kernel stack statically
 */
U32* k_alloc_k_stack(task_t tid)
{
    
    if ( tid >= MAX_TASKS) {
        errno = EAGAIN;
        return NULL;
    }
    U32 *sp = g_k_stacks[tid+1];
    
    // 8B stack alignment adjustment
    if ((U32)sp & 0x04) {   // if sp not 8B aligned, then it must be 4B aligned
        sp--;               // adjust it to 8B aligned
    }
    return sp;
}

/**
 * @brief allocate user/process stack statically
 * @attention  you should not use this function in your lab
 */

U32* k_alloc_p_stack(task_t tid)
{
    if ( tid >= NUM_TASKS ) {
        errno = EAGAIN;
        return NULL;
    }
    
    U32 *sp = g_p_stacks[tid+1];
    
    
    // 8B stack alignment adjustment
    if ((U32)sp & 0x04) {   // if sp not 8B aligned, then it must be 4B aligned
        sp--;               // adjust it to 8B aligned
    }
    return sp;
}

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */

