/*
 * buffer.c
 *
 * Created: 21/07/2017 12:01:24 AM
 *  Author: https://stackoverflow.com/questions/827691/how-do-you-implement-a-circular-buffer-in-c
 */

#include "history_buffer.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/**
 * \brief Average function
 * 
 * \param array
 * \param count
 * 
 * \return uint16_t
 */
uint16_t avg(uint16_t * array, uint8_t count)
{
  uint16_t sum = 0;

  for(uint8_t i = 0; i < count; i++)
  {
    sum += array[i];
  }

  return sum / count;
}

int32_t avg32(int32_t * array, uint8_t count)
{
  int32_t sum = 0;

  for(uint8_t i = 0; i < count; i++)
  {
    sum += array[i];
  }

  return sum / count;
}

//Algorithm from Numerical recipes in C of 1992

#define ELEM_SWAP(a,b) { register int16_t t=(a);(a)=(b);(b)=t; }

uint16_t quick_select_median(int16_t * arr, uint16_t n)
{
  uint16_t low, high;
  uint16_t median;
  uint16_t middle, ll, hh;
  low = 0 ; high = n - 1 ; median = (low + high) / 2;
  for (;;) {
    if (high <= low) /* One element only */
    return arr[median] ;
    if (high == low + 1) { /* Two elements only */
      if (arr[low] > arr[high])
      ELEM_SWAP(arr[low], arr[high]) ;
      return arr[median] ;
    }
    /* Find median of low, middle and high items; swap into position low */
    middle = (low + high) / 2;
    if (arr[middle] > arr[high])
    ELEM_SWAP(arr[middle], arr[high]) ;
    if (arr[low] > arr[high])
    ELEM_SWAP(arr[low], arr[high]) ;
    if (arr[middle] > arr[low])
    ELEM_SWAP(arr[middle], arr[low]) ;
    /* Swap low item (now in position middle) into position (low+1) */
    ELEM_SWAP(arr[middle], arr[low + 1]) ;
    /* Nibble from each end towards middle, swapping items when stuck */
    ll = low + 1;
    hh = high;
    for (;;) {
      do ll++; while (arr[low] > arr[ll]) ;
      do hh--; while (arr[hh] > arr[low]) ;
      if (hh < ll)
      break;
      ELEM_SWAP(arr[ll], arr[hh]) ;
    }
    /* Swap middle item (in position low) back into correct position */
    ELEM_SWAP(arr[low], arr[hh]) ;
    /* Re-set active partition */
    if (hh <= median)
    low = ll;
    if (hh >= median)
    high = hh - 1;
  }
  return arr[median] ;
}

void hb_init(circular_history_buffer *hb, size_t capacity, size_t sz)
{
    hb->buffer = malloc(capacity * sz);
    //if(cb->buffer == NULL)
    // handle error
    hb->buffer_end = (char *)hb->buffer + capacity * sz;
    hb->capacity = capacity;
    hb->count = 0;
    hb->sz = sz;
    hb->head = hb->buffer;
    hb->tail = hb->buffer;
}

void hb_free(circular_history_buffer *cb)
{
    free(cb->buffer);
    // clear out other fields too, just to be safe
}

void hb_push_back(circular_history_buffer *hb, const void *item)
{
    if(hb->count == hb->capacity)
    {
        //"Pop" then continue
        hb->tail = (char*)hb->tail + hb->sz;

        if(hb->tail == hb->buffer_end)
            hb->tail = hb->buffer;

        hb->count--;
    }

    memcpy(hb->head, item, hb->sz);
    hb->head = (char*)hb->head + hb->sz;

    if(hb->head == hb->buffer_end)
        hb->head = hb->buffer;

    hb->count++;
}

void hb_pop_front(circular_history_buffer *hb, void *item)
{
    if(hb->count == 0)
    {
        // do nothing if empty
    }
    else
    {
        memcpy(item, hb->tail, hb->sz);
        hb->tail = (char*)hb->tail + hb->sz;

        if(hb->tail == hb->buffer_end)
            hb->tail = hb->buffer;

        hb->count--;
    }
}
