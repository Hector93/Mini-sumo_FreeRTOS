/**
******************************************************************************
* File Name          : misc.h
* Description        : function definitions of the miscelaneous functions
******************************************************************************
**/

#ifndef __MISC_H
#define __MISC_H
#ifdef __cplusplus
extern "C" {
#endif
#include "cmsis_os.h"
  uint8_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);

#endif
