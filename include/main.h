/*==============================================================================
 * Name        : main.h
 * Author      : Martin Doff-Sotta (martin.doff-sotta@eng.ox.ac.uk) 
 * Description : Header for main.c file.
 -------------------------------------------------------------------------------
 * The MIT License (MIT)
 * Copyright (c) 2021 Martin Doff-Sotta
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
===============================================================================*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32h7xx.h"

/* Define registers */
#define RCC_AHB4ENR   *(volatile uint32_t *)(RCC_BASE   + 0x0E0) // register APB2 for clock enabling
#define GPIOA_MODER   *(volatile uint32_t *)(GPIOA_BASE + 0x00) // E port control register (config. I/O mode)
#define GPIOA_OTYPER  *(volatile uint32_t *)(GPIOA_BASE + 0x04) // E port control register (config. output type)
#define GPIOA_OSPEEDR *(volatile uint32_t *)(GPIOA_BASE + 0x08) // E port control register (config. speed)
#define GPIOA_PUPDR   *(volatile uint32_t *)(GPIOA_BASE + 0x0C)
#define GPIOA_ODR     *(volatile uint32_t *)(GPIOA_BASE + 0x14) // E port data register (read/write)

// Bit fields
#define GPIOAEN (  1 << 0) // IO port A clock enable
#define GPIOA1  (1UL << 1)


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */