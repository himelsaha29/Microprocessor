/*
 * lab1math.h
 *
 *  Created on: 13 sept. 2022
 *      Author: HimelSaha
 */
#include "main.h"


#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_


void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);
extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

void cMultiplication(float *arrayOne, float *arrayTwo, float *result, uint32_t size);
extern void asmMultiplication(float *arrayOne, float *arrayTwo, float *arrayResult, uint32_t size);

void cStd(float *array, uint32_t size, float *result);
extern void asmStd(float *array, uint32_t size, float *result);


#endif /* INC_LAB1MATH_H_ */
