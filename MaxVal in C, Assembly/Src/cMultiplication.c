/*
 * cMultiplication.c
 *
 *  Created on: 17 sept. 2022
 *      Author: HimelSaha
 */


#include "main.h"
#include "lab1math.h"




void cMultiplication(float *arrayOne, float *arrayTwo, float *result, uint32_t size) {

	for (uint32_t i = 0; i < size; i++) {
		result[i] = arrayOne[i] * arrayTwo[i];
	} // for


} // cMultiplication

