/*
 * cmax.c
 *
 *  Created on: 13 sept. 2022
 *      Author: HimelSaha
 */

#include "main.h"
#include "lab1math.h"




void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex) {
	(*max) = array[0];
	(*maxIndex) = 0;
	for (uint32_t i = 1; i < size; i++) {
		if (array[i] > (*max)) {
			(*max) = array[i];
			(*maxIndex) = i;
		} // if
	} // for
} // cMax
