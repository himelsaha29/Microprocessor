/*
 * cStd.c
 *
 *  Created on: 18 sept. 2022
 *      Author: HimelSaha
 */

#include "main.h"
#include "lab1math.h"


float square(float number);


void cStd(float *array, uint32_t size, float *result) {
	float mu = 0;
	float total = 0;
	for (uint32_t i = 0; i < size; i++) {
		total += array[i];
	} // for
	mu = total /(float)size;



	float middle = 0;
	//float mean = cMu(&array ,size, &mu);
	//cMu(&array ,size, &mu);
	for (uint32_t i = 0; i < size; i++) {
		middle += ((array[i] - mu) * (array[i] - mu))/((float)(size-1));
	} // for

	(*result) = square(middle);
	//return result;

}


void cMu(float *array, uint32_t size, float *result) {

	//return total;
}


float square(float number) {

	float temp, sqrt;

	// store the half of the given number e.g from 256 => 128
	sqrt = number / 2;
	temp = 0;

	// Iterate until sqrt is different of temp, that is updated on the loop
	while(sqrt != temp){
		// initially 0, is updated with the initial value of 128
	    // (on second iteration = 65)
	    // and so on
	    temp = sqrt;

	    // Then, replace values (256 / 128 + 128 ) / 2 = 6
	    // (on second iteration 34.46923076923077)
	    // and so on
	    sqrt = ( number/temp + temp) / 2;
	}
	return sqrt;

}
