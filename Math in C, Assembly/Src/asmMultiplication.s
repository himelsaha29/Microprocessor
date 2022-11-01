/*
 * asmMultiplication.s
 *
 *  Created on: 16 sept. 2022
 *      Author: HimelSaha

 * R5 = i current element address; arrayOne[0 + i]
 * R6 = i current element address; arrayTwo[0 + i]
 * R7 = i current element address; arrayResult[0 + i]
 * S1 = value of current element in arrayOne
 * S2 = value of current element in arrayTwo
 * S3 = value of current element in arrayResult

 */


 // unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified
// .global exports the label asmMax, which is expected by lab1math.h
.global asmMultiplication
// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata
/**
* void asmMultiplication(float *arrayOne, float *arrayTwo, float *arrayResult, uint32_t size);
*
* R0 = pointer to arrayOne
* R1 = pointer to arrayTwo
* R2 = pointer to arrayResult
* R3 = size
*/
asmMultiplication:
 PUSH {R4, R5, R6} // saving R4 and R5 according to calling convention
 MOV R6, R2 // R6 = R2
loop:
 SUBS R3, R3, #1 // size = size - 1
 BLT done // loop finishes when R1 < 0
 ADD R4, R0, R3, LSL #2 // calculate base address (in R5) for array element in arrayOne
 ADD R5, R1, R3, LSL #2 // calculate base address (in R5) for array element in arrayTwo
 ADD R2, R6, R3, LSL #2 // calculate base address (in R5) for array element in arrayResult
 VLDR.f32 S0, [R4] // load element into fp register S1 (from address in R5)
 VLDR.f32 S1, [R5] // load element into fp register S2 (from address in R6)
 VLDR.f32 S2, [R2] // load element into fp register S3 (from address in R7)

 VMUL.f32 S2, S0, S1 // MULTIPLY S0 with S1 and store it in S2
 VSTR.f32 S2, [R2]


continue:
 B loop // next iteration
done:
 POP {R4, R5, R6} // restore context
 BX LR
