/*
 * asmStd.s
 *
 *  Created on: 16 sept. 2022
 *      Author: HimelSaha
 */


 // unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified
// .global exports the label asmMax, which is expected by lab1math.h
.global asmStd
// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata
/**
* void asmStd(float *array, uint32_t size, float *result);   // RETURN RESULT IN ARRAY
*
* R0 = pointer to array
* R1 = size
* R2 = Final result
* R3 = SIZE(N)
* R4 = ADDRESS OF CURRENT ITH ELEMENT
* S0 = TOTAL
* S1 = VALUE OF CURRENT ITH ELEMENT
* S2 = SIZE(N)
* S3 = MU

*/
asmStd:
 PUSH {R4, R5, R6, R7} // saving R4 and R5 according to calling convention
 MOV R3, R1 // R3 = SIZE(N)
 MOV R4, #0
 VMOV.f32 S0, R4

 //VSUB.F32 S0, S0, S0
loopOne:
 SUBS R1, R1, #1 // size = size - 1
 BLT doneOne // loop finishes when R1 < 0
 ADD R4, R0, R1, LSL #2 // calculate base address (in R5) for array element in arrayOne
 VLDR.f32 S1, [R4] // load element into fp register S1 (from address in R5)
 //VLDR.f32 S2, [R2] // load element into fp register S2 (from address in R2)
 VADD.f32 S0, S0, S1


continueOne:
 B loopOne // next iteration
doneOne:
 VMOV.f32 S2, R3   // SQ BRACKET ADDED HERE CHECK
 VCVT.f32.u32 S2, S2
 VDIV.f32 S3, S0, S2

 // DEBUG
 //VSTR.f32 S3, [R2]
 //BX LR
 //


 // NOW CALCULATE STD. DEV.

 /**
 * R0 = pointer to array
 * R1 = SIZE
 * R2 = RESULT
 * R3 = N-1
 * R4 = ADDRESS OF CURRENT ITH ELEMENT
 * S0 = TOTAL
 * S1 = VALUE OF CURRENT ITH ELEMENT
 * S2 = A(i) - mu / (A(i) - mu)^2
 * S4 = N-1
 * S5 = MIDDLE
 * S6 = RESULT

 */




asmStdAfterMu:
 MOV R1, R3	// RESET COUNTER
 SUBS R3, R3, #1	//  N-1
 MOV R4, #0
 VMOV.f32 S0, R4
 VCVT.f32.u32 S0, S0
 //VSUB.F32 S0, S0, S0
loopTwo:
 SUBS R1, R1, #1 // size = size - 1
 BLT doneTwo // loop finishes when R1 < 0
 ADD R4, R0, R1, LSL #2 // calculate base address (in R5) for array element in arrayOne
 VLDR.f32 S1, [R4] // load element into fp register S1 (from address in R3)

 //VLDR.f32 S0,[R4]
 VSUB.f32 S2, S1, S3		// S2 CONTAINS A(i) - mu
 VMUL.f32 S2, S2, S2		// S2 CONTAINS (A(i) - mu)^2

 // VLDR.f32 S4, [R3] // N-1
 VMOV.f32 S4, R3 // N-1
 VCVT.f32.u32 S4, S4

 VDIV.f32 S5, S2, S4 // (A(i) - mu)^2 / (N-1)

 VADD.f32 S0, S0, S5  // ADD TO THE TOTAL


continueTwo:
 B loopTwo // next iteration
doneTwo:
 VSQRT.f32 S6, S0

 VSTR.f32 S6, [R2]  // PREVIOUSLY [R4]

 POP {R4, R5, R6, R7} // restore context
 BX LR // return








