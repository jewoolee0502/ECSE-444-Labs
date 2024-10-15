/*
kalmanFilter_assembly.s
*/
 .section .data

.syntax unified //as.pdf
.align 16
.section .text, "x"

.global kalmanFilter_assembly

kalmanFilter_assembly:	//label

/*
S0 = measurement
S1 = x
S3 = q
S4 = r
S5 = p
S6 = k
S7 = temp
*/

	VPUSH.32 {S0-S7} //pushing floating type registers into stack to initialize the above 7 registers
// extra
	MOV R1, #0
	VMSR FPSCR, R1 // clear FPSCR by writing to it
//--------
	//load values from the address into floating point registers
	VLDR.F32 S3,[R0] //q
	VLDR.F32 S4,[R0, #4] //r
	VLDR.F32 S1,[R0, #8] //x
	VLDR.F32 S5,[R0, #12] //p
	VLDR.F32 S6,[R0, #16] //k


	// p=p+q
	VADD.F32 S5, S5, S3 //p = p+q


	//k=p/(p+r)
	VADD.F32 S7, S5, S4 //temp = p+r
	VDIV.F32 S6, S5, S7 //k = p/temp


	//x=x+k(measurement-x)
	VSUB.F32 S7, S0, S1 //temp = measurement-x
	VMUL.F32 S7, S6, S7 //temp = k*temp --> (here temp = measurement-x)
	VADD.F32 S1, S1, S7 //x = x+temp


	//p=(1-k)*p
	VMOV.F32 S7, #1.0 //loading floating value 1 into S7 (temp)
	VSUB.F32 S7, S7, S6 //temp = temp-k
	VMUL.F32 S5, S7, S5 //p = temp*p


	VMRS R1, FPSCR 	//Copy FPSCR to R1
	TST R1, #0xf // test the DZC (Division by zero) bit
	BNE ERROR // bracnh to error if division by zero has happened

	//store back to the memory
	VSTR.F32 S3,[R0] //q
	VSTR.F32 S4,[R0, #4] //r
	VSTR.F32 S1,[R0, #8] //x
	VSTR.F32 S5,[R0, #12] //p
	VSTR.F32 S6,[R0, #16] //k

ERROR:
	MOV R0, #-1	//loading -1 into R0 to show there is an error FPSCR


END:	//end label

	VPOP.32 {S0-S7} //popping all pushed in registers

	BX LR	//end and return
