/*
 * ULP stack and subroutine macros
 */


.macro push rx
	st \rx,r3,0
	sub r3,r3,1
.endm

.macro pop rx
	add r3,r3,1
	ld \rx,r3,0
.endm

// Prepare subroutine jump
.macro psr
	.set addr,(.+16)
	move r1,addr
	push r1
.endm

// Return from subroutine
.macro ret
	pop r1
	jump r1
.endm





