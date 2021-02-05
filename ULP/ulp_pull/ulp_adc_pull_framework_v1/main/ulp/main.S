/* ULP Example: using ADC in deep sleep

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program
   measures input voltage on the given ADC channel 'adc_oversampling_factor'
   times. Measurements are accumulated and average value is calculated.
   Average value is compared to the two thresholds: 'low_thr' and 'high_thr'.
   If the value is less than 'low_thr' or more than 'high_thr', ULP wakes up
   the chip from deep sleep.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files 
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"

  .bss
	.global moving_times
     moving_times:	.long 0
     .global sysrun_times
     sysrun_times:	.long 0
	.global move_flags
     move_flags:	.long 0
	 .global compare_times
     compare_times:	.long 0
	.global stack
stack:
	.skip 100
	.global stackEnd
stackEnd:
	.long 0
	.text
	.global entry
entry:
   move r3,stackEnd

   move r1,sysrun_times 
   ld r0,r1,0
   add r0,r0,1
   st r0,r1,0

   move r2,compare_times
   ld r2,r2,0
   sub r2,r2,r0
   jump iswake,ov
    psr 
    jump init_mpu

	.global comparex
comparex:
    move r1,ac
	ld r0,r1,0
    lsh r0,r0,8
	move r1,acl
	ld r1,r1,0
	or r0,r1,r0

	move r2,p_ax
	ld r2,r2,0
	sub r2,r2,r0
    jump movecount,ov


	move r2,m_ax
	ld r2,r2,0
	sub r2,r0,r2
    jump movecount,ov

    jump comparez
	

   
		
comparez: 
    move r1,az
	ld r0,r1,0
    lsh r0,r0,8
	move r1,azl
	ld r1,r1,0
	or r0,r1,r0
	
	move r2,p_az
	ld r2,r2,0
	sub r2,r2,r0
    jump movecount,ov

	move r2,m_az
	ld r2,r2,0
	sub r2,r0,r2
    jump movecount,ov

    jump exit
	

noNegate:
	ret
		.global exit
exit:
	 halt
	.global wake_up
wake_up:
    
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump exit, eq
	wake
	halt
	
iswake:
	move r1,sysrun_times
	move r0,0
	st r0,r1,0

	move r1,moving_times
	ld r0,r1,0
	jumpr ismoveflag,3,ge

	move r1,move_flags
	move r0,0
	st r0,r1,0
    jump claertimes
movecount:
  move r1,moving_times 
  ld r0,r1,0
  add r0,r0,1
  st r0,r1,0
  jump exit
ismoveflag:
     move r1,move_flags
	 ld r0,r1,0
	 jumpr setflag,1,lt
	 jump claertimes
claertimes:
     move r1,moving_times
	 move r0,0
	 st r0,r1,0
	 jump exit
setflag:
     move r1,move_flags
	 move r0,1
	 st r0,r1,0
	move r1,moving_times
	move r0,0
	 st r0,r1,0
	 jump wake_up



   
   
