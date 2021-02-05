#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"
#define MPU_SAMPLE_RATE_REG		0X19
#define MPU_PWR_MGMT1_REG	 0X6B	
#define  MPU_PWR_MGMT2_REG	 0X6C   
#define  MPU_INT_EN_REG      0X38  
#define  MPU_USER_CTRL_REG   0X6A	
#define MPU_FIFO_EN_REG       0X23	
#define MPU_INTBP_CFG_REG	 0X37	
#define  MPU_ADDR            0X68
#define  MPU_ACCEL_CFG_REG	 0X1C	
#define  MPU_ACCEL_ZOUTL_REG 0X40	
#define MPU_CFG_REG				0X1A
.bss
    .global ac,ay,az,acl,azl,p_ax,p_az,m_ax,m_az
ac:	.long 0
ay:	.long 0
az: .long 0
acl:.long 0
azl:.long 0
p_ax:.long 0
p_az:.long 0
m_ax:.long 0
m_az:.long 0

.text
.global init_mpu
init_mpu:
	MOVE r1, MPU_ADDR
	push r1
	move r1, MPU_PWR_MGMT1_REG
	push r1
	move r1, 0x80
	push r1
    psr
	JUMP write8
    ADD r3,r3,3 

	move r2,500
	psr
	jump waitMs
    MOVE r1, MPU_ADDR
	push r1
	move r1, MPU_PWR_MGMT1_REG
	push r1
	move r1, 0x00   
	push r1
    psr
	JUMP write8
	ADD r3,r3,3 
    psr
	jump didinit
    
	ret
sleepmode:
    MOVE r1, MPU_ADDR
	push r1
	move r1, MPU_PWR_MGMT1_REG
	push r1
	move r1, 0x41
	push r1
    psr
	JUMP write8
    ADD r3,r3,3 
	ret
waitMs:
   wait 8000
   sub r2,r2,1
   jump doneWaitMs,eq
   jump waitMs
doneWaitMs:
   ret
fail:
	move r2,1
	ret

didinit:
	
	MOVE r1, MPU_ADDR
	push r1
	move r1, MPU_INT_EN_REG 
	push r1
	move r1, 0x00
	push r1
    psr
	JUMP write8

	ADD r3,r3,3 
	MOVE r1, MPU_ADDR
	push r1
	move r1, MPU_SAMPLE_RATE_REG  
	push r1
	move r1,50
	push r1
    psr
	JUMP write8
	ADD r3,r3,3 
	MOVE r1, MPU_ADDR
	push r1
	move r1, MPU_CFG_REG  
	push r1
	move r1, 25
	push r1
    psr
	JUMP write8
	ADD r3,r3,3 
	MOVE r1, MPU_ADDR
	push r1
	move r1, MPU_PWR_MGMT1_REG  
	push r1
	move r1, 0x01
	push r1
    psr
	JUMP write8
	ADD r3,r3,3
	MOVE r1, MPU_ADDR
	push r1
	move r1, MPU_PWR_MGMT2_REG  
	push r1
	move r1, 0x07
	push r1
    psr
	JUMP write8
	ADD r3,r3,3
    psr
    jump read

  read:
    move r2,500
	psr
	jump waitMs
    move r1, MPU_ADDR
    push r1
	
    MOVE r1, 0x3b
    push r1
    push r1
    psr
    jump read_intro
	add r3,r3,3

	move r2,0
	psr
	jump i2c_read_byte
	move r2, ac
    st r0,r2,0
	move r2,0 
	psr
	jump i2c_read_byte
	move r2, acl
	st r0,r2,0
	move r2,0 
	psr
	jump i2c_read_byte
	//move r2, ay
	//st r0,r2,0
	move r2,0 
	psr
	jump i2c_read_byte
	//move r2, ay
	//st r0,r2,0
	move r2,0 
	psr
	jump i2c_read_byte
	move r2, az
	st r0,r2,0
	move r2,0 
	psr
	jump i2c_read_byte
	move r2, azl
	st r0,r2,0
    psr
	jump i2c_stop_cond
	//psr
	jump sleepmode
	//WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 12, 1, 0)
    jump comparex

	

  




