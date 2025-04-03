  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global  Main

  @ Definitions are in definitions.s to keep blinky.s "clean"
  .include "definitions.s"

  .equ    BLINK_PERIOD, 500

  .section .text

Main:
  PUSH    {R4-R6,LR}

  @ Enable GPIO port E by enabling its clock
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ We'll blink LED LD3 (the orange LED)

  @ Configure LD3 for output
  @ by setting bits 27:26 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @ (by BIClearing then ORRing)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #0xFC00F000  @ Modify ...
  ORR     R5, #0x54005000  @ write 01 to bits 
  STR     R5, [R4]                  @ Write 
  LDR     R6, #8
  MOV     R1, #0x1 
  MOV R7, #1

  BL randomNumberGenerator
  MOV R8, R0 @R8 = random number

  @ Loop forever
.LwhBlink:
  @ Invert LD3
  @ by inverting bit 13 of GPIOE_ODR (GPIO Port E Output Data Register)
  @ (by using EOR to invert bit 13, leaving other bits unchanged)
  CMP     R6, #15
  BGT     .LResetPin

  CMP R7, R8 @r7= count, R8= random number
  BGT .LEndLights

  .LResetReturn:
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                  @ Read ...
  EOR     R5, R5, R1, LSL R6 
  STR     R5, [R4]                  @ Write

  @ wait for 1s ...
  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                  @ Read ...
  EOR     R5, R5, R1, LSL R6 
  STR     R5, [R4]

  @ wait for 1s ...
  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  @ ... and repeat
  ADD R6, R6, #1
  ADD R7, R7, #1
  B       .LwhBlink
  
  .LResetPin:
  LDR     R6, #(LD3_PIN)
  B .LResetReturn

  .LEndLights

End_Main:
  POP   {R4-R6,PC}


@ delay_ms subroutine
@ Use the Cortex SysTick timer to wait for a specified number of milliseconds
@
@ See Yiu, Joseph, "The Definitive Guide to the ARM Cortex-M3 and Cortex-M4
@   Processors", 3rd edition, Chapter 9.
@
@ Parameters:
@   R0: delay - time to wait in ms
@
@ Return:
@   None
delay_ms:
  PUSH  {R4-R5,LR}

  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR   R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR   R5, =7999                   @ Assuming a 8MHz clock
  STR   R5, [R4]                    @ 
  
  LDR   R4, =SYSTICK_VAL            @ Reset SysTick internal counter to 0
  LDR   R5, =0x1                    @   by writing any value
  STR   R5, [R4]  

  LDR   R4, =SYSTICK_CSR            @ Start SysTick timer by setting CSR to 0x5
  LDR   R5, =0x5                    @   set CLKSOURCE (bit 2) to system clock (1)
  STR   R5, [R4]                    @   set ENABLE (bit 0) to 1

.LwhDelay:                          @ while (delay != 0) {
  CMP   R0, #0  
  BEQ   .LendwhDelay  
  
.Lwait:
  LDR   R5, [R4]                    @   Repeatedly load the CSR and check bit 16
  AND   R5, #0x10000                @   Loop until bit 16 is 1, indicating that
  CMP   R5, #0                      @     the SysTick internal counter has counted
  BEQ   .Lwait                      @     from 0x3E7F down to 0 and 1ms has elapsed 

  SUB   R0, R0, #1                  @   delay = delay - 1
  B     .LwhDelay                   @ }

.LendwhDelay:

  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  POP   {R4-R5,PC}

  .end