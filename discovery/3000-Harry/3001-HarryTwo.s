.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global Main
.global EXTI0_IRQHandler

.include "definitions.s"

.equ    BLINK_PERIOD, 200

.section .text

Main:

Setup:

  PUSH  {LR}
  mov r4, #0
  ldr r5, =buttonCount2
  str r4, [r5]
  @ Enable SYSCFG clock (needed for EXTI configuration)
  LDR   R4, =RCC_AHBENR
  LDR   R5, [R4]
  ORR   R5, R5, #(1<<0)   @ Enable SYSCFG clock (bit 0 of APB2ENR)
  STR   R5, [R4]

  @ Configure USER pushbutton (GPIO Port A Pin 0) to use EXTI0
  LDR   R4, =SYSCFG_EXTIICR1
  LDR   R5, [R4]
  BIC   R5, R5, #0b1111   @ Clear bits 3:0 to select PA0
  STR   R5, [R4]

  @ Enable (unmask) interrupts on EXTI0
  LDR   R4, =EXTI_IMR
  LDR   R5, [R4]
  ORR   R5, R5, #1        @ Set bit 0 for EXTI0
  STR   R5, [R4]

  @ Set falling edge detection on EXTI0
  LDR   R4, =EXTI_FTSR
  LDR   R5, [R4]
  ORR   R5, R5, #1        @ Set bit 0 for EXTI0
  STR   R5, [R4]

  @ Enable NVIC interrupt for EXTI0 (IRQ 6)
  LDR   R4, =NVIC_ISER
  MOV   R5, #(1<<6)       @ Set bit 6 for EXTI0 IRQ
  STR   R5, [R4]

  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR   R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR   R5, =0xFFFFFFFFFFFFFFFF     @ Assuming a 8MHz clock
  STR   R5, [R4]                    @ 

  LDR   R4, =SYSTICK_CSR            @ Start SysTick timer by setting CSR to 0x5
  LDR   R5, =0x5                    @   set CLKSOURCE (bit 2) to system clock (1)
  STR   R5, [R4]                    @   set ENABLE (bit 0) to 1

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
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]
  LDR     R6, =0xFFFF0000        @ Clear MODER bits for pins 8–15
  BIC     R5, R6
  LDR     R6, =0x55550000        @ Set all pins 8–15 to output (0b01)
  ORR     R5, R6
  STR     R5, [R4]
  MOV     R6, #(LD4_PIN)
  
  MOV     R0, #0
  LDR     R4, =game
  STR     R0, [R4]
EndSetup:

Idle_Loop:
  B     Idle_Loop


Lights:

  PUSH  {R4-R5, LR}

  MOV   R8, R0
  LDR   R4, =game
  STR   R8, [R4]

  MOV   R6, #(LD4_PIN)
  MOV   R7, #0

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
  MOV     R12, #1
  LSL     R12, R12, R6     @ R12 = 1 << R6
  EOR     R5, R12         @ Modify ...
  STR     R5, [R4]                  @ Write

  @ wait for 1s ...
  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                  @ Read ...
  MOV     R12, #1
  LSL     R12, R12, R6     @ R12 = 1 << R6
  EOR     R5, R12         @ Modify ...
  STR     R5, [R4]  

  LDR     R0, =BLINK_PERIOD
  BL      delay_ms


  @ wait for 1s ...
  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  @ ... and repeat
  ADD R6, R6, #1
  ADD R7, R7, #1
  B       .LwhBlink
  
  .LResetPin:
  MOV     R6, #(LD4_PIN)
  B .LResetReturn

  .LEndLights:

  POP   {R4-R5, PC}

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
  LDR   R5, =4999                   @ Assuming a 8MHz clock
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


@ External interrupt line 0 (EXTI0) interrupt handler
.type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:
  PUSH  {R4-R5, LR}
  ldr   r4, =buttonCount2  @load current count
  ldr   r5, [r4] @load current count from memory
  add r5, r5, #1
  str  r5, [r4]  @store the count back in memory

  LDR   R4, =random
  LDR   R5, [R4]
  CMP   R5, #0
  BNE   Skip

  LDR   R4, =SYSTICK_VAL
  LDR   R5, [R4]
  LDR   R4, =random
  STR   R5, [R4]

Skip:
  @ Clear the EXTI0 pending flag
  LDR   R4, =EXTI_PR      @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)       @ Only clear bit 0
  STR   R5, [R4]          @

  LDR   R4, =game
  LDR   R3, [R4]
  CMP   R3, #0
  BNE   .Lcounter

  LDR   R4, =random
  LDR   R5, [R4]
  AND   R0, R5, 0b111
  LSR   R5, R5, #3
  STR   R5, [R4]
  BL    Lights
  B     .LendOfButtonCount

.Lcounter:
  LDR   R4, =buttonCount
  LDR   R3, [R4]
  ADD   R3, R3, #1
  LDR   R4, =game
  LDR   R5, [R4]
  CMP   R3, R5
  BLT   .LnotDone
  MOV   R3, #0
  STR   R3, [R4]

.LnotDone:
  LDR   R4, =buttonCount
  STR   R3, [R4]

.LendOfButtonCount:

  POP  {R4-R5, PC}

.section .data
  
random:
  .space  4

game:
  .space  4
  
buttonCount:
  .space  4

buttonCount2:
  .space  4

.end