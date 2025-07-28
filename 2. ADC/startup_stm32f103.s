/*======== startup_stm32f103.s ===========
      - Định nghĩa vector table cho STM32F103
      - Copy .data từ Flash vào RAM, clear .bss
      - Gọi main(), vào vòng lặp vô hạn nếu main() trả về
    ==========================================*/

    .syntax unified
    .cpu cortex-m3
    .thumb

/* ========= Vector Table ========= */
    .section .isr_vector, "a", %progbits
    .align  2
    .type   g_pfnVectors, %object
    .size   g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word   _estack                 /* 0x00: Initial Stack Pointer */
    .word   Reset_Handler           /* 0x04: Reset Handler */
    .word   NMI_Handler             /* 0x08: NMI Handler */
    .word   HardFault_Handler       /* 0x0C: HardFault Handler */
    .word   MemManage_Handler       /* 0x10: MemManage Handler */
    .word   BusFault_Handler        /* 0x14: BusFault Handler */
    .word   UsageFault_Handler      /* 0x18: UsageFault Handler */
    .word   0                        /* 0x1C: Reserved */
    .word   0                        /* 0x20: Reserved */
    .word   0                        /* 0x24: Reserved */
    .word   0                        /* 0x28: Reserved */
    .word   SVC_Handler             /* 0x2C: SVCall Handler */
    .word   DebugMon_Handler        /* 0x30: DebugMon Handler */
    .word   0                        /* 0x34: Reserved */
    .word   PendSV_Handler          /* 0x38: PendSV Handler */
    .word   SysTick_Handler         /* 0x3C: SysTick Handler */

/* ========= Các interrupt handlers ========= */    ; .word   WWDG_IRQHandler         /* 0x40: Window Watchdog */
    .word   PVD_IRQHandler          /* 0x44: PVD through EXTI Line detect */
    .word   TAMPER_IRQHandler       /* 0x48: Tamper */
    .word   RTC_IRQHandler          /* 0x4C: RTC global interrupt */
    .word   FLASH_IRQHandler        /* 0x50: Flash global interrupt */  
    .word   RCC_IRQHandler          /* 0x54: RCC global interrupt */
    .word   EXTI0_IRQHandler        /* 0x58: EXTI Line0 */
    .word   EXTI1_IRQHandler        /* 0x5C: EXTI Line1 */
    .word   EXTI2_IRQHandler        /* 0x60: EXTI Line2 */  
    .word   EXTI3_IRQHandler        /* 0x64: EXTI Line3 */
    .word   EXTI4_IRQHandler        /* 0x68: EXTI Line4 */
    .word   DMA1_Channel1_IRQHandler /* 0x6C: DMA1 Channel1 */
    .word   DMA1_Channel2_IRQHandler /* 0x70: DMA1 Channel2 */
    .word   DMA1_Channel3_IRQHandler /* 0x74: DMA1 Channel3 */      
    .word   DMA1_Channel4_IRQHandler /* 0x78: DMA1 Channel4 */
    .word   DMA1_Channel5_IRQHandler /* 0x7C: DMA1 Channel5 */
    .word   DMA1_Channel6_IRQHandler /* 0x80: DMA1 Channel6 */
    .word   DMA1_Channel7_IRQHandler /* 0x84: DMA1 Channel7 */
    .word   ADC1_2_IRQHandler       /* 0x88: ADC1 and   ADC2 global interrupt */
    .word   USB_HP_CAN1_TX_IRQHandler /* 0x8C: USB High Priority or CAN1 TX */
    .word   USB_LP_CAN1_RX0_IRQHandler /* 0x90: USB Low Priority or CAN1 RX0 */
    .word   CAN1_RX1_IRQHandler     /* 0x94: CAN1 RX1 */
    .word   CAN1_SCE_IRQHandler     /* 0x98: CAN1 SCE */
    .word   EXTI9_5_IRQHandler       /* 0x9C: External  Line[9:5] */
    .word   TIM1_BRK_IRQHandler      /* 0xA0: TIM1 Break */
    .word   TIM1_UP_IRQHandler       /* 0xA4: TIM1 Update */
    .word   TIM1_TRG_COM_IRQHandler  /* 0xA8: TIM1 Trigger and Commutation */
    .word   TIM1_CC_IRQHandler       /* 0xAC: TIM1 Capture Compare */
    .word   TIM2_IRQHandler          /* 0xB0: TIM2 global interrupt */
    .word   TIM3_IRQHandler          /* 0xB4: TIM3 global interrupt */
    .word   TIM4_IRQHandler          /* 0xB8: TIM4 global interrupt */
    .word   I2C1_EV_IRQHandler       /* 0xBC: I2C1 Event */
    .word   I2C1_ER_IRQHandler       /* 0xC0: I2C1 Error */
    .word   I2C2_EV_IRQHandler       /* 0xC4: I2C2 Event */
    .word   I2C2_ER_IRQHandler       /* 0xC8: I2C2 Error */
    .word   SPI1_IRQHandler          /* 0xCC: SPI1 global interrupt */
    .word   SPI2_IRQHandler          /* 0xD0: SPI2 global interrupt */
    .word   USART1_IRQHandler        /* 0xD4: USART1 global interrupt */
    .word   USART2_IRQHandler        /* 0xD8: USART2 global interrupt */
    .word   USART3_IRQHandler        /* 0xDC: USART3 global interrupt */
    .word   EXTI15_10_IRQHandler     /* 0xE0: External Line[15:10] */
    .word   RTCAlarm_IRQHandler      /* 0xE4: RTC Alarm through EXTI Line */
    .word   USBWakeUp_IRQHandler     /* 0xE8: USB Wakeup from suspend */


/* ========= Default Handler (vòng lặp vô hạn) ========= */
    .section .text.Default_Handler, "ax", %progbits
    .weak   Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    b   Default_Handler

/* ========= Weak aliases cho tất cả các interrupt handlers ========= */
/* Nếu user không định nghĩa riêng, chúng sẽ trỏ về Default_Handler */
    .weak   NMI_Handler
    .set    NMI_Handler, Default_Handler

    .weak   HardFault_Handler
    .set    HardFault_Handler, Default_Handler

    .weak   MemManage_Handler
    .set    MemManage_Handler, Default_Handler

    .weak   BusFault_Handler
    .set    BusFault_Handler, Default_Handler

    .weak   UsageFault_Handler
    .set    UsageFault_Handler, Default_Handler

    .weak   SVC_Handler
    .set    SVC_Handler, Default_Handler

    .weak   DebugMon_Handler
    .set    DebugMon_Handler, Default_Handler

    .weak   PendSV_Handler
    .set    PendSV_Handler, Default_Handler

    .weak   SysTick_Handler
    .set    SysTick_Handler, Default_Handler

    .weak   WWDG_IRQHandler
    .set    WWDG_IRQHandler, Default_Handler

    .weak   PVD_IRQHandler
    .set    PVD_IRQHandler, Default_Handler

    .weak   TAMPER_IRQHandler
    .set    TAMPER_IRQHandler, Default_Handler

    .weak   RTC_IRQHandler
    .set    RTC_IRQHandler, Default_Handler
    .weak   FLASH_IRQHandler
    .set    FLASH_IRQHandler, Default_Handler
    .weak   RCC_IRQHandler
    .set    RCC_IRQHandler, Default_Handler
    .weak   EXTI0_IRQHandler
    .set    EXTI0_IRQHandler, Default_Handler
    .weak   EXTI1_IRQHandler
    .set    EXTI1_IRQHandler, Default_Handler
    .weak   EXTI2_IRQHandler
    .set    EXTI2_IRQHandler, Default_Handler
    .weak   EXTI3_IRQHandler
    .set    EXTI3_IRQHandler, Default_Handler
    .weak   EXTI4_IRQHandler
    .set    EXTI4_IRQHandler, Default_Handler
    .weak   DMA1_Channel1_IRQHandler
    .set    DMA1_Channel1_IRQHandler, Default_Handler
    .weak   DMA1_Channel2_IRQHandler
    .set    DMA1_Channel2_IRQHandler, Default_Handler
    .weak   DMA1_Channel3_IRQHandler
    .set    DMA1_Channel3_IRQHandler, Default_Handler
    .weak   DMA1_Channel4_IRQHandler    
    .set    DMA1_Channel4_IRQHandler, Default_Handler
    .weak   DMA1_Channel5_IRQHandler
    .set    DMA1_Channel5_IRQHandler, Default_Handler
    .weak   DMA1_Channel6_IRQHandler
    .set    DMA1_Channel6_IRQHandler, Default_Handler
    .weak   DMA1_Channel7_IRQHandler
    .set    DMA1_Channel7_IRQHandler, Default_Handler
    .weak   ADC1_2_IRQHandler
    .set    ADC1_2_IRQHandler, Default_Handler
    .weak   USB_HP_CAN1_TX_IRQHandler
    .set    USB_HP_CAN1_TX_IRQHandler, Default_Handler

    .weak   USB_LP_CAN1_RX0_IRQHandler
    .set    USB_LP_CAN1_RX0_IRQHandler, Default_Handler
    .weak   CAN1_RX1_IRQHandler
    .set    CAN1_RX1_IRQHandler, Default_Handler
    .weak   CAN1_SCE_IRQHandler
    .set    CAN1_SCE_IRQHandler, Default_Handler
    .weak   EXTI9_5_IRQHandler
    .set    EXTI9_5_IRQHandler, Default_Handler
    .weak   TIM1_BRK_IRQHandler
    .set    TIM1_BRK_IRQHandler, Default_Handler
    .weak   TIM1_UP_IRQHandler
    .set    TIM1_UP_IRQHandler, Default_Handler
    .weak   TIM1_TRG_COM_IRQHandler
    .set    TIM1_TRG_COM_IRQHandler, Default_Handler
    .weak   TIM1_CC_IRQHandler
    .set    TIM1_CC_IRQHandler, Default_Handler
    .weak   TIM2_IRQHandler
    .set    TIM2_IRQHandler, Default_Handler
    .weak   TIM3_IRQHandler
    .set    TIM3_IRQHandler, Default_Handler
    .weak   TIM4_IRQHandler
    .set    TIM4_IRQHandler, Default_Handler
    .weak   I2C1_EV_IRQHandler
    .set    I2C1_EV_IRQHandler, Default_Handler
    .weak   I2C1_ER_IRQHandler
    .set    I2C1_ER_IRQHandler, Default_Handler
    .weak   I2C2_EV_IRQHandler
    .set    I2C2_EV_IRQHandler, Default_Handler
    .weak   I2C2_ER_IRQHandler
    .set    I2C2_ER_IRQHandler, Default_Handler
    .weak   SPI1_IRQHandler
    .set    SPI1_IRQHandler, Default_Handler
    .weak   SPI2_IRQHandler
    .set    SPI2_IRQHandler, Default_Handler
    .weak   USART1_IRQHandler
    .set    USART1_IRQHandler, Default_Handler
    .weak   USART2_IRQHandler
    .set    USART2_IRQHandler, Default_Handler
    .weak   USART3_IRQHandler
    .set    USART3_IRQHandler, Default_Handler
    .weak   EXTI15_10_IRQHandler
    .set    EXTI15_10_IRQHandler, Default_Handler
    .weak   RTCAlarm_IRQHandler
    .set    RTCAlarm_IRQHandler, Default_Handler
    .weak   USBWakeUp_IRQHandler
    .set    USBWakeUp_IRQHandler, Default_Handler
    

/* ========= Reset Handler ========= */
    .section .text.Reset_Handler, "ax", %progbits
    .weak   Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:
    /* 1/ Copy .data từ Flash sang RAM */
    LDR   R0, =_sidata      /* _sidata = địa chỉ đầu của vùng gốc .data trong Flash */
    LDR   R1, =_sdata       /* _sdata = địa chỉ đầu vùng .data trong RAM */
    LDR   R2, =_edata       /* _edata = địa chỉ kết thúc vùng .data trong RAM */
copy_data_loop:
    CMP   R1, R2            /* nếu R1 >= R2 thì dừng */
    ITT   LT
    LDRLT R3, [R0], #4      /* load 4 byte tại R0, R0 += 4 */
    STRLT R3, [R1], #4      /* store 4 byte vào R1, R1 += 4 */
    BLT   copy_data_loop

    /* 2/ Clear .bss (set 0) */
    LDR   R0, =_sbss        /* _sbss = địa chỉ đầu của vùng .bss trong RAM */
    LDR   R1, =_ebss        /* _ebss = địa chỉ kết thúc vùng .bss trong RAM */
    MOV   R2, #0
clear_bss_loop:
    CMP   R0, R1            /* nếu R0 >= R1 thì dừng */
    ITT   LT
    STRLT R2, [R0], #4      /* store 0 vào [R0], R0 += 4 */
    BLT   clear_bss_loop

    /* 3/ Gọi hàm main() */
    BL    main

    /* 4/ Nếu main() trả về, vào vòng lặp vô hạn */
infinite_loop:
    B    infinite_loop

    .size Reset_Handler, .-Reset_Handler