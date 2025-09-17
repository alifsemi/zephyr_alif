/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_H_
#define _SOC_H_

#include "stdint.h"

#define __MPU_PRESENT 1

#define __NVIC_PRIO_BITS          NUM_IRQ_PRIO_BITS

#define __MVE_FP                  1U        /* MVE floating point present */

/* Host Base System Control Registers */
#define HOST_BASE_SYS_CTRL	0x1A010000
#define HOST_BSYS_PWR_REQ	(HOST_BASE_SYS_CTRL + 0x400)
#define HOST_BSYS_PWR_ST	(HOST_BASE_SYS_CTRL + 0x404)

/* AON registers. */
#define AON_BASE                        0x1A604000UL
#define AON_RTSS_HE_CTRL                (AON_BASE + 0x10)
#define AON_RTSS_HE_RESET               (AON_BASE + 0x14)
#define AON_RTSS_HE_LPUART_CKEN         (AON_BASE + 0x1C)

/* VBAT Modules */
#define VBAT_BASE                       0x1A609000UL
#define VBAT_PWR_CTRL                   (VBAT_BASE + 0x8)
#define LPRTC0_CLK_EN                   (VBAT_BASE + 0x10)
#define LPRTC1_CLK_EN                   (VBAT_BASE + 0x14)

/* Peripheral CLKCTRL */
#define CLKCTRL_PER_SLV                 0X4902F000UL
#define UART_CLK_EN                     (CLKCTRL_PER_SLV + 0x8)

/* CGU Module */
#define CGU_BASE			0x1A602000
#define CGU_CLK_ENA			(CGU_BASE + 0x14)

/* ANA Register */
#define ANA_BASE			0x1A60A000
#define ANA_VBAT_REG1                   (ANA_BASE + 0x38)
#define ANA_VBAT_REG2			(ANA_BASE + 0x3C)

/* Expansion Slave registers. */
#define EXPSLV_BASE			0x4902F000
#define EXPSLV_UART_CTRL		(EXPSLV_BASE + 0x8)
#define EXPSLV_CANFD_CTRL		(EXPSLV_BASE + 0xC)
#define EXPSLV_SSI_CTRL			(EXPSLV_BASE + 0x28)
#define EXPSLV_ADC_CTRL			(EXPSLV_BASE + 0x30)
#define EXPSLV_DAC_CTRL			(EXPSLV_BASE + 0x34)
#define EXPSLV_CMP_CTRL			(EXPSLV_BASE + 0x38)
#define EXPSLV_OSPI_CTRL		(EXPSLV_BASE + 0x3C)

/* Expansion Master-0 registers. */
#define EXPMST_BASE			0x4903F000
#define EXPMST_CDC200_PIXCLK_CTRL	(EXPMST_BASE + 0x4)
#define EXPMST_PERIPH_CLK_EN		(EXPMST_BASE + 0xC)

/* M55-HE Config Registers */
#define M55HE_CFG_BASE			0x43007000UL
#define HE_PER_CLK_EN			(M55HE_CFG_BASE + 0x10)
#define HE_CAMERA_PIXCLK		(M55HE_CFG_BASE + 0x20)

/* VBAT Modules */
#define VBAT_REG_BASE			0x1A609000UL
#define VBAT_GPIO_CTRL_EN		(VBAT_REG_BASE + 0x0)

/* LPGPIO Base address for LPTIMER pin config */
#define LPGPIO_BASE			0x42002008UL

/* Event Router registers. */
#define EVTRTRLOCAL_BASE		0x400E2000
#define EVTRTRLOCAL_DMA_CTRL0		(EVTRTRLOCAL_BASE)
#define EVTRTRLOCAL_DMA_REQ_CTRL	(EVTRTRLOCAL_BASE + 0x80)
#define EVTRTRLOCAL_DMA_ACK_TYPE0	(EVTRTRLOCAL_BASE + 0x90)

/* M55-HE Config registers. */
#define M55HE_CFG_HE_CFG_BASE		0x43007000
#define M55HE_CFG_HE_DMA_CTRL		(M55HE_CFG_HE_CFG_BASE)
#define M55HE_CFG_HE_DMA_IRQ		(M55HE_CFG_HE_CFG_BASE + 0x4)
#define M55HE_CFG_HE_DMA_PERIPH		(M55HE_CFG_HE_CFG_BASE + 0x8)
#define M55HE_CFG_HE_DMA_SEL		(M55HE_CFG_HE_CFG_BASE + 0xC)
#define M55HE_CFG_HE_CLK_ENA		(M55HE_CFG_HE_CFG_BASE + 0x10)
#define M55HE_CFG_HE_CAMERA_PIXCLK	(M55HE_CFG_HE_CFG_BASE + 0x20)

/* Macros for CAN FD mode Control */
#define CAN0_FD_CTRL_REG          EXPSLV_CANFD_CTRL
#define CAN0_CTRL_FD_ENA_POS      10U
#define CAN1_FD_CTRL_REG          EXPSLV_CANFD_CTRL
#define CAN1_CTRL_FD_ENA_POS      26U

/*
 * CMSIS IRQn_Type enum is broken relative to ARM GNU compiler.
 *
 * So redefine the IRQn_Type enum to a unsigned int to avoid
 * the ARM compiler from sign extending IRQn_Type values higher than 0x80
 * into negative IRQ values, which causes hard-to-debug Hard Faults.
 */
typedef uint32_t IRQn_Type;

typedef enum IRQn {
/* -------------------  Processor Exceptions Numbers  ----------------------------- */
	Reset_IRQn                    = -15,
	NonMaskableInt_IRQn           = -14,     /*  2 Non Maskable Interrupt */
	HardFault_IRQn                = -13,     /*  3 HardFault Interrupt */
	MemoryManagement_IRQn         = -12,     /*  4 Memory Management Interrupt */
	BusFault_IRQn                 = -11,     /*  5 Bus Fault Interrupt */
	UsageFault_IRQn               = -10,     /*  6 Usage Fault Interrupt */
	SecureFault_IRQn              =  -9,     /*  7 Secure Fault Interrupt */
	SVCall_IRQn                   =  -5,     /* 11 SV Call Interrupt */
	DebugMonitor_IRQn             =  -4,     /* 12 Debug Monitor Interrupt */
	PendSV_IRQn                   =  -2,     /* 14 Pend SV Interrupt */
	SysTick_IRQn                  =  -1,     /* 15 System Tick Interrupt */

/* ---  Processor Interrupt Numbers  --- */
	DMA2_IRQ0                     =   0,
	DMA2_IRQ1                     =   1,
	DMA2_IRQ2                     =   2,
	DMA2_IRQ3                     =   3,
	DMA2_IRQ4                     =   4,
	DMA2_IRQ5                     =   5,
	DMA2_IRQ6                     =   6,
	DMA2_IRQ7                     =   7,
	DMA2_IRQ8                     =   8,
	DMA2_IRQ9                     =   9,
	DMA2_IRQ10                    =   10,
	DMA2_IRQ11                    =   11,
	DMA2_IRQ12                    =   12,
	DMA2_IRQ13                    =   13,
	DMA2_IRQ14                    =   14,
	DMA2_IRQ15                    =   15,
	DMA2_IRQ16                    =   16,
	DMA2_IRQ17                    =   17,
	DMA2_IRQ18                    =   18,
	DMA2_IRQ19                    =   19,
	DMA2_IRQ20                    =   20,
	DMA2_IRQ21                    =   21,
	DMA2_IRQ22                    =   22,
	DMA2_IRQ23                    =   23,
	DMA2_IRQ24                    =   24,
	DMA2_IRQ25                    =   25,
	DMA2_IRQ26                    =   26,
	DMA2_IRQ27                    =   27,
	DMA2_IRQ28                    =   28,
	DMA2_IRQ29                    =   29,
	DMA2_IRQ30                    =   30,
	DMA2_IRQ31                    =   31,
	DMA2_ABORT_IRQ                =   32,
	/* IRQs 33 to 36 are Reserved. */
	MHU_SEES10_RX_IRQ             =   37,
	MHU_ES1SE0_TX_IRQ             =   38,
	MHU_SEES11_RX_IRQ             =   39,
	MHU_ES1SE1_TX_IRQ             =   40,
	/* IRQs 41 to 44 are Reserved. */
	LPUART_IRQ                    =   45,
	LPSPI_IRQ                     =   46,
	LPI2C_IRQ                     =   47,
	LPI2S_IRQ                     =   48,
	LPPDM_IRQ                     =   49,
  /* IRQs 50 to 53 are Reserved. */
	LPCPI_IRQ                     =   54,
	ETHOS_U55_1_IRQ               =   55,
	LPCMP_IRQ                     =   56,
	LPGPIO_IRQ                    =   57,
	LPRTC_IRQ                     =   58,
  /* IRQ 59 is reserved. */
	LPTIMER0_IRQ                  =   60,
	LPTIMER1_IRQ                  =   61,
	LPTIMER2_IRQ                  =   62,
	LPTIMER3_IRQ                  =   63,
	FW_IRQ                        =   64,
	SDC600_IRQ                    =   65,
	PPU_COMB_IRQ                  =   66,
	REFCLK_CNTBASE0_IRQ           =   67,
	REFCLK_CNTBASE1_IRQ           =   68,
	REFCLK_CNTBASE2_IRQ           =   69,
	REFCLK_CNTBASE3_IRQ           =   70,
	S32K_CNTBASE0_IRQ             =   71,
	S32K_CNTBASE1_IRQ             =   72,
	SOC_ETR_IRQ                   =   73,
	SOC_CATU_IRQ                  =   74,
  /* IRQs 75 to 95 are Reserved. */
	XNVM_OSPI0_IRQ                =   96,
	XNVM_OSPI1_IRQ                =   97,
	XNVM_AESDEC0_IRQ              =   98,
	XNVM_AESDEC1_IRQ              =   99,
	VBAT_BOD_IRQ                  =   100,
	USB0_IRQ                      =   101,
	SDMMC_IRQ                     =   102,
	SDMMC_WAKEUP_IRQ              =   103,
	CANFD0_IRQ                    =   104,
	CANFD1_IRQ                    =   105,
	PPU0_IRQ                      =   121,
	PPU1_IRQ                      =   122,
	PPU2_IRQ                      =   123,
	UART0_IRQ                     =   124,
	UART1_IRQ                     =   125,
	UART2_IRQ                     =   126,
	UART3_IRQ                     =   127,
	UART4_IRQ                     =   128,
	UART5_IRQ                     =   129,
  /* IRQs 130 and 131 reserved */
	I2C0_IRQ                      =   132,
	I2C1_IRQ                      =   133,
  /* IRQs 134 and 135 reserved */
	I3C0_IRQ                      =   136,
	SPI0_IRQ                      =   137,
	SPI1_IRQ                      =   138,
	SPI2_IRQ                      =   139,
  /* IRQ 140 reserved */
	I2S0_IRQ                      =   141,
	I2S1_IRQ                      =   142,
  /* IRQs 143 to 150 reserved */
	ADC0_INTR_DONE_IRQ            =   151,
	ADC1_INTR_DONE_IRQ            =   152,
  /* IRQ 153 reserved */
	ADC0_INTR_DONE2_IRQ           =   154,
	ADC1_INTR_DONE2_IRQ           =   155,
  /* IRQ 156 reserved */
	ADC0_INTR_CMP1_IRQ            =   157,
	ADC1_INTR_CMP1_IRQ            =   158,
  /* IRQ 159 reserved */
	ADC0_INTR_CMP2_IRQ            =   160,
	ADC1_INTR_CMP2_IRQ            =   161,
  /* IRQ 162 reserved */
	ADC24_INTR_DONE_IRQ           =   163,
	ADC24_INTR_DONE2_IRQ          =   164,
	ADC24_INTR_CMP0_IRQ           =   165,
	ADC24_INTR_CMP1_IRQ           =   166,
	CMP0_IRQ                      =   167,
	CMP1_IRQ                      =   168,
  /* IRQs 169 and 170 reserved */
	LPGPIO_PIN0_IRQ               =   171,
	LPGPIO_PIN1_IRQ               =   172,
  /* IRQs 173 - 178 reserved */
	GPIOX0_PIN0_IRQ               =   179,
	GPIOX0_PIN1_IRQ               =   180,
	GPIOX0_PIN2_IRQ               =   181,
	GPIOX0_PIN3_IRQ               =   182,
	GPIOX0_PIN4_IRQ               =   183,
	GPIOX0_PIN5_IRQ               =   184,
	GPIOX0_PIN6_IRQ               =   185,
	GPIOX0_PIN7_IRQ               =   186,
	GPIOX1_PIN0_IRQ               =   187,
	GPIOX1_PIN1_IRQ               =   188,
	GPIOX1_PIN2_IRQ               =   189,
	GPIOX1_PIN3_IRQ               =   190,
	GPIOX1_PIN4_IRQ               =   191,
	GPIOX1_PIN5_IRQ               =   192,
	GPIOX1_PIN6_IRQ               =   193,
	GPIOX1_PIN7_IRQ               =   194,
	GPIOX2_PIN0_IRQ               =   195,
	GPIOX2_PIN1_IRQ               =   196,
	GPIOX2_PIN2_IRQ               =   197,
	GPIOX2_PIN3_IRQ               =   198,
	GPIOX2_PIN4_IRQ               =   199,
	GPIOX2_PIN5_IRQ               =   200,
	GPIOX2_PIN6_IRQ               =   201,
	GPIOX2_PIN7_IRQ               =   202,
	GPIOX3_PIN0_IRQ               =   203,
	GPIOX3_PIN1_IRQ               =   204,
	GPIOX3_PIN2_IRQ               =   205,
	GPIOX3_PIN3_IRQ               =   206,
	GPIOX3_PIN4_IRQ               =   207,
	GPIOX3_PIN5_IRQ               =   208,
	GPIOX3_PIN6_IRQ               =   209,
	GPIOX3_PIN7_IRQ               =   210,
	GPIOX4_PIN0_IRQ               =   211,
	GPIOX4_PIN1_IRQ               =   212,
	GPIOX4_PIN2_IRQ               =   213,
	GPIOX4_PIN3_IRQ               =   214,
	GPIOX4_PIN4_IRQ               =   215,
	GPIOX4_PIN5_IRQ               =   216,
	GPIOX4_PIN6_IRQ               =   217,
	GPIOX4_PIN7_IRQ               =   218,
	GPIOX5_PIN0_IRQ               =   219,
	GPIOX5_PIN1_IRQ               =   220,
	GPIOX5_PIN2_IRQ               =   221,
	GPIOX5_PIN3_IRQ               =   222,
	GPIOX5_PIN4_IRQ               =   223,
	GPIOX5_PIN5_IRQ               =   224,
	GPIOX5_PIN6_IRQ               =   225,
	GPIOX5_PIN7_IRQ               =   226,
	GPIOX6_PIN0_IRQ               =   227,
	GPIOX6_PIN1_IRQ               =   228,
	GPIOX6_PIN2_IRQ               =   229,
	GPIOX6_PIN3_IRQ               =   230,
	GPIOX6_PIN4_IRQ               =   231,
	GPIOX6_PIN5_IRQ               =   232,
	GPIOX6_PIN6_IRQ               =   233,
	GPIOX6_PIN7_IRQ               =   234,
	GPIOX7_PIN0_IRQ               =   235,
	GPIOX7_PIN1_IRQ               =   236,
	GPIOX7_PIN2_IRQ               =   237,
	GPIOX7_PIN3_IRQ               =   238,
	GPIOX7_PIN4_IRQ               =   239,
	GPIOX7_PIN5_IRQ               =   240,
	GPIOX7_PIN6_IRQ               =   241,
	GPIOX7_PIN7_IRQ               =   242,
	GPIOX8_PIN0_IRQ               =   243,
	GPIOX8_PIN1_IRQ               =   244,
	GPIOX8_PIN2_IRQ               =   245,
	GPIOX8_PIN3_IRQ               =   246,
	GPIOX8_PIN4_IRQ               =   247,
	GPIOX8_PIN5_IRQ               =   248,
	GPIOX8_PIN6_IRQ               =   249,
	GPIOX8_PIN7_IRQ               =   250,
	GPIOX9_PIN0_IRQ               =   251,
	GPIOX9_PIN1_IRQ               =   252,
	GPIOX9_PIN2_IRQ               =   253,
	GPIOX9_PIN3_IRQ               =   254,
	GPIOX9_PIN4_IRQ               =   255,
	GPIOX9_PIN5_IRQ               =   256,
	GPIOX9_PIN6_IRQ               =   257,
	GPIOX9_PIN7_IRQ               =   258,
	/* IRQs 259 - 331 reserved */
	DAVE2_IRQ                     =   332,
	CDC200_SCANLINE0_IRQ          =   333,
	CDC200_SCANLINE1_IRQ          =   334,
	CDC200_FIFO_WARNING0_IRQ      =   335,
	CDC200_FIFO_WARNING1_IRQ      =   336,
	CDC200_FIFO_UNDERRUN0_IRQ     =   337,
	CDC200_FIFO_UNDERRUN1_IRQ     =   338,
	CDC200_BUS_ERROR0_IRQ         =   339,
	CDC200_BUS_ERROR1_IRQ         =   340,
	CDC200_REG_RELOAD0_IRQ        =   341,
	CDC200_REG_RELOAD1_IRQ        =   342,
	MIPI_DSI_IRQ                  =   343,
	/* IRQs 344 - 345 reserved */
	LPTIMER_CHANNEL0_IRQ          =   346,
	LPTIMER_CHANNEL1_IRQ          =   347,
	/* IRQs 348 - 349 reserved */
	LPRTC0_IRQ                    =   350,
	LPRTC_IRQ1                    =   351,
	/* IRQs 352 - 376 reserved */
	UTIMER0_IRQ                   =   377,
	UTIMER1_IRQ                   =   378,
	UTIMER2_IRQ                   =   379,
	UTIMER3_IRQ                   =   380,
	UTIMER4_IRQ                   =   381,
	UTIMER5_IRQ                   =   382,
	UTIMER6_IRQ                   =   383,
	UTIMER7_IRQ                   =   384,
	UTIMER8_IRQ                   =   385,
	UTIMER9_IRQ                   =   386,
	UTIMER10_IRQ                  =   387,
	UTIMER11_IRQ                  =   388,
	UTIMER12_IRQ                  =   389,
	UTIMER13_IRQ                  =   390,
	UTIMER14_IRQ                  =   391,
	UTIMER15_IRQ                  =   392,
	UTIMER16_IRQ                  =   393,
	UTIMER17_IRQ                  =   394,
	UTIMER18_IRQ                  =   395,
	UTIMER19_IRQ                  =   396,
	UTIMER20_IRQ                  =   397,
	UTIMER21_IRQ                  =   398,
	UTIMER22_IRQ                  =   399,
	UTIMER23_IRQ                  =   400,
	UTIMER24_IRQ                  =   401,
	UTIMER25_IRQ                  =   402,
	UTIMER26_IRQ                  =   403,
	UTIMER27_IRQ                  =   404,
	UTIMER28_IRQ                  =   405,
	UTIMER29_IRQ                  =   406,
	UTIMER30_IRQ                  =   407,
	UTIMER31_IRQ                  =   408,
  /* Interrupts 409 .. 480 are left out */
} CMSIS_IRQn_Type;

#define __Vendor_SysTickConfig         0 /* Default to standard SysTick */
#include <cmsis_core_m_defaults.h>
#if defined(CONFIG_PM)
bool balletto_vbat_resume_enabled(void);
void balletto_vbat_resume_enable(void);
#endif


#endif /* _SOC_H_ */
