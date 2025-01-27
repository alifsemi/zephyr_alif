/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_H_
#define _SOC_H_

#define __MPU_PRESENT 1

/* Host Base System Control Registers */
#define HOST_BASE_SYS_CTRL	0x1A010000
#define HOST_BSYS_PWR_REQ	(HOST_BASE_SYS_CTRL + 0x400)
#define HOST_BSYS_PWR_ST	(HOST_BASE_SYS_CTRL + 0x404)

/* CGU registers. */
#define CGU_BASE		    0x1A602000
#define CGU_PLL_CLK_SEL		(CGU_BASE + 0x8)
#define CGU_CLK_ENA			(CGU_BASE + 0x14)

/* AON registers. */
#define AON_BASE                     0x1A604000
#define AON_RTSS_HP_CTRL             (AON_BASE + 0x0)
#define AON_RTSS_HP_RESET            (AON_BASE + 0x4)
#define AON_RTSS_HE_CTRL             (AON_BASE + 0x10)
#define AON_RTSS_HE_RESET            (AON_BASE + 0x14)
#define AON_RTSS_HE_LPUART_CKEN      (AON_BASE + 0x1C)

/* VBAT registers. */
#define VBAT_BASE		0x1A609000
#define VBAT_PWR_CTRL		(VBAT_BASE + 0x8)
#define VBAT_RTC_CLK_EN		(VBAT_BASE + 0x10)

/* Expansion Slave registers. */
#define EXPSLV_BASE		0x4902F000
#define EXPSLV_EXPMST0_CTRL	(EXPSLV_BASE)
#define EXPSLV_UART_CTRL	(EXPSLV_BASE + 0x8)
#define EXPSLV_CANFD_CTRL	(EXPSLV_BASE + 0xC)
#define EXPSLV_SSI_CTRL		(EXPSLV_BASE + 0x28)

#define EVTRTR0_BASE			0x49035000
#define EVTRTR0_DMA_CTRL0		(EVTRTR0_BASE)
#define EVTRTR0_DMA_REQ_CTRL		(EVTRTR0_BASE + 0x80)
#define EVTRTR0_DMA_ACK_TYPE0		(EVTRTR0_BASE + 0x90)

#define EVTRTRLOCAL_BASE			0x400E2000
#define EVTRTRLOCAL_DMA_CTRL0			(EVTRTRLOCAL_BASE)
#define EVTRTRLOCAL_DMA_REQ_CTRL		(EVTRTRLOCAL_BASE + 0x80)
#define EVTRTRLOCAL_DMA_ACK_TYPE0		(EVTRTRLOCAL_BASE + 0x90)

/* Expansion Master-0 registers. */
#define EXPMST_BASE			0x4903F000
#define EXPMST_CAMERA_PIXCLK_CTRL	(EXPMST_BASE)
#define EXPMST_CDC200_PIXCLK_CTRL	(EXPMST_BASE + 0x4)
#define EXPMST_CSI_PIXCLK_CTRL		(EXPMST_BASE + 0x8)
#define EXPMST_PERIPH_CLK_EN		(EXPMST_BASE + 0xC)
#define EXPMST_MIPI_CKEN		(EXPMST_BASE + 0x40)
#define EXPMST_DMA_CTRL			(EXPMST_BASE + 0x70)
#define EXPMST_DMA_IRQ			(EXPMST_BASE + 0x74)
#define EXPMST_DMA_PERIPH		(EXPMST_BASE + 0x78)

/* M55-HE Config registers. */
#define M55HE_CFG_HE_CFG_BASE		0x43007000
#define M55HE_CFG_HE_DMA_CTRL		(M55HE_CFG_HE_CFG_BASE)
#define M55HE_CFG_HE_DMA_IRQ		(M55HE_CFG_HE_CFG_BASE + 0x4)
#define M55HE_CFG_HE_DMA_PERIPH		(M55HE_CFG_HE_CFG_BASE + 0x8)
#define M55HE_CFG_HE_DMA_SEL		(M55HE_CFG_HE_CFG_BASE + 0xC)
#define M55HE_CFG_HE_CLK_ENA		(M55HE_CFG_HE_CFG_BASE + 0x10)
#define M55HE_CFG_HE_CAMERA_PIXCLK	(M55HE_CFG_HE_CFG_BASE + 0x20)

/* M55-HP Config registers. */
#define M55HP_CFG_HP_CFG_BASE		0x400F0000
#define M55HP_CFG_HP_DMA_CTRL		(M55HP_CFG_HP_CFG_BASE)
#define M55HP_CFG_HP_DMA_IRQ		(M55HP_CFG_HP_CFG_BASE + 0x4)
#define M55HP_CFG_HP_DMA_PERIPH		(M55HP_CFG_HP_CFG_BASE + 0x8)
#define M55HP_CFG_HP_DMA_SEL		(M55HP_CFG_HP_CFG_BASE + 0xC)
#define M55HP_CFG_HP_CLK_ENA		(M55HP_CFG_HP_CFG_BASE + 0x10)

#if !defined(CONFIG_SOC_E7_DK_APSS)

#define __SAUREGION_PRESENT       1U        /* SAU regions present */
#define __FPU_PRESENT             CONFIG_CPU_HAS_FPU
#define __DSP_PRESENT             1U        /* DSP extension present */
#define __MVE_PRESENT             1U        /* MVE extensions present */
#define __MVE_FP                  1U        /* MVE floating point present */
#define __ICACHE_PRESENT          1U        /* ICACHE present */
#define __DCACHE_PRESENT          1U        /* DCACHE present */

#define __NVIC_PRIO_BITS          NUM_IRQ_PRIO_BITS

/* ANA Register */
#define ANA_BASE            0x1A60A000
#define ANA_VBAT_REG1       (ANA_BASE + 0x38)
#define ANA_VBAT_REG2       (ANA_BASE + 0x3C)

/* Expansion Slave registers. */
#define EXPSLV_BASE     0x4902F000
#define EXPSLV_ADC_CTRL     (EXPSLV_BASE + 0x30)
#define EXPSLV_CMP_CTRL     (EXPSLV_BASE + 0x38)

/* LPGPIO Base address for LPTIMER pin config */
#define LPGPIO_BASE     0x42002008

/* Macros for CAN FD mode Control */
#define CAN0_FD_CTRL_REG          EXPSLV_CANFD_CTRL
#define CAN0_CTRL_FD_ENA_POS      20U


/*
 * CMSIS IRQn_Type enum is broken relative to ARM GNU compiler.
 *
 * So redefine the IRQn_Type enum to a unsigned int to avoid
 * the ARM compiler from sign extending IRQn_Type values higher than 0x80
 * into negative IRQ values, which causes hard-to-debug Hard Faults.
 */
typedef uint32_t IRQn_Type;

typedef enum IRQn {
/* ---------------  Processor Exceptions Numbers  ------------------------- */
	Reset_IRQn                 = -15,
	NonMaskableInt_IRQn        = -14, /*  2 Non Maskable Interrupt */
	HardFault_IRQn             = -13, /*  3 HardFault Interrupt */
	MemoryManagement_IRQn      = -12, /*  4 Memory Management Interrupt */
	BusFault_IRQn              = -11, /*  5 Bus Fault Interrupt */
	UsageFault_IRQn            = -10, /*  6 Usage Fault Interrupt */
	SecureFault_IRQn           =  -9, /*  7 Secure Fault Interrupt */
	SVCall_IRQn                =  -5, /* 11 SV Call Interrupt */
	DebugMonitor_IRQn          =  -4, /* 12 Debug Monitor Interrupt */
	PendSV_IRQn                =  -2, /* 14 Pend SV Interrupt */
	SysTick_IRQn               =  -1, /* 15 System Tick Interrupt */

	/* ---  Processor Interrupt Numbers  --- */
	DMA1_IRQ0                  =   0,
	DMA1_IRQ1                  =   1,
	DMA1_IRQ2                  =   2,
	DMA1_IRQ3                  =   3,
	DMA1_IRQ4                  =   4,
	DMA1_IRQ5                  =   5,
	DMA1_IRQ6                  =   6,
	DMA1_IRQ7                  =   7,
	DMA1_IRQ8                  =   8,
	DMA1_IRQ9                  =   9,
	DMA1_IRQ10                 =   10,
	DMA1_IRQ11                 =   11,
	DMA1_IRQ12                 =   12,
	DMA1_IRQ13                 =   13,
	DMA1_IRQ14                 =   14,
	DMA1_IRQ15                 =   15,
	DMA1_IRQ16                 =   16,
	DMA1_IRQ17                 =   17,
	DMA1_IRQ18                 =   18,
	DMA1_IRQ19                 =   19,
	DMA1_IRQ20                 =   20,
	DMA1_IRQ21                 =   21,
	DMA1_IRQ22                 =   22,
	DMA1_IRQ23                 =   23,
	DMA1_IRQ24                 =   24,
	DMA1_IRQ25                 =   25,
	DMA1_IRQ26                 =   26,
	DMA1_IRQ27                 =   27,
	DMA1_IRQ28                 =   28,
	DMA1_IRQ29                 =   29,
	DMA1_IRQ30                 =   30,
	DMA1_IRQ31                 =   31,
	DMA1_ABORT_IRQ             =   32,
	MHU_HES10_RX_IRQ           =   33,
	MHU_ES1H0_TX_IRQ           =   34,
	MHU_HES11_RX_IRQ           =   35,
	MHU_ES1H1_TX_IRQ           =   36,
	MHU_SEES10_RX_IRQ          =   37,
	MHU_ES1SE0_TX_IRQ          =   38,
	MHU_SEES11_RX_IRQ          =   39,
	MHU_ES1SE1_TX_IRQ          =   40,
	MHU_ES0ES10_RX_IRQ         =   41,
	MHU_ES1ES00_TX_IRQ         =   42,
	MHU_ES0ES11_RX_IRQ         =   43,
	MHU_ES1ES01_TX_IRQ         =   44,
	LPUART_IRQ                 =   45,
	LPSPI_IRQ                  =   46,
	LPI2C_IRQ                  =   47,
	LPI2S_IRQ                  =   48,
	LPPDM_IRQ                  =   49,
	/* IRQs 50 to 53 are Reserved. */
	LPCPI_IRQ                  =   54,
	ETHOS_U55_1_IRQ            =   55,
	LPCMP_IRQ                  =   56,
	LPGPIO_IRQ                 =   57,
	LPRTC_IRQ                  =   58,
	/* IRQ 59 is reserved. */
	LPTIMER0_IRQ               =   60,
	LPTIMER1_IRQ               =   61,
	LPTIMER2_IRQ               =   62,
	LPTIMER3_IRQ               =   63,
	FW_IRQ                     =   64,
	SDC600_IRQ                 =   65,
	PPU_COMB_IRQ               =   66,
	REFCLK_CNTBASE0_IRQ        =   67,
	REFCLK_CNTBASE1_IRQ        =   68,
	REFCLK_CNTBASE2_IRQ        =   69,
	REFCLK_CNTBASE3_IRQ        =   70,
	S32K_CNTBASE0_IRQ          =   71,
	S32K_CNTBASE1_IRQ          =   72,
	SOC_ETR_IRQ                =   73,
	SOC_CATU_IRQ               =   74,
	/* IRQs 75 to 95 are Reserved. */
	XNVM_OSPI0_IRQ             =   96,
	XNVM_OSPI1_IRQ             =   97,
	XNVM_AESDEC0_IRQ           =   98,
	XNVM_AESDEC1_IRQ           =   99,
	VBAT_BOD_IRQ               =   100,
	USB0_IRQ                   =   101,
	SDMMC_IRQ                  =   102,
	SDMMC_WAKEUP_IRQ           =   103,
	CANFD_IRQ                  =   104,
	HWSEM0_IRQ                 =   105,
	HWSEM1_IRQ                 =   106,
	HWSEM2_IRQ                 =   107,
	HWSEM3_IRQ                 =   108,
	HWSEM4_IRQ                 =   109,
	HWSEM5_IRQ                 =   110,
	HWSEM6_IRQ                 =   111,
	HWSEM7_IRQ                 =   112,
	HWSEM8_IRQ                 =   113,
	HWSEM9_IRQ                 =   114,
	HWSEM10_IRQ                =   115,
	HWSEM11_IRQ                =   116,
	HWSEM12_IRQ                =   117,
	HWSEM13_IRQ                =   118,
	HWSEM14_IRQ                =   119,
	HWSEM15_IRQ                =   120,
	PPU0_IRQ                   =   121,
	PPU1_IRQ                   =   122,
	PPU2_IRQ                   =   123,
	UART0_IRQ                  =   124,
	UART1_IRQ                  =   125,
	UART2_IRQ                  =   126,
	UART3_IRQ                  =   127,
	UART4_IRQ                  =   128,
	UART5_IRQ                  =   129,
	UART6_IRQ                  =   130,
	UART7_IRQ                  =   131,
	I2C0_IRQ                   =   132,
	I2C1_IRQ                   =   133,
	I2C2_IRQ                   =   134,
	I2C3_IRQ                   =   135,
	I3C0_IRQ                   =   136,
	SPI0_IRQ                   =   137,
	SPI1_IRQ                   =   138,
	SPI2_IRQ                   =   139,
	SPI3_IRQ                   =   140,
	I2S0_IRQ                   =   141,
	I2S1_IRQ                   =   142,
	I2S2_IRQ                   =   143,
	I2S3_IRQ                   =   144,
	PDM_WARN_IRQ               =   145,
	PDM_ERR_IRQ                =   146,
	PDM_AUDIO_DET_IRQ2         =   147,
	ETH_SBD_IRQ                =   148,
	ETH_PMT_IRQ                =   149,
	ETH_LPI_IRQ                =   150,
	ADC0_INTR_DONE_IRQ         =   151,
	ADC1_INTR_DONE_IRQ         =   152,
	ADC2_INTR_DONE_IRQ         =   153,
	ADC0_INTR_DONE2_IRQ        =   154,
	ADC1_INTR_DONE2_IRQ        =   155,
	ADC2_INTR_DONE2_IRQ        =   156,
	ADC0_INTR_CMP1_IRQ         =   157,
	ADC1_INTR_CMP1_IRQ         =   158,
	ADC2_INTR_CMP1_IRQ         =   159,
	ADC0_INTR_CMP2_IRQ         =   160,
	ADC1_INTR_CMP2_IRQ         =   161,
	ADC2_INTR_CMP2_IRQ         =   162,
	ADC24_INTR_DONE_IRQ        =   163,
	ADC24_INTR_DONE2_IRQ       =   164,
	ADC24_INTR_CMP0_IRQ        =   165,
	ADC24_INTR_CMP1_IRQ        =   166,
	WAKEUP_IRQ                 =   167,
	CMP1_IRQ                   =   168,
	CMP2_IRQ                   =   169,
	CMP3_IRQ                   =   170,
	LPGPIO_PIN0_IRQ            =   171,
	LPGPIO_PIN1_IRQ            =   172,
	LPGPIO_PIN2_IRQ            =   173,
	LPGPIO_PIN3_IRQ            =   174,
	LPGPIO_PIN4_IRQ            =   175,
	LPGPIO_PIN5_IRQ            =   176,
	LPGPIO_PIN6_IRQ            =   177,
	LPGPIO_PIN7_IRQ            =   178,
	GPIOX0_PIN0_IRQ            =   179,
	GPIOX0_PIN1_IRQ            =   180,
	GPIOX0_PIN2_IRQ            =   181,
	GPIOX0_PIN3_IRQ            =   182,
	GPIOX0_PIN4_IRQ            =   183,
	GPIOX0_PIN5_IRQ            =   184,
	GPIOX0_PIN6_IRQ            =   185,
	GPIOX0_PIN7_IRQ            =   186,
	GPIOX1_PIN0_IRQ            =   187,
	GPIOX1_PIN1_IRQ            =   188,
	GPIOX1_PIN2_IRQ            =   189,
	GPIOX1_PIN3_IRQ            =   190,
	GPIOX1_PIN4_IRQ            =   191,
	GPIOX1_PIN5_IRQ            =   192,
	GPIOX1_PIN6_IRQ            =   193,
	GPIOX1_PIN7_IRQ            =   194,
	GPIOX2_PIN0_IRQ            =   195,
	GPIOX2_PIN1_IRQ            =   196,
	GPIOX2_PIN2_IRQ            =   197,
	GPIOX2_PIN3_IRQ            =   198,
	GPIOX2_PIN4_IRQ            =   199,
	GPIOX2_PIN5_IRQ            =   200,
	GPIOX2_PIN6_IRQ            =   201,
	GPIOX2_PIN7_IRQ            =   202,
	GPIOX3_PIN0_IRQ            =   203,
	GPIOX3_PIN1_IRQ            =   204,
	GPIOX3_PIN2_IRQ            =   205,
	GPIOX3_PIN3_IRQ            =   206,
	GPIOX3_PIN4_IRQ            =   207,
	GPIOX3_PIN5_IRQ            =   208,
	GPIOX3_PIN6_IRQ            =   209,
	GPIOX3_PIN7_IRQ            =   210,
	GPIOX4_PIN0_IRQ            =   211,
	GPIOX4_PIN1_IRQ            =   212,
	GPIOX4_PIN2_IRQ            =   213,
	GPIOX4_PIN3_IRQ            =   214,
	GPIOX4_PIN4_IRQ            =   215,
	GPIOX4_PIN5_IRQ            =   216,
	GPIOX4_PIN6_IRQ            =   217,
	GPIOX4_PIN7_IRQ            =   218,
	GPIOX5_PIN0_IRQ            =   219,
	GPIOX5_PIN1_IRQ            =   220,
	GPIOX5_PIN2_IRQ            =   221,
	GPIOX5_PIN3_IRQ            =   222,
	GPIOX5_PIN4_IRQ            =   223,
	GPIOX5_PIN5_IRQ            =   224,
	GPIOX5_PIN6_IRQ            =   225,
	GPIOX5_PIN7_IRQ            =   226,
	GPIOX6_PIN0_IRQ            =   227,
	GPIOX6_PIN1_IRQ            =   228,
	GPIOX6_PIN2_IRQ            =   229,
	GPIOX6_PIN3_IRQ            =   230,
	GPIOX6_PIN4_IRQ            =   231,
	GPIOX6_PIN5_IRQ            =   232,
	GPIOX6_PIN6_IRQ            =   233,
	GPIOX6_PIN7_IRQ            =   234,
	GPIOX7_PIN0_IRQ            =   235,
	GPIOX7_PIN1_IRQ            =   236,
	GPIOX7_PIN2_IRQ            =   237,
	GPIOX7_PIN3_IRQ            =   238,
	GPIOX7_PIN4_IRQ            =   239,
	GPIOX7_PIN5_IRQ            =   240,
	GPIOX7_PIN6_IRQ            =   241,
	GPIOX7_PIN7_IRQ            =   242,
	GPIOX8_PIN0_IRQ            =   243,
	GPIOX8_PIN1_IRQ            =   244,
	GPIOX8_PIN2_IRQ            =   245,
	GPIOX8_PIN3_IRQ            =   246,
	GPIOX8_PIN4_IRQ            =   247,
	GPIOX8_PIN5_IRQ            =   248,
	GPIOX8_PIN6_IRQ            =   249,
	GPIOX8_PIN7_IRQ            =   250,
	GPIOX9_PIN0_IRQ            =   251,
	GPIOX9_PIN1_IRQ            =   252,
	GPIOX9_PIN2_IRQ            =   253,
	GPIOX9_PIN3_IRQ            =   254,
	GPIOX9_PIN4_IRQ            =   255,
	GPIOX9_PIN5_IRQ            =   256,
	GPIOX9_PIN6_IRQ            =   257,
	GPIOX9_PIN7_IRQ            =   258,
	GPIOX10_PIN0_IRQ           =   259,
	GPIOX10_PIN1_IRQ           =   260,
	GPIOX10_PIN2_IRQ           =   261,
	GPIOX10_PIN3_IRQ           =   262,
	GPIOX10_PIN4_IRQ           =   263,
	GPIOX10_PIN5_IRQ           =   264,
	GPIOX10_PIN6_IRQ           =   265,
	GPIOX10_PIN7_IRQ           =   266,
	GPIOX11_PIN0_IRQ           =   267,
	GPIOX11_PIN1_IRQ           =   268,
	GPIOX11_PIN2_IRQ           =   269,
	GPIOX11_PIN3_IRQ           =   270,
	GPIOX11_PIN4_IRQ           =   271,
	GPIOX11_PIN5_IRQ           =   272,
	GPIOX11_PIN6_IRQ           =   273,
	GPIOX11_PIN7_IRQ           =   274,
	GPIOX12_PIN0_IRQ           =   275,
	GPIOX12_PIN1_IRQ           =   276,
	GPIOX12_PIN2_IRQ           =   277,
	GPIOX12_PIN3_IRQ           =   278,
	GPIOX12_PIN4_IRQ           =   279,
	GPIOX12_PIN5_IRQ           =   280,
	GPIOX12_PIN6_IRQ           =   281,
	GPIOX12_PIN7_IRQ           =   282,
	GPIOX13_PIN0_IRQ           =   283,
	GPIOX13_PIN1_IRQ           =   284,
	GPIOX13_PIN2_IRQ           =   285,
	GPIOX13_PIN3_IRQ           =   286,
	GPIOX13_PIN4_IRQ           =   287,
	GPIOX13_PIN5_IRQ           =   288,
	GPIOX13_PIN6_IRQ           =   289,
	GPIOX13_PIN7_IRQ           =   290,
	GPIOX14_PIN0_IRQ           =   291,
	GPIOX14_PIN1_IRQ           =   292,
	GPIOX14_PIN2_IRQ           =   293,
	GPIOX14_PIN3_IRQ           =   294,
	GPIOX14_PIN4_IRQ           =   295,
	GPIOX14_PIN5_IRQ           =   296,
	GPIOX14_PIN6_IRQ           =   297,
	GPIOX14_PIN7_IRQ           =   298,
	DMA0_IRQ0                  =   299,
	DMA0_IRQ1                  =   300,
	DMA0_IRQ2                  =   301,
	DMA0_IRQ3                  =   302,
	DMA0_IRQ4                  =   303,
	DMA0_IRQ5                  =   304,
	DMA0_IRQ6                  =   305,
	DMA0_IRQ7                  =   306,
	DMA0_IRQ8                  =   307,
	DMA0_IRQ9                  =   308,
	DMA0_IRQ10                 =   309,
	DMA0_IRQ11                 =   310,
	DMA0_IRQ12                 =   311,
	DMA0_IRQ13                 =   312,
	DMA0_IRQ14                 =   313,
	DMA0_IRQ15                 =   314,
	DMA0_IRQ16                 =   315,
	DMA0_IRQ17                 =   316,
	DMA0_IRQ18                 =   317,
	DMA0_IRQ19                 =   318,
	DMA0_IRQ20                 =   319,
	DMA0_IRQ21                 =   320,
	DMA0_IRQ22                 =   321,
	DMA0_IRQ23                 =   322,
	DMA0_IRQ24                 =   323,
	DMA0_IRQ25                 =   324,
	DMA0_IRQ26                 =   325,
	DMA0_IRQ27                 =   326,
	DMA0_IRQ28                 =   327,
	DMA0_IRQ29                 =   328,
	DMA0_IRQ30                 =   329,
	DMA0_IRQ31                 =   330,
	DMA0_ABORT_IRQ             =   331,
	DAVE2_IRQ                  =   332,
	CDC200_SCANLINE0_IRQ       =   333,
	CDC200_SCANLINE1_IRQ       =   334,
	CDC200_FIFO_WARNING0_IRQ   =   335,
	CDC200_FIFO_WARNING1_IRQ   =   336,
	CDC200_FIFO_UNDERRUN0_IRQ  =   337,
	CDC200_FIFO_UNDERRUN1_IRQ  =   338,
	CDC200_BUS_ERROR0_IRQ      =   339,
	CDC200_BUS_ERROR1_IRQ      =   340,
	CDC200_REG_RELOAD0_IRQ     =   341,
	CDC200_REG_RELOAD1_IRQ     =   342,
	MIPI_DSI_IRQ               =   343,
	MIPI_CSI2_IRQ              =   344,
	CPI_IRQ                    =   345,
	LPTIMER_CHANNEL0_IRQ       =   346,
	LPTIMER_CHANNEL1_IRQ       =   347,
	LPTIMER_CHANNEL2_IRQ       =   348,
	LPTIMER_CHANNEL3_IRQ       =   349,
	RTC0_IRQ                   =   350,
	MODEM_IRQ0                 =   351,
	MODEM_IRQ1                 =   352,
	MODEM_IRQ2                 =   353,
	MODEM_IRQ3                 =   354,
	MODEM_IRQ4                 =   355,
	MODEM_IRQ5                 =   356,
	MODEM_IRQ6                 =   357,
	MODEM_IRQ7                 =   358,
	MODEM_IRQ8                 =   359,
	MODEM_IRQ9                 =   360,
	MODEM_IRQ10                =   361,
	MODEM_IRQ11                =   362,
	IPC_GNSS_IRQ0              =   363,
	IPC_GNSS_IRQ1              =   364,
	IPC_MDM_XTENSA_IRQ0        =   365,
	IPC_MDM_IRQ0               =   366,
	IPC_MDM_ITR1               =   367,
	IPC_MDM_ITR2               =   368,
	QEC0_INTR_CMP_A_IRQ        =   369,
	QEC1_INTR_CMP_A_IRQ        =   370,
	QEC2_INTR_CMP_A_IRQ        =   371,
	QEC3_INTR_CMP_A_IRQ        =   372,
	QEC0_INTR_CMP_B_IRQ        =   373,
	QEC1_INTR_CMP_B_IRQ        =   374,
	QEC2_INTR_CMP_B_IRQ        =   375,
	QEC3_INTR_CMP_B_IRQ        =   376,
	UTIMER0_IRQ                =   377,
	UTIMER1_IRQ                =   378,
	UTIMER2_IRQ                =   379,
	UTIMER3_IRQ                =   380,
	UTIMER4_IRQ                =   381,
	UTIMER5_IRQ                =   382,
	UTIMER6_IRQ                =   383,
	UTIMER7_IRQ                =   384,
	UTIMER8_IRQ                =   385,
	UTIMER9_IRQ                =   386,
	UTIMER10_IRQ               =   387,
	UTIMER11_IRQ               =   388,
	UTIMER12_IRQ               =   389,
	UTIMER13_IRQ               =   390,
	UTIMER14_IRQ               =   391,
	UTIMER15_IRQ               =   392,
	UTIMER16_IRQ               =   393,
	UTIMER17_IRQ               =   394,
	UTIMER18_IRQ               =   395,
	UTIMER19_IRQ               =   396,
	UTIMER20_IRQ               =   397,
	UTIMER21_IRQ               =   398,
	UTIMER22_IRQ               =   399,
	UTIMER23_IRQ               =   400,
	UTIMER24_IRQ               =   401,
	UTIMER25_IRQ               =   402,
	UTIMER26_IRQ               =   403,
	UTIMER27_IRQ               =   404,
	UTIMER28_IRQ               =   405,
	UTIMER29_IRQ               =   406,
	UTIMER30_IRQ               =   407,
	UTIMER31_IRQ               =   408,
	UTIMER32_IRQ               =   409,
	UTIMER33_IRQ               =   410,
	UTIMER34_IRQ               =   411,
	UTIMER35_IRQ               =   412,
	UTIMER36_IRQ               =   413,
	UTIMER37_IRQ               =   414,
	UTIMER38_IRQ               =   415,
	UTIMER39_IRQ               =   416,
	UTIMER40_IRQ               =   417,
	UTIMER41_IRQ               =   418,
	UTIMER42_IRQ               =   419,
	UTIMER43_IRQ               =   420,
	UTIMER44_IRQ               =   421,
	UTIMER45_IRQ               =   422,
	UTIMER46_IRQ               =   423,
	UTIMER47_IRQ               =   424,
	UTIMER48_IRQ               =   425,
	UTIMER49_IRQ               =   426,
	UTIMER50_IRQ               =   427,
	UTIMER51_IRQ               =   428,
	UTIMER52_IRQ               =   429,
	UTIMER53_IRQ               =   430,
	UTIMER54_IRQ               =   431,
	UTIMER55_IRQ               =   432,
	UTIMER56_IRQ               =   433,
	UTIMER57_IRQ               =   434,
	UTIMER58_IRQ               =   435,
	UTIMER59_IRQ               =   436,
	UTIMER60_IRQ               =   437,
	UTIMER61_IRQ               =   438,
	UTIMER62_IRQ               =   439,
	UTIMER63_IRQ               =   440,
	UTIMER64_IRQ               =   441,
	UTIMER65_IRQ               =   442,
	UTIMER66_IRQ               =   443,
	UTIMER67_IRQ               =   444,
	UTIMER68_IRQ               =   445,
	UTIMER69_IRQ               =   446,
	UTIMER70_IRQ               =   447,
	UTIMER71_IRQ               =   448,
	UTIMER72_IRQ               =   449,
	UTIMER73_IRQ               =   450,
	UTIMER74_IRQ               =   451,
	UTIMER75_IRQ               =   452,
	UTIMER76_IRQ               =   453,
	UTIMER77_IRQ               =   454,
	UTIMER78_IRQ               =   455,
	UTIMER79_IRQ               =   456,
	UTIMER80_IRQ               =   457,
	UTIMER81_IRQ               =   458,
	UTIMER82_IRQ               =   459,
	UTIMER83_IRQ               =   460,
	UTIMER84_IRQ               =   461,
	UTIMER85_IRQ               =   462,
	UTIMER86_IRQ               =   463,
	UTIMER87_IRQ               =   464,
	UTIMER88_IRQ               =   465,
	UTIMER89_IRQ               =   466,
	UTIMER90_IRQ               =   467,
	UTIMER91_IRQ               =   468,
	UTIMER92_IRQ               =   469,
	UTIMER93_IRQ               =   470,
	UTIMER94_IRQ               =   471,
	UTIMER95_IRQ               =   472
	/* Interrupts 473 .. 480 are left out */
} CMSIS_IRQn_Type;

#define __Vendor_SysTickConfig         0 /* Default to standard SysTick */

#endif /* CONFIG_SOC_E7_DK_APSS */

#include <cmsis_core_m_defaults.h>

#endif /* _SOC_H_ */
