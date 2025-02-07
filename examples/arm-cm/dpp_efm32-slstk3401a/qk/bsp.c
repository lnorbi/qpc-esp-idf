/*****************************************************************************
* Product: DPP example, EFM32-SLSTK3401A board, preemptive QK kernel
* Last updated for version 7.2.0
* Last updated on  2022-12-17
*
*                    Q u a n t u m  L e a P s
*                    ------------------------
*                    Modern Embedded Software
*
* Copyright (C) 2005 Quantum Leaps, LLC. All rights reserved.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Alternatively, this program may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GNU General Public License and are specifically designed for
* licensees interested in retaining the proprietary status of their code.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <www.gnu.org/licenses/>.
*
* Contact information:
* <www.state-machine.com/licensing>
* <info@state-machine.com>
*****************************************************************************/
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

#include "em_device.h"  /* the device specific header (SiLabs) */
#include "em_cmu.h"     /* Clock Management Unit (SiLabs) */
#include "em_gpio.h"    /* GPIO (SiLabs) */
#include "em_usart.h"   /* USART (SiLabs) */
/* add other drivers if necessary... */

Q_DEFINE_THIS_FILE

/* ISRs defined in this BSP ------------------------------------------------*/
void SysTick_Handler(void);
void GPIO_EVEN_IRQHandler(void);
void USART0_RX_IRQHandler(void);

/* Local-scope objects -----------------------------------------------------*/
#define LED_PORT    gpioPortF
#define LED0_PIN    4
#define LED1_PIN    5

#define PB_PORT     gpioPortF
#define PB0_PIN     6
#define PB1_PIN     7

static uint32_t l_rnd;      /* random seed */

#ifdef Q_SPY

    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;

    /* QSpy source IDs */
    static QSpyId const l_SysTick_Handler = { 0U };
    static QSpyId const l_GPIO_EVEN_IRQHandler = { 0U };

    static USART_TypeDef * const l_USART0 = ((USART_TypeDef *)(0x40010000UL));

    enum AppRecords { /* application-specific trace records */
        PHILO_STAT = QS_USER,
        COMMAND_STAT
    };

#endif

/*..........................................................................*/
void SysTick_Handler(void) {
    QK_ISR_ENTRY();   /* inform QK about entering an ISR */

    QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); /* process time-evts for rate 0 */
    //QACTIVE_POST(the_Ticker0, 0, &l_SysTick_Handler); /* post to Ticker0 */

    /* Perform the debouncing of buttons. The algorithm for debouncing
    * adapted from the book "Embedded Systems Dictionary" by Jack Ganssle
    * and Michael Barr, page 71.
    */
    /* state of the button debouncing, see below */
    static struct ButtonsDebouncing {
        uint32_t depressed;
        uint32_t previous;
    } buttons = { 0U, 0U };
    uint32_t current = ~GPIO->P[PB_PORT].DIN; /* read PB0 and BP1 */
    uint32_t tmp = buttons.depressed; /* the debounced depressed buttons */
    buttons.depressed |= (buttons.previous & current); /* set depressed */
    buttons.depressed &= (buttons.previous | current); /* clear released */
    buttons.previous   = current; /* update the history */
    tmp ^= buttons.depressed;     /* changed debounced depressed */
    if ((tmp & (1U << PB0_PIN)) != 0U) {  /* debounced PB0 state changed? */
        if ((buttons.depressed & (1U << PB0_PIN)) != 0U) { /* PB0 depressed?*/
            static QEvt const pauseEvt = { PAUSE_SIG, 0U, 0U};
            QACTIVE_PUBLISH(&pauseEvt, &l_SysTick_Handler);
        }
        else {            /* the button is released */
            static QEvt const serveEvt = { SERVE_SIG, 0U, 0U};
            QACTIVE_PUBLISH(&serveEvt, &l_SysTick_Handler);
        }
    }

#ifdef Q_SPY
    {
        tmp = SysTick->CTRL; /* clear SysTick_CTRL_COUNTFLAG */
        QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
    }
#endif

    QK_ISR_EXIT();  /* inform QK about exiting an ISR */
}
/*..........................................................................*/
void GPIO_EVEN_IRQHandler(void) { /* for testing, NOTE3 */
    QK_ISR_ENTRY(); /* inform QK about entering an ISR */

    QACTIVE_POST(AO_Table, Q_NEW(QEvt, MAX_PUB_SIG), /* for testing... */
                 &l_GPIO_EVEN_IRQHandler);

    QK_ISR_EXIT();  /* inform QK about exiting an ISR */
}
/*..........................................................................*/
#ifdef Q_SPY
/*
* ISR for receiving bytes from the QSPY Back-End
* NOTE: This ISR is "QF-unaware" meaning that it does not interact with
* the QF/QK and is not disabled. Such ISRs don't need to call QK_ISR_ENTRY/
* QK_ISR_EXIT and they cannot post or publish events.
*/
void USART0_RX_IRQHandler(void) {
    /* while RX FIFO NOT empty */
    while ((l_USART0->STATUS & USART_STATUS_RXDATAV) != 0) {
        uint32_t b = l_USART0->RXDATA;
        QS_RX_PUT(b);
    }
    QK_ARM_ERRATUM_838869();
}
#else
void USART0_RX_IRQHandler(void) {}
#endif


/* BSP functions ===========================================================*/
/* MPU setup for EFM32PG1B200F256GM48 MCU */
static void EFM32PG182_MPU_setup(void) {
    /* The following MPU configuration contains the general EFM32PG1 memory
    * map described in the EFM32PG1 Data Sheet Figure 3.2. EFM32PG1 Memory Map
    *
    * Please note that the actual STM32 MCUs provide much less Flash and SRAM
    * than the maximums configured here. This means that actual MCUs have
    * unmapped memory regions (e.g., beyond the actual SRAM). Attempts to
    * access these regions causes the HardFault exception, which is the
    * desired behavior.
    */
    static struct {
        uint32_t rbar;
        uint32_t rasr;
    } const mpu_setup[] = {

        { /* region #0: Flash: base=0x0000'0000, size=512M=2^(28+1) */
          0x00000000U                       /* base address */
              | MPU_RBAR_VALID_Msk          /* valid region */
              | (MPU_RBAR_REGION_Msk & 0U), /* region #0 */
          (28U << MPU_RASR_SIZE_Pos)        /* 2^(18+1) region */
              | (0x6U << MPU_RASR_AP_Pos)   /* PA:ro/UA:ro */
              | (1U << MPU_RASR_C_Pos)      /* C=1 */
              | MPU_RASR_ENABLE_Msk         /* region enable */
        },

        { /* region #1: SRAM: base=0x2000'0000, size=512M=2^(28+1) */
          0x20000000U                       /* base address */
              | MPU_RBAR_VALID_Msk          /* valid region */
              | (MPU_RBAR_REGION_Msk & 1U), /* region #1 */
          (28U << MPU_RASR_SIZE_Pos)        /* 2^(28+1) region */
              | (0x3U << MPU_RASR_AP_Pos)   /* PA:rw/UA:rw */
              | (1U << MPU_RASR_XN_Pos)     /* XN=1 */
              | (1U << MPU_RASR_S_Pos)      /* S=1 */
              | (1U << MPU_RASR_C_Pos)      /* C=1 */
              | MPU_RASR_ENABLE_Msk         /* region enable */
        },

        /* region #3: (not configured) */
        { MPU_RBAR_VALID_Msk | (MPU_RBAR_REGION_Msk & 2U), 0U },

        { /* region #3: Peripherals: base=0x4000'0000, size=512M=2^(28+1) */
          0x40000000U                       /* base address */
              | MPU_RBAR_VALID_Msk          /* valid region */
              | (MPU_RBAR_REGION_Msk & 3U), /* region #3 */
          (28U << MPU_RASR_SIZE_Pos)        /* 2^(28+1) region */
              | (0x3U << MPU_RASR_AP_Pos)   /* PA:rw/UA:rw */
              | (1U << MPU_RASR_XN_Pos)     /* XN=1 */
              | (1U << MPU_RASR_S_Pos)      /* S=1 */
              | (1U << MPU_RASR_B_Pos)      /* B=1 */
              | MPU_RASR_ENABLE_Msk         /* region enable */
        },

        { /* region #4: Priv. Periph: base=0xE000'0000, size=512M=2^(28+1) */
          0xE0000000U                       /* base address */
              | MPU_RBAR_VALID_Msk          /* valid region */
              | (MPU_RBAR_REGION_Msk & 4U), /* region #4 */
          (28U << MPU_RASR_SIZE_Pos)        /* 2^(28+1) region */
              | (0x3U << MPU_RASR_AP_Pos)   /* PA:rw/UA:rw */
              | (1U << MPU_RASR_XN_Pos)     /* XN=1 */
              | (1U << MPU_RASR_S_Pos)      /* S=1 */
              | (1U << MPU_RASR_B_Pos)      /* B=1 */
              | MPU_RASR_ENABLE_Msk         /* region enable */
        },

        { /* region #5: Ext RAM: base=0x6000'0000, size=1G=2^(29+1) */
          0x60000000U                       /* base address */
              | MPU_RBAR_VALID_Msk          /* valid region */
              | (MPU_RBAR_REGION_Msk & 5U), /* region #5 */
          (29U << MPU_RASR_SIZE_Pos)        /* 2^(28+1) region */
              | (0x3U << MPU_RASR_AP_Pos)   /* PA:rw/UA:rw */
              | (1U << MPU_RASR_XN_Pos)     /* XN=1 */
              | (1U << MPU_RASR_S_Pos)      /* S=1 */
              | (1U << MPU_RASR_B_Pos)      /* B=1 */
              | MPU_RASR_ENABLE_Msk         /* region enable */
        },

        { /* region #6: Ext Dev: base=0xA000'0000, size=1G=2^(29+1) */
          0xA0000000U                       /* base address */
              | MPU_RBAR_VALID_Msk          /* valid region */
              | (MPU_RBAR_REGION_Msk & 6U), /* region #6 */
          (29U << MPU_RASR_SIZE_Pos)        /* 2^(28+1) region */
              | (0x3U << MPU_RASR_AP_Pos)   /* PA:rw/UA:rw */
              | (1U << MPU_RASR_XN_Pos)     /* XN=1 */
              | (1U << MPU_RASR_S_Pos)      /* S=1 */
              | (1U << MPU_RASR_B_Pos)      /* B=1 */
              | MPU_RASR_ENABLE_Msk         /* region enable */
        },

        { /* region #7: NULL-pointer: base=0x000'0000, size=256B, NOTE0 */
          0x00000000U                       /* base address */
              | MPU_RBAR_VALID_Msk          /* valid region */
              | (MPU_RBAR_REGION_Msk & 7U), /* region #7 */
          (7U << MPU_RASR_SIZE_Pos)         /* 2^(7+1)=256B region */
              | (0x0U << MPU_RASR_AP_Pos)   /* PA:na/UA:na */
              | (1U << MPU_RASR_XN_Pos)     /* XN=1 */
              | MPU_RASR_ENABLE_Msk         /* region enable */
        },
    };

    /* enable the MemManage_Handler for MPU exception */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    __DSB();
    MPU->CTRL = 0U; /* disable the MPU */
    for (uint_fast8_t n = 0U; n < Q_DIM(mpu_setup); ++n) {
        MPU->RBAR = mpu_setup[n].rbar;
        MPU->RASR = mpu_setup[n].rasr;
    }
    MPU->CTRL = MPU_CTRL_ENABLE_Msk;        /* enable the MPU */
    __ISB();
    __DSB();
}

/*..........................................................................*/
void BSP_init(void) {
    /* setup the MPU... */
    EFM32PG182_MPU_setup();

    /* NOTE: SystemInit() already called from the startup code
    *  but SystemCoreClock needs to be updated
    */
    SystemCoreClockUpdate();

    /* NOTE: The VFP (hardware Floating Point) unit is configured by QK-port */

    /* enable clock for to the peripherals used by this application... */
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_GPIO,  true);
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_GPIO,  true);

    /* configure the LEDs */
    GPIO_PinModeSet(LED_PORT, LED0_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(LED_PORT, LED1_PIN, gpioModePushPull, 0);
    GPIO_PinOutClear(LED_PORT, LED0_PIN);
    GPIO_PinOutClear(LED_PORT, LED1_PIN);

    /* configure the Buttons */
    GPIO_PinModeSet(PB_PORT, PB0_PIN, gpioModeInputPull, 1);
    GPIO_PinModeSet(PB_PORT, PB1_PIN, gpioModeInputPull, 1);

    /*... */
    BSP_randomSeed(1234U);

    if (QS_INIT((void *)0) == 0) { /* initialize the QS software tracing */
        Q_ERROR();
    }
    /* global signals */
    QS_SIG_DICTIONARY(DONE_SIG,      (void *)0);
    QS_SIG_DICTIONARY(EAT_SIG,       (void *)0);
    QS_SIG_DICTIONARY(PAUSE_SIG,     (void *)0);
    QS_SIG_DICTIONARY(SERVE_SIG,     (void *)0);
    QS_SIG_DICTIONARY(TEST_SIG,      (void *)0);
    QS_SIG_DICTIONARY(HUNGRY_SIG,    (void *)0);
    QS_SIG_DICTIONARY(HUNGRY_SIG,    (void *)0);
    QS_SIG_DICTIONARY(TIMEOUT_SIG,   (void *)0);

    QS_OBJ_DICTIONARY(the_Ticker0);
    QS_OBJ_DICTIONARY(&l_SysTick_Handler);
    QS_OBJ_DICTIONARY(&l_GPIO_EVEN_IRQHandler);

    QS_USR_DICTIONARY(PHILO_STAT);
    QS_USR_DICTIONARY(COMMAND_STAT);

    /* setup the QS filters... */
    //QS_GLB_FILTER(QS_SM_RECORDS); /* state machine records */
    //QS_GLB_FILTER(QS_AO_RECORDS); /* active object records */
    //QS_GLB_FILTER(QS_UA_RECORDS); /* all user records */
    QS_GLB_FILTER(QS_ALL_RECORDS); /* all records */
    QS_GLB_FILTER(-QS_QF_TICK);    /* exclude the clock tick */
}
/*..........................................................................*/
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    QS_TEST_PROBE_DEF(&BSP_displayPhilStat)

    QS_TEST_PROBE_ID(1,
        stat = "Unknown";
    )

    if (stat[0] == 'e') {
        GPIO->P[LED_PORT].DOUT |=  (1U << LED0_PIN);
    }
    else {
        GPIO->P[LED_PORT].DOUT &=  ~(1U << LED0_PIN);
    }

    QS_BEGIN_ID(PHILO_STAT, AO_Philo[n]->prio) /* app-specific record */
        QS_U8(1, n);  /* Philosopher number */
        QS_STR(stat); /* Philosopher status */
    QS_END()          /* application-specific record end */
}
/*..........................................................................*/
void BSP_displayPaused(uint8_t paused) {
    if (paused != 0U) {
        GPIO->P[LED_PORT].DOUT |=  (1U << LED0_PIN);
    }
    else {
        GPIO->P[LED_PORT].DOUT &= ~(1U << LED0_PIN);
    }
}
/*..........................................................................*/
uint32_t BSP_random(void) { /* a very cheap pseudo-random-number generator */
    uint32_t rnd;
    QSchedStatus lockStat;

    /* Some flating point code is to exercise the VFP... */
    float volatile x = 3.1415926F;
    x = x + 2.7182818F;

    lockStat = QK_schedLock(N_PHILO); /* lock scheduler up to N_PHILO prio */
    /* "Super-Duper" Linear Congruential Generator (LCG)
    * LCG(2^32, 3*7*11*13*23, 0, seed)
    */
    rnd = l_rnd * (3U*7U*11U*13U*23U);
    l_rnd = rnd; /* set for the next time */
    QK_schedUnlock(lockStat); /* unlock the scheduler */

    return (rnd >> 8);
}
/*..........................................................................*/
void BSP_randomSeed(uint32_t seed) {
    l_rnd = seed;
}
/*..........................................................................*/
void BSP_terminate(int16_t result) {
    (void)result;
}

/* QF callbacks ============================================================*/
void QF_onStartup(void) {
    /* set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate */
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    /* assign all priority bits for preemption-prio. and none to sub-prio. */
    NVIC_SetPriorityGrouping(0U);

    /* set priorities of ALL ISRs used in the system, see NOTE1
    *
    * !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    * Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
    * DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
    */
    NVIC_SetPriority(USART0_RX_IRQn, 0U); /* kernel unaware interrupt */
    NVIC_SetPriority(GPIO_EVEN_IRQn, QF_AWARE_ISR_CMSIS_PRI);
    NVIC_SetPriority(SysTick_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 1U);
    /* ... */

    /* enable IRQs... */
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#ifdef Q_SPY
    NVIC_EnableIRQ(USART0_RX_IRQn); /* UART0 interrupt used for QS-RX */
#endif
}
/*..........................................................................*/
void QF_onCleanup(void) {
}
/*..........................................................................*/
void QK_onIdle(void) {
    /* toggle the User LED on and then off, see NOTE2 */
    QF_INT_DISABLE();
    GPIO->P[LED_PORT].DOUT |=  (1U << LED1_PIN);
    GPIO->P[LED_PORT].DOUT &= ~(1U << LED1_PIN);
    QF_INT_ENABLE();

#ifdef Q_SPY
    QS_rxParse();  /* parse all the received bytes */

    if ((l_USART0->STATUS & USART_STATUS_TXBL) != 0) {  /* is TXE empty? */
        uint16_t b;

        QF_INT_DISABLE();
        b = QS_getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {  /* not End-Of-Data? */
            l_USART0->TXDATA = (b & 0xFFU);  /* put into the DR register */
        }
    }
#elif defined NDEBUG
    /* Put the CPU and peripherals to the low-power mode.
    * you might need to customize the clock management for your application,
    * see the datasheet for your particular Cortex-M3 MCU.
    */
    __WFI(); /* Wait-For-Interrupt */
#endif
}

/*..........................................................................*/
Q_NORETURN Q_onError(char const * const module, int_t const id) {
    /*
    * NOTE: add here your application-specific error handling
    */
    Q_UNUSED_PAR(module);
    Q_UNUSED_PAR(id);

    QS_ASSERTION(module, id, 10000U); /* report assertion to QS */

#ifndef NDEBUG
    /* light up both LEDs */
    GPIO->P[LED_PORT].DOUT |= ((1U << LED0_PIN) | (1U << LED1_PIN));
    /* for debugging, hang on in an endless loop... */
    for (;;) {
    }
#endif

    NVIC_SystemReset();
}
/*..........................................................................*/
void assert_failed(char const * const module, int_t const id); /* prototype */
void assert_failed(char const * const module, int_t const id) {
    Q_onError(module, id);
}

/* QS callbacks ============================================================*/
#ifdef Q_SPY
/*..........................................................................*/
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsTxBuf[2*1024]; /* buffer for QS transmit channel */
    static uint8_t qsRxBuf[100];    /* buffer for QS receive channel */
    static USART_InitAsync_TypeDef init = {
        usartEnable,      /* Enable RX/TX when init completed */
        0,                /* Use current clock for configuring baudrate */
        115200,           /* 115200 bits/s */
        usartOVS16,       /* 16x oversampling */
        usartDatabits8,   /* 8 databits */
        usartNoParity,    /* No parity */
        usartStopbits1,   /* 1 stopbit */
        0,                /* Do not disable majority vote */
        0,                /* Not USART PRS input mode */
        usartPrsRxCh0,    /* PRS channel 0 */
        0,                /* Auto CS functionality enable/disable switch */
        0,                /* Auto CS Hold cycles */
        0                 /* Auto CS Setup cycles */
    };

    QS_initBuf  (qsTxBuf, sizeof(qsTxBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    /* Enable peripheral clocks */
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* To avoid false start, configure output as high */
    GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1); // TX pin
    GPIO_PinModeSet(gpioPortA, 1, gpioModeInput, 0);    // RX pin

    /* Enable DK RS232/UART switch */
    GPIO_PinModeSet(gpioPortA, 5, gpioModePushPull, 1);
    CMU_ClockEnable(cmuClock_USART0, true);

    /* configure the UART for the desired baud rate, 8-N-1 operation */
    init.enable = usartDisable;
    USART_InitAsync(l_USART0, &init);

    /* enable pins at correct UART/USART location. */
    l_USART0->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
    l_USART0->ROUTELOC0 = (l_USART0->ROUTELOC0 &
                           ~(_USART_ROUTELOC0_TXLOC_MASK
                           | _USART_ROUTELOC0_RXLOC_MASK));

    /* Clear previous RX interrupts */
    USART_IntClear(l_USART0, USART_IF_RXDATAV);
    NVIC_ClearPendingIRQ(USART0_RX_IRQn);

    /* Enable RX interrupts */
    USART_IntEnable(l_USART0, USART_IF_RXDATAV);
    /* NOTE: do not enable the UART0 interrupt in the NVIC yet.
    * Wait till QF_onStartup()
    */

    /* Finally enable the UART */
    USART_Enable(l_USART0, usartEnable);

    QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */

    return 1U; /* return success */
}
/*..........................................................................*/
void QS_onCleanup(void) {
}
/*..........................................................................*/
QSTimeCtr QS_onGetTime(void) {  /* NOTE: invoked with interrupts DISABLED */
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) { /* not set? */
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { /* the rollover occured, but the SysTick_ISR did not run yet */
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}
/*..........................................................................*/
void QS_onFlush(void) {
    uint16_t b;

    QF_INT_DISABLE();
    while ((b = QS_getByte()) != QS_EOD) { /* while not End-Of-Data... */
        QF_INT_ENABLE();
        /* while TXE not empty */
        while ((l_USART0->STATUS & USART_STATUS_TXBL) == 0U) {
        }
        l_USART0->TXDATA  = (b & 0xFFU); /* put into the DR register */
        QF_INT_DISABLE();
    }
    QF_INT_ENABLE();
}
/*..........................................................................*/
/*! callback function to reset the target (to be implemented in the BSP) */
void QS_onReset(void) {
    NVIC_SystemReset();
}
/*..........................................................................*/
/*! callback function to execute a user command (to be implemented in BSP) */
void QS_onCommand(uint8_t cmdId,
                  uint32_t param1, uint32_t param2, uint32_t param3)
{
    (void)cmdId;
    (void)param1;
    (void)param2;
    (void)param3;
    QS_BEGIN_ID(COMMAND_STAT, 0U) /* app-specific record */
        QS_U8(2, cmdId);
        QS_U32(8, param1);
        QS_U32(8, param2);
        QS_U32(8, param3);
    QS_END()

    if (cmdId == 10U) {
        Q_ERROR();
    }
}

#endif /* Q_SPY */
/*--------------------------------------------------------------------------*/

/*============================================================================
* NOTE0:
* The MPU protection against NULL-pointer dereferencing sets up a no-access
* MPU region #7 around the NULL address (0x0). This works even though the
* Vector Table also resides at address 0x0. However, the *size* of the
* no-access region should not exceed the size of the Vector Table. In this
* case, the size is set to 2**(7+1)==256 bytes, which does not contain any
* data that the CPU would legitimately read with the LDR instruction.
*
* NOTE1:
* The QF_AWARE_ISR_CMSIS_PRI constant from the QF port specifies the highest
* ISR priority that is disabled by the QF framework. The value is suitable
* for the NVIC_SetPriority() CMSIS function.
*
* Only ISRs prioritized at or below the QF_AWARE_ISR_CMSIS_PRI level (i.e.,
* with the numerical values of priorities equal or higher than
* QF_AWARE_ISR_CMSIS_PRI) are allowed to call the QK_ISR_ENTRY/QK_ISR_ENTRY
* macros or any other QF/QK  services. These ISRs are "QF-aware".
*
* Conversely, any ISRs prioritized above the QF_AWARE_ISR_CMSIS_PRI priority
* level (i.e., with the numerical values of priorities less than
* QF_AWARE_ISR_CMSIS_PRI) are never disabled and are not aware of the kernel.
* Such "QF-unaware" ISRs cannot call any QF/QK services. In particular they
* can NOT call the macros QK_ISR_ENTRY/QK_ISR_ENTRY. The only mechanism
* by which a "QF-unaware" ISR can communicate with the QF framework is by
* triggering a "QF-aware" ISR, which can post/publish events.
*
* NOTE2:
* The User LED is used to visualize the idle loop activity. The brightness
* of the LED is proportional to the frequency of invcations of the idle loop.
* Please note that the LED is toggled with interrupts locked, so no interrupt
* execution time contributes to the brightness of the User LED.
*
* NOTE3:
* GPIO_EVEN_IRQHandler() is for testing various preemption scenarios in QK.
* The general testing strategy is to trigger this IRQ manually from the
* debugger. To do so in IAR, you need to:
* 1. open the Register view
* 2. open NVIC registers
* 3. scroll down to NVIC_ISPR0 register
* 4. write 0x200 to NVIC_ISPR0.SETPEND register
*/
