/*============================================================================
* Product: DPP example, STM32 NUCLEO-L152RE board, cooperative QV kernel
<<<<<<< HEAD
* Last updated for version 7.2.1
* Last updated on  2023-01-26
=======
* Last updated for version 7.3.0
* Last updated on  2023-05-26
>>>>>>> 503419cfc7b6785562856d24396f6bbe6d9cf4a3
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
============================================================================*/
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

#include "stm32l1xx.h"  /* CMSIS-compliant header file for the MCU used */
/* add other drivers if necessary... */

Q_DEFINE_THIS_FILE

/* Local-scope objects -----------------------------------------------------*/
/* LED pins available on the board (just one user LED LD2--Green on PA.5) */
#define LED_PIN  5U

/* Button pins available on the board (just one user Button B1 on PC.13) */
#define BTN_PIN  13U

static uint32_t l_rnd;  /* random seed */

#ifdef Q_SPY
    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;

    /* QSpy source IDs */
    static QSpyId const l_SysTick_Handler = { 0U };

    enum AppRecords { /* application-specific trace records */
        PHILO_STAT = QS_USER
    };

#endif

/* ISRs used in the application ==========================================*/
void SysTick_Handler(void);
void USART2_IRQHandler(void);

/*..........................................................................*/
void SysTick_Handler(void) { /* system clock tick ISR -- kernel aware */
    /* state of the button debouncing, see below */
    static struct ButtonsDebouncing {
        uint32_t depressed;
        uint32_t previous;
    } buttons = { 0U, 0U };
    uint32_t current;
    uint32_t tmp;

#ifdef Q_SPY
    {
        tmp = SysTick->CTRL; /* clear CTRL_COUNTFLAG */
        QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
    }
#endif

    QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); /* process time events for rate 0 */

    /* get state of the user button */
    /* Perform the debouncing of buttons. The algorithm for debouncing
    * adapted from the book "Embedded Systems Dictionary" by Jack Ganssle
    * and Michael Barr, page 71.
    */
    current = ~GPIOC->IDR; /* read Port C with the state of Button B1 */
    tmp = buttons.depressed; /* save the debounced depressed buttons */
    buttons.depressed |= (buttons.previous & current); /* set depressed */
    buttons.depressed &= (buttons.previous | current); /* clear released */
    buttons.previous   = current; /* update the history */
    tmp ^= buttons.depressed;     /* changed debounced depressed */
    if ((tmp & (1U << BTN_PIN)) != 0U) { /* debounced BTN state changed? */
        if ((buttons.depressed & (1U << BTN_PIN)) != 0U) { /* is BTN depressed? */
            static QEvt const pauseEvt = { PAUSE_SIG, 0U, 0U};
            QACTIVE_PUBLISH(&pauseEvt, &l_SysTick_Handler);
        }
        else { /* the button is released */
            static QEvt const serveEvt = { SERVE_SIG, 0U, 0U};
            QACTIVE_PUBLISH(&serveEvt, &l_SysTick_Handler);
        }
    }
    QV_ARM_ERRATUM_838869();
}
/*..........................................................................*/
#if Q_SPY
void USART2_IRQHandler(void) { /* kernel UNAWARE interrupt */
    /* is RX register NOT empty? */
    if ((USART2->SR & (1U << 5)) != 0) {
        uint32_t b = USART2->DR;
        QS_RX_PUT(b);
    }
    QV_ARM_ERRATUM_838869();
}
#endif

/* BSP functions ===========================================================*/
void BSP_init(void) {
    /* NOTE: SystemInit() has been already called from the startup code
    *  but SystemCoreClock needs to be updated
    */
    SystemCoreClockUpdate();

    /* enable GPIOA clock port for the LED LD2 */
    RCC->AHBENR |= (1U << 0);

    /* configure LED (PA.5) pin as push-pull output, no pull-up, pull-down */
    GPIOA->MODER   &= ~((3U << 2U*LED_PIN));
    GPIOA->MODER   |=  ((1U << 2U*LED_PIN));
    GPIOA->OTYPER  &= ~((1U <<    LED_PIN));
    GPIOA->OSPEEDR &= ~((3U << 2U*LED_PIN));
    GPIOA->OSPEEDR |=  ((1U << 2U*LED_PIN));
    GPIOA->PUPDR   &= ~((3U << 2U*LED_PIN));

    /* enable GPIOC clock port for the Button B1 */
    RCC->AHBENR |=  (1U << 2);

    /* configure Button (PC.13) pins as input, no pull-up, pull-down */
    GPIOC->MODER   &= ~(3U << 2U*BTN_PIN);
    GPIOC->OSPEEDR &= ~(3U << 2U*BTN_PIN);
    GPIOC->OSPEEDR |=  (1U << 2U*BTN_PIN);
    GPIOC->PUPDR   &= ~(3U << 2U*BTN_PIN);

    BSP_randomSeed(1234U); /* seed the random number generator */

    /* initialize the QS software tracing... */
    if (QS_INIT((void *)0) == 0U) {
        Q_ERROR();
    }
    QS_OBJ_DICTIONARY(&l_SysTick_Handler);
    QS_USR_DICTIONARY(PHILO_STAT);

    /* setup the QS filters... */
    QS_GLB_FILTER(QS_ALL_RECORDS);
    QS_GLB_FILTER(-QS_QF_TICK);
}
/*..........................................................................*/
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    if (stat[0] == 'h') {
<<<<<<< HEAD
        GPIOA->BSRRL = LED_LD2;  /* turn LED on  */
    }
    else {
        GPIOA->BSRRH = LED_LD2;  /* turn LED off */
=======
        GPIOA->BSRRL = (1U << LED_PIN);  /* turn LED on  */
    }
    else {
        GPIOA->BSRRH = (1U << LED_PIN);  /* turn LED off */
>>>>>>> 503419cfc7b6785562856d24396f6bbe6d9cf4a3
    }

    QS_BEGIN_ID(PHILO_STAT, AO_Philo[n]->prio) /* app-specific record */
        QS_U8(1, n);                  /* Philosopher number */
        QS_STR(stat);                 /* Philosopher status */
    QS_END()
}
/*..........................................................................*/
void BSP_displayPaused(uint8_t paused) {
    // not enough LEDs to show the "Paused" status
    if (paused != 0U) {
<<<<<<< HEAD
        //GPIOA->BSRRL = LED_LD2;  /* turn LED on  */
    }
    else {
        //GPIOA->BSRRH = LED_LD2;  /* turn LED off */
=======
        //GPIOA->BSRRL = (1U << LED_PIN);  /* turn LED on  */
    }
    else {
        //GPIOA->BSRRH = (1U << LED_PIN);  /* turn LED off */
>>>>>>> 503419cfc7b6785562856d24396f6bbe6d9cf4a3
    }
}
/*..........................................................................*/
uint32_t BSP_random(void) { /* a very cheap pseudo-random-number generator */
    /* "Super-Duper" Linear Congruential Generator (LCG)
    * LCG(2^32, 3*7*11*13*23, 0, seed)
    */
    l_rnd = l_rnd * (3U*7U*11U*13U*23U);
    return l_rnd >> 8;
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

    /* set priorities of ALL ISRs used in the system, see NOTE00
    *
    * !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    * Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
    * DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
    */
    NVIC_SetPriority(SysTick_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 1);
    NVIC_SetPriority(USART2_IRQn,    0); /* kernel UNAWARE interrupt */
    /* ... */

    /* enable IRQs... */
#ifdef Q_SPY
    NVIC_EnableIRQ(USART2_IRQn); /* UART2 interrupt used for QS-RX */
#endif
}
/*..........................................................................*/
void QF_onCleanup(void) {
}
/*..........................................................................*/
void QV_onIdle(void) {  /* called with interrupts disabled, see NOTE01 */

    /* toggle an LED on and then off (not enough LEDs, see NOTE02) */
<<<<<<< HEAD
    //GPIOA->BSRRL = LED_LD2;  /* turn LED[n] on  */
    //GPIOA->BSRRH = LED_LD2;  /* turn LED[n] off */
=======
    //GPIOA->BSRRL = (1U << LED_PIN); /* turn LED[n] on  */
    //GPIOA->BSRRH = (1U << LED_PIN); /* turn LED[n] off */
>>>>>>> 503419cfc7b6785562856d24396f6bbe6d9cf4a3

#ifdef Q_SPY
    QF_INT_ENABLE();
    QS_rxParse();  /* parse all the received bytes */

    if ((USART2->SR & (1U << 7U)) != 0U) {  /* is TXE empty? */
        uint16_t b;

        QF_INT_DISABLE();
        b = QS_getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {  /* not End-Of-Data? */
            USART2->DR = (b & 0xFFU);  /* put into the DR register */
        }
    }
#elif defined NDEBUG
    /* Put the CPU and peripherals to the low-power mode.
    * you might need to customize the clock management for your application,
    * see the datasheet for your particular Cortex-M MCU.
    */
    /* !!!CAUTION!!!
    * QV_CPU_SLEEP() contains the WFI instruction, which stops the CPU
    * clock, which unfortunately disables the JTAG port, so the ST-Link
    * debugger can no longer connect to the board. For that reason, the call
    * to QV_CPU_SLEEP() has to be used with CAUTION.
    */
    /* NOTE: If you find your board "frozen" like this, strap BOOT0 to VDD and
    * reset the board, then connect with ST-Link Utilities and erase the part.
    * The trick with BOOT(0) is it gets the part to run the System Loader
    * instead of your broken code. When done disconnect BOOT0, and start over.
    */
    //QV_CPU_SLEEP();  /* atomically go to sleep and enable interrupts */
    QF_INT_ENABLE(); /* for now, just enable interrupts */
#else
    QF_INT_ENABLE(); /* just enable interrupts */
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
#define __DIV(__PCLK, __BAUD)       (((__PCLK / 4) *25)/(__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   \
    (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) \
        * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) \
    ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))

/* USART2 pins PA.2 and PA.3 */
#define USART2_TX_PIN 2U
#define USART2_RX_PIN 3U

/*..........................................................................*/
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsBuf[2*1024]; /* buffer for Quantum Spy */
    static uint8_t qsRxBuf[128];  /* buffer for QS-RX channel */

    (void)arg; /* avoid the "unused parameter" compiler warning */

    QS_initBuf(qsBuf, sizeof(qsBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    /* enable peripheral clock for USART2 */
    RCC->AHBENR  |= ( 1U <<  0U);  /* Enable GPIOA clock   */
    RCC->APB1ENR |= ( 1U << 17U);  /* Enable USART#2 clock */

    /* Configure PA3 to USART2_RX, PA2 to USART2_TX */
    GPIOA->AFR[0] &= ~((15U << 4U*USART2_RX_PIN) | (15U << 4U*USART2_TX_PIN));
    GPIOA->AFR[0] |=  (( 7U << 4U*USART2_RX_PIN) | ( 7U << 4U*USART2_TX_PIN));
    GPIOA->MODER  &= ~(( 3U << 2U*USART2_RX_PIN) | ( 3U << 2U*USART2_TX_PIN));
    GPIOA->MODER  |=  (( 2U << 2U*USART2_RX_PIN) | ( 2U << 2U*USART2_TX_PIN));

    USART2->BRR  = __USART_BRR(SystemCoreClock, 115200U);  /* baud rate */
    USART2->CR3  = 0x0000U;         /* no flow control     */
    USART2->CR2  = 0x0000U;         /* 1 stop bit          */
    USART2->CR1  = ((1U <<  2U) |   /* enable RX           */
                    (1U <<  3U) |   /* enable TX           */
                    (1U <<  5U) |   /* enable RX interrupt */
                    (0U << 12U) |   /* 8 data bits         */
                    (0U << 28U) |   /* 8 data bits         */
                    (1U << 13U));   /* enable USART             */

    QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */

    return 1U; /* return success */
}
/*..........................................................................*/
void QS_onCleanup(void) {
}
/*..........................................................................*/
QSTimeCtr QS_onGetTime(void) { /* NOTE: invoked with interrupts DISABLED */
    if ((SysTick->CTRL & 0x00010000) == 0) {  /* COUNT no set? */
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
    while ((b = QS_getByte()) != QS_EOD) {    /* while not End-Of-Data... */
        QF_INT_ENABLE();
        while ((USART2->SR & (1U << 7U)) == 0U) { /* while TXE not empty */
        }
        USART2->DR = (b & 0xFFU);  /* put into the DR register */
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
}

#endif /* Q_SPY */
/*--------------------------------------------------------------------------*/

/*****************************************************************************
* NOTE00:
* The QF_AWARE_ISR_CMSIS_PRI constant from the QF port specifies the highest
* ISR priority that is disabled by the QF framework. The value is suitable
* for the NVIC_SetPriority() CMSIS function.
*
* Only ISRs prioritized at or below the QF_AWARE_ISR_CMSIS_PRI level (i.e.,
* with the numerical values of priorities equal or higher than
* QF_AWARE_ISR_CMSIS_PRI) are allowed to call any QF services. These ISRs
* are "QF-aware".
*
* Conversely, any ISRs prioritized above the QF_AWARE_ISR_CMSIS_PRI priority
* level (i.e., with the numerical values of priorities less than
* QF_AWARE_ISR_CMSIS_PRI) are never disabled and are not aware of the kernel.
* Such "QF-unaware" ISRs cannot call any QF services. The only mechanism
* by which a "QF-unaware" ISR can communicate with the QF framework is by
* triggering a "QF-aware" ISR, which can post/publish events.
*
* NOTE01:
* The QV_onIdle() callback is called with interrupts disabled, because the
* determination of the idle condition might change by any interrupt posting
* an event. QV_onIdle() must internally enable interrupts, ideally
* atomically with putting the CPU to the power-saving mode.
*
* NOTE02:
* The User LED is used to visualize the idle loop activity. The brightness
* The User LED is used to visualize the idle loop activity. The brightness
* of the LED is proportional to the frequency of invcations of the idle loop.
* Please note that the LED is toggled with interrupts locked, so no interrupt
* execution time contributes to the brightness of the User LED.
*/
