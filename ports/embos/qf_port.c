/*============================================================================
* QP/C Real-Time Embedded Framework (RTEF)
* Copyright (C) 2005 Quantum Leaps, LLC. All rights reserved.
*
* SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-QL-commercial
*
* This software is dual-licensed under the terms of the open source GNU
* General Public License version 3 (or any later version), or alternatively,
* under the terms of one of the closed source Quantum Leaps commercial
* licenses.
*
* The terms of the open source GNU General Public License version 3
* can be found at: <www.gnu.org/licenses/gpl-3.0>
*
* The terms of the closed source Quantum Leaps commercial licenses
* can be found at: <www.state-machine.com/licensing>
*
* Redistributions in source code must retain this top-level comment block.
* Plagiarizing this software to sidestep the license obligations is illegal.
*
* Contact information:
* <www.state-machine.com>
* <info@state-machine.com>
============================================================================*/
/*!
<<<<<<< HEAD
* @date Last updated on: 2023-04-12
* @version Last updated for: @ref qpc_7_2_2
=======
* @date Last updated on: 2023-05-18
* @version Last updated for: @ref qpc_7_3_0
>>>>>>> 503419cfc7b6785562856d24396f6bbe6d9cf4a3
*
* @file
* @brief QF/C port to embOS
*/
#define QP_IMPL           /* this is QP implementation */
#include "qf_port.h"      /* QF port */
#include "qf_pkg.h"
#include "qsafety.h"      /* QP Functional Safety (FuSa) System */
#ifdef Q_SPY              /* QS software tracing enabled? */
    #include "qs_port.h"  /* QS port */
    #include "qs_pkg.h"   /* QS package-scope internal interface */
#else
    #include "qs_dummy.h" /* disable the QS software tracing */
#endif /* Q_SPY */

Q_DEFINE_THIS_MODULE("qf_port")

/*..........................................................................*/
/* define __TARGET_FPU_VFP symbol depending on the compiler...  */
#if defined (__CC_ARM)          /* ARM Compiler */
    /* in ARM Compiler __TARGET_FPU_VFP is a pre-defined symbol*/
#elif defined (__ICCARM__)      /* IAR Compiler */
    #if defined __ARMVFP__
        #define __TARGET_FPU_VFP 1
    #endif
#elif defined (__GNUC__)        /* GNU Compiler */
    #if defined (__VFP_FP__) && !defined(__SOFTFP__)
        #define __TARGET_FPU_VFP 1
    #endif
#endif

/*..........................................................................*/
void QF_init(void) {
    OS_InitKern();  /* initialize embOS */
    OS_InitHW();    /* initialize the hardware used by embOS */
}
/*..........................................................................*/
int_t QF_run(void) {
    QF_onStartup();  /* QF callback to configure and start interrupts */

    /* produce the QS_QF_RUN trace record */
#ifdef Q_SPY
    QF_CRIT_STAT_
    QF_CRIT_E_();
    QS_beginRec_((uint_fast8_t)QS_QF_RUN);
    QS_endRec_();
    QF_CRIT_X_();
#endif

    OS_Start(); /* start embOS multitasking */

    return 0; /* dummy return to make the compiler happy */
}
/*..........................................................................*/
void QF_stop(void) {
    QF_onCleanup();  /* cleanup callback */
}

/*..........................................................................*/
static void thread_function(void *pVoid) { /* embOS signature */
    QActive *act = (QActive *)pVoid;

#ifdef __TARGET_FPU_VFP
    /* does the task use the FPU? see NOTE1 */
    if ((act->osObject & TASK_USES_FPU) != 0U) {
        OS_ExtendTaskContext_VFP();
    }
#endif  /* __TARGET_FPU_VFP */

    /* event-loop */
    for (;;) {  /* for-ever */
        QEvt const *e = QActive_get_(act);
        /* dispatch event (virtual call) */
        (*act->super.vptr->dispatch)(&act->super, e, act->prio);
        QF_gc(e); /* check if the event is garbage, and collect it if so */
    }
}
/*..........................................................................*/
void QActive_start_(QActive * const me, QPrioSpec const prioSpec,
                    QEvt const * * const qSto, uint_fast16_t const qLen,
                    void * const stkSto, uint_fast16_t const stkSize,
                    void const * const par)
{
    /* create the embOS message box for the AO */
    OS_CreateMB(&me->eQueue,
                (OS_U16)sizeof(QEvt *),
                (OS_UINT)qLen,
                (void *)&qSto[0]);

    me->prio  = (uint8_t)(prioSpec & 0xFFU); /* QF-priority of the AO */
    me->pthre = (uint8_t)(prioSpec >> 8U);   /* preemption-threshold */
    QActive_register_(me); /* register this AO */

    /* top-most initial tran. (virtual call) */
    (*me->super.vptr->init)(&me->super, par, me->prio);
    QS_FLUSH(); /* flush the trace buffer to the host */

    /* create an embOS task for the AO */
    OS_CreateTaskEx(&me->thread,
#if (OS_TRACKNAME != 0)
                    me->thread.Name, /* the configured task name */
#elif
                    "AO",            /* a generic AO task name */
#endif
                    (OS_PRIO)me->prio,/* embOS uses the same numbering as QP*/
                    &thread_function,
                    (void OS_STACKPTR *)stkSto,
                    (OS_UINT)stkSize,
                    (OS_UINT)0,    /* no AOs at the same prio */
                    (void *)me);
}
/*..........................................................................*/
void QActive_setAttr(QActive *const me, uint32_t attr1, void const *attr2) {
    QF_CRIT_STAT_
    QF_CRIT_E_();
    switch (attr1) {
        case TASK_NAME_ATTR: {
#if (OS_TRACKNAME != 0)
            Q_ASSERT_NOCRIT_(300, me->thread.Name == (char *)0);
            me->thread.Name = (char const *)attr2;
#endif
            break;
        }
        case TASK_USES_FPU:
            me->osObject = attr1;
            break;
        /* ... */
        default:
            break;
    }
    QF_CRIT_X_();
}
/*..........................................................................*/
bool QActive_post_(QActive * const me, QEvt const * const e,
                   uint_fast16_t const margin, void const * const sender)
{
    QF_CRIT_STAT_
    QF_CRIT_E_();
    uint_fast16_t nFree =
        (uint_fast16_t)(me->eQueue.maxMsg - me->eQueue.nofMsg);

    bool status;
    if (margin == QF_NO_MARGIN) {
        if (nFree > (QEQueueCtr)0) {
            status = true; /* can post */
        }
        else {
            status = false; /* cannot post */
            Q_ERROR_NOCRIT_(510); /* must be able to post the event */
        }
    }
    else if (nFree > (QEQueueCtr)margin) {
        status = true; /* can post */
    }
    else {
        status = false; /* cannot post */
    }

    if (status) { /* can post the event? */

        QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_POST, me->prio)
            QS_TIME_PRE_();      /* timestamp */
            QS_OBJ_PRE_(sender); /* the sender object */
            QS_SIG_PRE_(e->sig); /* the signal of the event */
            QS_OBJ_PRE_(me);     /* this active object (recipient) */
            QS_2U8_PRE_(e->poolId_, e->refCtr_); /* pool Id & ref Count */
            QS_EQC_PRE_(nFree);  /* # free entries available */
            QS_EQC_PRE_(0U);     /* min # free entries (unknown) */
        QS_END_NOCRIT_PRE_()

        if (e->poolId_ != 0U) { /* is it a pool event? */
            QEvt_refCtr_inc_(e); /* increment the reference counter */
        }
        QF_CRIT_X_();

        char err = OS_PutMailCond(&me->eQueue, (OS_CONST_PTR void *)&e);

        QF_CRIT_E_();
        /* posting to the embOS mailbox must succeed, see NOTE3 */
        Q_ASSERT_NOCRIT_(520, err == (char)0);
    }
    else {

        QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_POST_ATTEMPT, me->prio)
            QS_TIME_PRE_();      /* timestamp */
            QS_OBJ_PRE_(sender); /* the sender object */
            QS_SIG_PRE_(e->sig); /* the signal of the event */
            QS_OBJ_PRE_(me);     /* this active object (recipient) */
            QS_2U8_PRE_(e->poolId_, e->refCtr_); /* pool Id & ref Count */
            QS_EQC_PRE_(nFree);  /* # free entries available */
            QS_EQC_PRE_(margin); /* margin requested */
        QS_END_NOCRIT_PRE_()

    }
    QF_CRIT_X_();

    return status;
}
/*..........................................................................*/
void QActive_postLIFO_(QActive * const me, QEvt const * const e) {
    QF_CRIT_STAT_
    QF_CRIT_E_();

    QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_POST_LIFO, me->prio)
        QS_TIME_PRE_();          /* timestamp */
        QS_SIG_PRE_(e->sig);     /* the signal of this event */
        QS_OBJ_PRE_(me);         /* this active object */
        QS_2U8_PRE_(e->poolId_, e->refCtr_); /* pool Id & ref Count */
        QS_EQC_PRE_(me->eQueue.maxMsg - me->eQueue.nofMsg); /* # free */
        QS_EQC_PRE_(0U);         /* min # free entries (unknown) */
    QS_END_NOCRIT_PRE_()

    if (e->poolId_ != 0U) { /* is it a pool event? */
        QEvt_refCtr_inc_(e); /* increment the reference counter */
    }
    QF_CRIT_X_();

    char err = OS_PutMailFrontCond(&me->eQueue, (OS_CONST_PTR void *)&e);

    QF_CRIT_E_();
    /* posting to the embOS mailbox must succeed, see NOTE3 */
    Q_ASSERT_NOCRIT_(810, err == (char)0);
    QF_CRIT_X_();
}
/*..........................................................................*/
QEvt const *QActive_get_(QActive * const me) {
    QEvt const *e;
    OS_GetMail(&me->eQueue, (void *)&e);

    QS_CRIT_STAT_
    QS_CRIT_E_();
    QS_BEGIN_NOCRIT_PRE_(QS_QF_ACTIVE_GET, me->prio)
        QS_TIME_PRE_();          /* timestamp */
        QS_SIG_PRE_(e->sig);     /* the signal of this event */
        QS_OBJ_PRE_(me);         /* this active object */
        QS_2U8_PRE_(e->poolId_, e->refCtr_); /* pool Id & ref Count */
        QS_EQC_PRE_(me->eQueue.maxMsg - me->eQueue.nofMsg);/* # free */
    QS_END_NOCRIT_PRE_()
    QS_CRIT_X_();

    return e;
}

/*============================================================================
* NOTE1:
* In case of hardware-supported floating point unit (FPU), a task must
* preserve the FPU registers across the context switch. However, this
* additional overhead is necessary only for tasks that actually use the
* FPU. In this QP-embOS port, an active object task that uses the FPU is
* designated by the QF_TASK_USES_FPU attribute, which can be set with the
* QF_setEmbOsTaskAttr() function. The task attributes must be set *before*
* calling QACTIVE_START(). The task attributes are saved in QActive.osObject
* member.
*
* NOTE3:
* The event posting to embOS mailbox occurs OUTSIDE critical section,
* which means that the remaining margin of available slots in the queue
* cannot be guaranteed. The problem is that interrupts and other tasks can
* preempt the event posting after checking the margin, but before actually
* posting the event to the queue.
*/
