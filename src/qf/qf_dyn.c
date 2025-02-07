/*$file${src::qf::qf_dyn.c} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*
* Model: qpc.qm
* File:  ${src::qf::qf_dyn.c}
*
* This code has been generated by QM 5.2.5 <www.state-machine.com/qm>.
* DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
*
* This code is covered by the following QP license:
* License #    : LicenseRef-QL-dual
* Issued to    : Any user of the QP/C real-time embedded framework
* Framework(s) : qpc
* Support ends : 2023-12-31
* License scope:
*
* Copyright (C) 2005 Quantum Leaps, LLC <state-machine.com>.
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
* <www.state-machine.com/licensing>
* <info@state-machine.com>
*/
/*$endhead${src::qf::qf_dyn.c} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*! @file
* @brief QF/C dynamic event management
*/
#define QP_IMPL           /* this is QP implementation */
#include "qf_port.h"      /* QF port */
#include "qf_pkg.h"       /* QF package-scope interface */
#include "qsafety.h"      /* QP Functional Safety (FuSa) System */
#ifdef Q_SPY              /* QS software tracing enabled? */
    #include "qs_port.h"  /* QS port */
    #include "qs_pkg.h"   /* QS facilities for pre-defined trace records */
#else
    #include "qs_dummy.h" /* disable the QS software tracing */
#endif /* Q_SPY */

#if (QF_MAX_EPOOL > 0U)  /* dynamic events configured? */

Q_DEFINE_THIS_MODULE("qf_dyn")

//============================================================================
/*$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/* Check for the minimum required QP version */
#if (QP_VERSION < 700U) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8U))
#error qpc version 7.0.0 or higher required
#endif
/*$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

/*$define${QF::QF-pkg::maxPool_} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

/*${QF::QF-pkg::maxPool_} ..................................................*/
uint_fast8_t QF_maxPool_;
/*$enddef${QF::QF-pkg::maxPool_} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*$define${QF::QF-pkg::ePool_[QF_MAX_EPOOL]} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

/*${QF::QF-pkg::ePool_[QF_MAX_EPOOL]} ......................................*/
#if (QF_MAX_EPOOL > 0U)
QF_EPOOL_TYPE_ QF_ePool_[QF_MAX_EPOOL];
#endif /*  (QF_MAX_EPOOL > 0U) */
/*$enddef${QF::QF-pkg::ePool_[QF_MAX_EPOOL]} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
//============================================================================
/*$define${QEP::QEvt} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

/*${QEP::QEvt} .............................................................*/
/*$enddef${QEP::QEvt} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
//============================================================================
/*$define${QF::QF-dyn} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

/*${QF::QF-dyn::poolInit} ..................................................*/
/*! @static @public @memberof QF */
void QF_poolInit(
    void * const poolSto,
    uint_fast32_t const poolSize,
    uint_fast16_t const evtSize)
{
    /* see precondition{qf_dyn,200} */
    QF_CRIT_STAT_
    QF_CRIT_E_();
    Q_REQUIRE_NOCRIT_(200, QF_maxPool_ < QF_MAX_EPOOL);
    Q_REQUIRE_NOCRIT_(201,
        (QF_maxPool_ == 0U)
         || (QF_EPOOL_EVENT_SIZE_(QF_ePool_[QF_maxPool_ - 1U])
             < evtSize));
    QF_CRIT_X_();

    /* perform the port-dependent initialization of the event-pool */
    QF_EPOOL_INIT_(QF_ePool_[QF_maxPool_], poolSto, poolSize, evtSize);
    ++QF_maxPool_; /* one more pool */

    #ifdef Q_SPY
    /* generate the object-dictionary entry for the initialized pool */
    {
        uint8_t obj_name[9] = "EvtPool?";
        obj_name[7] = (uint8_t)(((uint8_t)'0' + QF_maxPool_) & 0x7FU);
        QF_CRIT_E_();
        QS_obj_dict_pre_(&QF_ePool_[QF_maxPool_ - 1U], (char const *)obj_name);
        QF_CRIT_X_();
    }
    #endif /* Q_SPY*/
}

/*${QF::QF-dyn::poolGetMaxBlockSize} .......................................*/
/*! @static @public @memberof QF */
uint_fast16_t QF_poolGetMaxBlockSize(void) {
    return QF_EPOOL_EVENT_SIZE_(QF_ePool_[QF_maxPool_ - 1U]);
}

/*${QF::QF-dyn::getPoolMin} ................................................*/
/*! @static @public @memberof QF */
uint_fast16_t QF_getPoolMin(uint_fast8_t const poolId) {
    QF_CRIT_STAT_
    QF_CRIT_E_();
    Q_REQUIRE_NOCRIT_(400, (poolId <= QF_MAX_EPOOL)
                      && (0U < poolId) && (poolId <= QF_maxPool_));

    uint_fast16_t const min = (uint_fast16_t)QF_ePool_[poolId - 1U].nMin;
    QF_CRIT_X_();

    return min;
}

/*${QF::QF-dyn::newX_} .....................................................*/
/*! @static @private @memberof QF */
QEvt * QF_newX_(
    uint_fast16_t const evtSize,
    uint_fast16_t const margin,
    enum_t const sig)
{
    uint_fast8_t idx;

    /* find the pool index that fits the requested event size ... */
    for (idx = 0U; idx < QF_maxPool_; ++idx) {
        if (evtSize <= QF_EPOOL_EVENT_SIZE_(QF_ePool_[idx])) {
            break;
        }
    }

    QF_CRIT_STAT_
    QF_CRIT_E_();
    /* cannot run out of registered pools */
    Q_REQUIRE_NOCRIT_(300, idx < QF_maxPool_);
    QF_CRIT_X_();

    /* get event e (port-dependent)... */
    QEvt *e;
    #ifdef Q_SPY
    QF_EPOOL_GET_(QF_ePool_[idx], e,
                  ((margin != QF_NO_MARGIN) ? margin : 0U),
                  (uint_fast8_t)QS_EP_ID + idx + 1U);
    #else
    QF_EPOOL_GET_(QF_ePool_[idx], e,
                  ((margin != QF_NO_MARGIN) ? margin : 0U), 0U);
    #endif

    QF_CRIT_E_();
    if (e != (QEvt *)0) { /* was e allocated correctly? */
        e->sig = (QSignal)sig;     /* set signal for this event */
        e->poolId_ = (uint8_t)(idx + 1U); /* store the pool ID */
        e->refCtr_ = 0U; /* set the reference counter to 0 */

        QS_BEGIN_NOCRIT_PRE_(QS_QF_NEW,
               (uint_fast8_t)QS_EP_ID + e->poolId_)
            QS_TIME_PRE_();        /* timestamp */
            QS_EVS_PRE_(evtSize);  /* the size of the event */
            QS_SIG_PRE_(sig);      /* the signal of the event */
        QS_END_NOCRIT_PRE_()
    }
    else { /* event was not allocated */
        /* This assertion means that the event allocation failed,
         * and this failure cannot be tolerated. The most frequent
         * reason is an event leak in the application.
         */
        Q_ASSERT_NOCRIT_(320, margin != QF_NO_MARGIN);

        QS_BEGIN_NOCRIT_PRE_(QS_QF_NEW_ATTEMPT,
               (uint_fast8_t)QS_EP_ID + idx + 1U)
            QS_TIME_PRE_();        /* timestamp */
            QS_EVS_PRE_(evtSize);  /* the size of the event */
            QS_SIG_PRE_(sig);      /* the signal of the event */
        QS_END_NOCRIT_PRE_()
    }
    QF_CRIT_X_();

    /* the returned event e is guaranteed to be valid (not NULL)
    * if we can't tolerate failed allocation
    */
    return e;
}

/*${QF::QF-dyn::gc} ........................................................*/
/*! @static @public @memberof QF */
void QF_gc(QEvt const * const e) {
    if (e->poolId_ != 0U) { /* is it a pool event (dynamic)? */
        QF_CRIT_STAT_
        QF_CRIT_E_();
        if (e->refCtr_ > 1U) { /* isn't this the last reference? */

            QS_BEGIN_NOCRIT_PRE_(QS_QF_GC_ATTEMPT,
                                 (uint_fast8_t)QS_EP_ID + e->poolId_)
                QS_TIME_PRE_();         /* timestamp */
                QS_SIG_PRE_(e->sig);    /* the signal of the event */
                QS_2U8_PRE_(e->poolId_, e->refCtr_); /* pool Id & ref Count */
            QS_END_NOCRIT_PRE_()

            QEvt_refCtr_dec_(e); /* decrement the ref counter */

            QF_CRIT_X_();
        }
        else {  /* this is the last reference to this event, recycle it */
            uint_fast8_t const idx = (uint_fast8_t)e->poolId_ - 1U;

            QS_BEGIN_NOCRIT_PRE_(QS_QF_GC,
                                 (uint_fast8_t)QS_EP_ID + e->poolId_)
                QS_TIME_PRE_();         /* timestamp */
                QS_SIG_PRE_(e->sig);    /* the signal of the event */
                QS_2U8_PRE_(e->poolId_, e->refCtr_); /* pool Id & ref Count */
            QS_END_NOCRIT_PRE_()

            /* pool ID must be in range */
            Q_ASSERT_NOCRIT_(410, idx < QF_maxPool_);

            QF_CRIT_X_();

            /* cast 'const' away, which is OK, because it's a pool event */
    #ifdef Q_SPY
            QF_EPOOL_PUT_(QF_ePool_[idx], (QEvt *)e,
                          (uint_fast8_t)QS_EP_ID + e->poolId_);
    #else
            QF_EPOOL_PUT_(QF_ePool_[idx], (QEvt *)e, 0U);
    #endif
        }
    }
}

/*${QF::QF-dyn::newRef_} ...................................................*/
/*! @static @private @memberof QF */
QEvt const * QF_newRef_(
    QEvt const * const e,
    void const * const evtRef)
{
    #ifdef Q_UNSAFE
    Q_UNUSED_PAR(evtRef);
    #endif

    QF_CRIT_STAT_
    QF_CRIT_E_();
    Q_REQUIRE_NOCRIT_(500,
        (e->poolId_ != 0U)
        && (evtRef == (void *)0));

    QEvt_refCtr_inc_(e); /* increments the ref counter */

    QS_BEGIN_NOCRIT_PRE_(QS_QF_NEW_REF,
                         (uint_fast8_t)QS_EP_ID + e->poolId_)
        QS_TIME_PRE_();      /* timestamp */
        QS_SIG_PRE_(e->sig); /* the signal of the event */
        QS_2U8_PRE_(e->poolId_, e->refCtr_); /* pool Id & ref Count */
    QS_END_NOCRIT_PRE_()

    QF_CRIT_X_();

    return e;
}

/*${QF::QF-dyn::deleteRef_} ................................................*/
/*! @static @private @memberof QF */
void QF_deleteRef_(void const * const evtRef) {
    QEvt const * const e = (QEvt const *)evtRef;

    QS_CRIT_STAT_
    QS_BEGIN_PRE_(QS_QF_DELETE_REF,
                  (uint_fast8_t)QS_EP_ID + e->poolId_)
        QS_TIME_PRE_();      /* timestamp */
        QS_SIG_PRE_(e->sig); /* the signal of the event */
        QS_2U8_PRE_(e->poolId_, e->refCtr_); /* pool Id & ref Count */
    QS_END_PRE_()

    #if (QF_MAX_EPOOL > 0U)
    QF_gc(e);
    #endif
}
/*$enddef${QF::QF-dyn} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

#endif /* (QF_MAX_EPOOL > 0U) dynamic events configured */
