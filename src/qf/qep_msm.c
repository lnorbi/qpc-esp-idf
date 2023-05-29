/*$file${src::qf::qep_msm.c} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*
* Model: qpc.qm
* File:  ${src::qf::qep_msm.c}
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
/*$endhead${src::qf::qep_msm.c} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
#define QP_IMPL           /* this is QP implementation */
#include "qep_port.h"     /* QEP port */
#include "qsafety.h"      /* QP Functional Safety (FuSa) System */
#ifdef Q_SPY              /* QS software tracing enabled? */
    #include "qs_port.h"  /* QS port */
    #include "qs_pkg.h"   /* QS facilities for pre-defined trace records */
#else
    #include "qs_dummy.h" /* disable the QS software tracing */
#endif /* Q_SPY */

Q_DEFINE_THIS_MODULE("qep_msm")

/*==========================================================================*/
/*! internal QEP constants */

/*! top state object for QMsm-style state machines. */
static struct QMState const l_msm_top_s = {
    (struct QMState *)0,
    Q_STATE_CAST(0),
    Q_ACTION_CAST(0),
    Q_ACTION_CAST(0),
    Q_ACTION_CAST(0)
};

/*! maximum depth of entry levels in a MSM for transition to history. */
enum { QMSM_MAX_ENTRY_DEPTH_ = 4};

/*==========================================================================*/
/*$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/* Check for the minimum required QP version */
#if (QP_VERSION < 700U) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8U))
#error qpc version 7.0.0 or higher required
#endif
/*$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

/*$define${QEP::QMsm} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

/*${QEP::QMsm} .............................................................*/

/*${QEP::QMsm::isInState} ..................................................*/
/*! @public @memberof QMsm */
bool QMsm_isInState(QMsm const * const me,
    QMState const * const state)
{
    bool inState = false; /* assume that this MSM is not in 'state' */

    for (QMState const *s = me->super.state.obj;
         s != (QMState *)0;
         s = s->superstate)
    {
        if (s == state) {
            inState = true; /* match found, return 'true' */
            break;
        }
    }
    return inState;
}

/*${QEP::QMsm::stateObj} ...................................................*/
/*! @public @memberof QMsm */
QMState const * QMsm_stateObj(QHsm const * const me) {
    return me->state.obj;
}

/*${QEP::QMsm::childStateObj} ..............................................*/
/*! @public @memberof QMsm */
QMState const * QMsm_childStateObj(
    QHsm const * const me,
    QMState const * const parent)
{
    QMState const *child = me->state.obj;
    bool isFound = false; /* start with the child not found */
    QMState const *s;

    for (s = me->state.obj; s != (QMState *)0; s = s->superstate) {
        if (s == parent) {
            isFound = true; /* child is found */
            break;
        }
        else {
            child = s;
        }
    }
    if (!isFound) { /* still not found? */
        for (s = me->temp.obj; s != (QMState *)0; s = s->superstate) {
            if (s == parent) {
                isFound = true; /* child is found */
                break;
            }
            else {
                child = s;
            }
        }
    }

    Q_ENSURE_ID(890, isFound);

    #ifdef QP_NDBC
    Q_UNUSED_PAR(isFound);
    #endif

    return child; /* return the child */
}

/*${QEP::QMsm::ctor} .......................................................*/
/*! @protected @memberof QMsm */
void QMsm_ctor(QMsm * const me,
    QStateHandler const initial)
{
    static struct QHsmVtable const vtable = { /* QHsm virtual table */
        &QMsm_init_,
        &QMsm_dispatch_
    #ifdef Q_SPY
        ,&QMsm_getStateHandler_
    #endif
    };
    /* do not call the QHsm_ctor() here */
    me->super.vptr = &vtable;
    me->super.state.obj = &l_msm_top_s; /* the current state (top) */
    me->super.temp.fun  = initial;      /* the initial transition handler */
}

/*${QEP::QMsm::init_} ......................................................*/
/*! @private @memberof QMsm */
void QMsm_init_(
    QHsm * const me,
    void const * const e,
    uint_fast8_t const qs_id)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(qs_id);
    #endif

    Q_REQUIRE_ID(200, (me->vptr != (struct QHsmVtable *)0)
                      && (me->temp.fun != Q_STATE_CAST(0))
                      && (me->state.obj == &l_msm_top_s));

    /* execute the top-most initial tran. */
    QState r = (*me->temp.fun)(me, Q_EVT_CAST(QEvt));

    /* the top-most initial transition must be taken */
    Q_ASSERT_ID(210, r == Q_RET_TRAN_INIT);

    QS_CRIT_STAT_
    QS_BEGIN_PRE_(QS_QEP_STATE_INIT, qs_id)
        QS_OBJ_PRE_(me); /* this state machine object */
        QS_FUN_PRE_(me->state.obj->stateHandler);          /* source state */
        QS_FUN_PRE_(me->temp.tatbl->target->stateHandler); /* target state */
    QS_END_PRE_()

    /* set state to the last tran. target */
    me->state.obj = me->temp.tatbl->target;

    /* drill down into the state hierarchy with initial transitions... */
    /* execute the tran. table */
    do {
        r = QMsm_execTatbl_(me, me->temp.tatbl, qs_id);
    } while (r >= Q_RET_TRAN_INIT);

    QS_BEGIN_PRE_(QS_QEP_INIT_TRAN, qs_id)
        QS_TIME_PRE_();   /* time stamp */
        QS_OBJ_PRE_(me);  /* this state machine object */
        QS_FUN_PRE_(me->state.obj->stateHandler); /* the new current state */
    QS_END_PRE_()

    #ifndef QP_NDBC
    me->temp.uint = ~me->state.uint;
    #endif
}

/*${QEP::QMsm::dispatch_} ..................................................*/
/*! @private @memberof QMsm */
void QMsm_dispatch_(
    QHsm * const me,
    QEvt const * const e,
    uint_fast8_t const qs_id)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(qs_id);
    #endif

    QMState const *s = me->state.obj; /* store the current state */
    QMState const *t = s;

<<<<<<< HEAD
    Q_REQUIRE_ID(300, s != (QMState *)0);
=======
    Q_REQUIRE_ID(302, (s != (QMState *)0)
                      && (me->state.uint == (uintptr_t)(~me->temp.uint)));
>>>>>>> 503419cfc7b6785562856d24396f6bbe6d9cf4a3

    QS_CRIT_STAT_
    QS_BEGIN_PRE_(QS_QEP_DISPATCH, qs_id)
        QS_TIME_PRE_();               /* time stamp */
        QS_SIG_PRE_(e->sig);          /* the signal of the event */
        QS_OBJ_PRE_(me);              /* this state machine object */
        QS_FUN_PRE_(s->stateHandler); /* the current state handler */
    QS_END_PRE_()

    /* scan the state hierarchy up to the top state... */
    QState r;
    do {
        r = (*t->stateHandler)(me, e);  /* call state handler function */

        /* event handled? (the most frequent case) */
        if (r >= Q_RET_HANDLED) {
            break; /* done scanning the state hierarchy */
        }
        /* event unhandled and passed to the superstate? */
        else if (r == Q_RET_SUPER) {
            t = t->superstate; /* advance to the superstate */
        }
        /* event unhandled and passed to a submachine superstate? */
        else if (r == Q_RET_SUPER_SUB) {
            t = me->temp.obj; /* current host state of the submachie */
        }
        /* event unhandled due to a guard? */
        else if (r == Q_RET_UNHANDLED) {

            QS_BEGIN_PRE_(QS_QEP_UNHANDLED, qs_id)
                QS_SIG_PRE_(e->sig);  /* the signal of the event */
                QS_OBJ_PRE_(me);      /* this state machine object */
                QS_FUN_PRE_(t->stateHandler); /* the current state */
            QS_END_PRE_()

            t = t->superstate; /* advance to the superstate */
        }
        else {
            /* no other return value should be produced */
            Q_ERROR_ID(310);
        }
    } while (t != (QMState *)0);


    /* any kind of transition taken? */
    if (r >= Q_RET_TRAN) {
    #ifdef Q_SPY
        QMState const * const ts = t; /* transition source for QS tracing */

        /* the transition source state must not be NULL */
        Q_ASSERT_ID(320, ts != (QMState *)0);
    #endif /* Q_SPY */

        do {
            /* save the transition-action table before it gets clobbered */
            struct QMTranActTable const * const tatbl = me->temp.tatbl;
            union QHsmAttr tmp; /* temporary to save intermediate values */

            /* was TRAN, TRAN_INIT, or TRAN_EP taken? */
            if (r <= Q_RET_TRAN_EP) {
                me->temp.obj = (QMState *)0; /* clear */
                QMsm_exitToTranSource_(me, s, t, qs_id);
                r = QMsm_execTatbl_(me, tatbl, qs_id);
                s = me->state.obj;
            }
            /* was a transition segment to history taken? */
            else if (r == Q_RET_TRAN_HIST) {
                tmp.obj = me->state.obj; /* save history */
                me->state.obj = s; /* restore the original state */
                QMsm_exitToTranSource_(me, s, t, qs_id);
                (void)QMsm_execTatbl_(me, tatbl, qs_id);
                r = QMsm_enterHistory_(me, tmp.obj, qs_id);
                s = me->state.obj;
            }
            /* was a transition segment to an exit point taken? */
            else if (r == Q_RET_TRAN_XP) {
                tmp.act = me->state.act; /* save XP action */
                me->state.obj = s; /* restore the original state */
                r = (*tmp.act)(me); /* execute the XP action */
                if (r == Q_RET_TRAN) { /* XP -> TRAN ? */
                    tmp.tatbl = me->temp.tatbl; /* save me->temp */
                    QMsm_exitToTranSource_(me, s, t, qs_id);
                    /* take the tran-to-XP segment inside submachine */
                    (void)QMsm_execTatbl_(me, tatbl, qs_id);
                    s = me->state.obj;
    #ifdef Q_SPY
                    me->temp.tatbl = tmp.tatbl; /* restore me->temp */
    #endif /* Q_SPY */
                }
                else if (r == Q_RET_TRAN_HIST) { /* XP -> HIST ? */
                    tmp.obj = me->state.obj; /* save the history */
                    me->state.obj = s; /* restore the original state */
                    s = me->temp.obj; /* save me->temp */
                    QMsm_exitToTranSource_(me, me->state.obj, t, qs_id);
                    /* take the tran-to-XP segment inside submachine */
                    (void)QMsm_execTatbl_(me, tatbl, qs_id);
    #ifdef Q_SPY
                    me->temp.obj = s; /* restore me->temp */
    #endif /* Q_SPY */
                    s = me->state.obj;
                    me->state.obj = tmp.obj; /* restore the history */
                }
                else {
                    /* TRAN_XP must NOT be followed by any other tran type */
                    Q_ASSERT_ID(330, r < Q_RET_TRAN);
                }
            }
            else {
                /* no other return value should be produced */
                Q_ERROR_ID(340);
            }

            t = s; /* set target to the current state */

        } while (r >= Q_RET_TRAN);

        QS_BEGIN_PRE_(QS_QEP_TRAN, qs_id)
            QS_TIME_PRE_();                /* time stamp */
            QS_SIG_PRE_(e->sig);           /* the signal of the event */
            QS_OBJ_PRE_(me);               /* this state machine object */
            QS_FUN_PRE_(ts->stateHandler); /* the transition source */
            QS_FUN_PRE_(s->stateHandler);  /* the new active state */
        QS_END_PRE_()
    }

    #ifdef Q_SPY
    /* was the event handled? */
    else if (r == Q_RET_HANDLED) {
        /* internal tran. source can't be NULL */
        Q_ASSERT_ID(340, t != (QMState *)0);

        QS_BEGIN_PRE_(QS_QEP_INTERN_TRAN, qs_id)
            QS_TIME_PRE_();                /* time stamp */
            QS_SIG_PRE_(e->sig);           /* the signal of the event */
            QS_OBJ_PRE_(me);               /* this state machine object */
            QS_FUN_PRE_(t->stateHandler);  /* the source state */
        QS_END_PRE_()

    }
    /* event bubbled to the 'top' state? */
    else if (t == (QMState *)0) {

        QS_BEGIN_PRE_(QS_QEP_IGNORED, qs_id)
            QS_TIME_PRE_();                /* time stamp */
            QS_SIG_PRE_(e->sig);           /* the signal of the event */
            QS_OBJ_PRE_(me);               /* this state machine object */
            QS_FUN_PRE_(s->stateHandler);  /* the current state */
        QS_END_PRE_()

    }
    #endif /* Q_SPY */
    else {
        /* empty */
    }

    #ifndef QP_NDBC
    me->temp.uint = ~me->state.uint;
    #endif
}

/*${QEP::QMsm::getStateHandler_} ...........................................*/
#ifdef Q_SPY
/*! @public @memberof QMsm */
QStateHandler QMsm_getStateHandler_(QHsm * const me) {
    return me->state.obj->stateHandler;
}
#endif /* def Q_SPY */

/*${QEP::QMsm::execTatbl_} .................................................*/
/*! @private @memberof QMsm */
QState QMsm_execTatbl_(
    QHsm * const me,
    QMTranActTable const * const tatbl,
    uint_fast8_t const qs_id)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(qs_id);
    #endif

    /*! @pre the transition-action table pointer must not be NULL */
    Q_REQUIRE_ID(400, tatbl != (struct QMTranActTable *)0);

    QState r = Q_RET_NULL;
    QS_CRIT_STAT_

    for (QActionHandler const *a = &tatbl->act[0];
         *a != Q_ACTION_CAST(0);
         ++a)
    {
        r = (*(*a))(me); /* call the action through the 'a' pointer */
    #ifdef Q_SPY
        if (r == Q_RET_ENTRY) {

            QS_BEGIN_PRE_(QS_QEP_STATE_ENTRY, qs_id)
                QS_OBJ_PRE_(me); /* this state machine object */
                QS_FUN_PRE_(me->temp.obj->stateHandler);/*entered state */
            QS_END_PRE_()
        }
        else if (r == Q_RET_EXIT) {

            QS_BEGIN_PRE_(QS_QEP_STATE_EXIT, qs_id)
                QS_OBJ_PRE_(me); /* this state machine object */
                QS_FUN_PRE_(me->temp.obj->stateHandler); /* exited state */
            QS_END_PRE_()
        }
        else if (r == Q_RET_TRAN_INIT) {

            QS_BEGIN_PRE_(QS_QEP_STATE_INIT, qs_id)
                QS_OBJ_PRE_(me); /* this state machine object */
                QS_FUN_PRE_(tatbl->target->stateHandler);         /* source */
                QS_FUN_PRE_(me->temp.tatbl->target->stateHandler);/* target */
            QS_END_PRE_()
        }
        else if (r == Q_RET_TRAN_EP) {

            QS_BEGIN_PRE_(QS_QEP_TRAN_EP, qs_id)
                QS_OBJ_PRE_(me); /* this state machine object */
                QS_FUN_PRE_(tatbl->target->stateHandler);         /* source */
                QS_FUN_PRE_(me->temp.tatbl->target->stateHandler);/* target */
            QS_END_PRE_()
        }
        else if (r == Q_RET_TRAN_XP) {

            QS_BEGIN_PRE_(QS_QEP_TRAN_XP, qs_id)
                QS_OBJ_PRE_(me); /* this state machine object */
                QS_FUN_PRE_(tatbl->target->stateHandler);         /* source */
                QS_FUN_PRE_(me->temp.tatbl->target->stateHandler);/* target */
            QS_END_PRE_()
        }
        else {
            /* empty */
        }
    #endif /* Q_SPY */
    }

    me->state.obj = (r >= Q_RET_TRAN)
        ? me->temp.tatbl->target
        : tatbl->target;
    return r;
}

/*${QEP::QMsm::exitToTranSource_} ..........................................*/
/*! @private @memberof QMsm */
void QMsm_exitToTranSource_(
    QHsm * const me,
    QMState const * const cs,
    QMState const * const ts,
    uint_fast8_t const qs_id)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(qs_id);
    #endif

    /* exit states from the current state to the tran. source state */
    QMState const *s = cs;
    while (s != ts) {
        /* exit action provided in state 's'? */
        if (s->exitAction != Q_ACTION_CAST(0)) {
            (void)(*s->exitAction)(me); /* execute the exit action */

            QS_CRIT_STAT_
            QS_BEGIN_PRE_(QS_QEP_STATE_EXIT, qs_id)
                QS_OBJ_PRE_(me);              /* this state machine object */
                QS_FUN_PRE_(s->stateHandler); /* the exited state handler */
            QS_END_PRE_()
        }

        s = s->superstate; /* advance to the superstate */

        if (s == (QMState *)0) { /* reached the top of a submachine? */
            s = me->temp.obj; /* the superstate from QM_SM_EXIT() */
            Q_ASSERT_ID(510, s != (QMState *)0); /* must be valid */
        }
    }
}

/*${QEP::QMsm::enterHistory_} ..............................................*/
/*! @private @memberof QMsm */
QState QMsm_enterHistory_(
    QHsm * const me,
    QMState const *const hist,
    uint_fast8_t const qs_id)
{
    #ifndef Q_SPY
    Q_UNUSED_PAR(qs_id);
    #endif

    QMState const *s = hist;
    QMState const *ts = me->state.obj; /* transition source */
    QMState const *epath[QMSM_MAX_ENTRY_DEPTH_];

    QS_CRIT_STAT_
    QS_BEGIN_PRE_(QS_QEP_TRAN_HIST, qs_id)
        QS_OBJ_PRE_(me);                 /* this state machine object */
        QS_FUN_PRE_(ts->stateHandler);   /* source state handler */
        QS_FUN_PRE_(hist->stateHandler); /* target state handler */
    QS_END_PRE_()

    int_fast8_t i = 0;  /* transition entry path index */
    while (s != ts) {
        if (s->entryAction != Q_ACTION_CAST(0)) {
            Q_ASSERT_ID(620, i < QMSM_MAX_ENTRY_DEPTH_);
            epath[i] = s;
            ++i;
        }
        s = s->superstate;
        if (s == (QMState *)0) {
            ts = s; /* force exit from the for-loop */
        }
    }

    /* retrace the entry path in reverse (desired) order... */
    while (i > 0) {
        --i;
        (void)(*epath[i]->entryAction)(me); /* run entry action in epath[i] */

        QS_BEGIN_PRE_(QS_QEP_STATE_ENTRY, qs_id)
            QS_OBJ_PRE_(me);
            QS_FUN_PRE_(epath[i]->stateHandler); /* entered state handler */
        QS_END_PRE_()
    }

    me->state.obj = hist; /* set current state to the transition target */

    /* initial tran. present? */
    QState r;
    if (hist->initAction != Q_ACTION_CAST(0)) {
        r = (*hist->initAction)(me); /* execute the transition action */
    }
    else {
        r = Q_RET_NULL;
    }
    return r;
}
/*$enddef${QEP::QMsm} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
