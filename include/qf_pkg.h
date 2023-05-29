/*$file${include::qf_pkg.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*
* Model: qpc.qm
* File:  ${include::qf_pkg.h}
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
/*$endhead${include::qf_pkg.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*! @file
* @brief Internal (package scope) QF/C interface.
*
* @trace
* - @tr{DVP-QP-MC3-D04_08}
*/
#ifndef QF_PKG_H_
#define QF_PKG_H_

/*==========================================================================*/
/*$declare${QF::QF-pkg} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

/*${QF::QF-pkg::ePool_[QF_MAX_EPOOL]} ......................................*/
#if (QF_MAX_EPOOL > 0U)
/*! array of event pools managed by QF */
extern QF_EPOOL_TYPE_ QF_ePool_[QF_MAX_EPOOL];
#endif /*  (QF_MAX_EPOOL > 0U) */

/*${QF::QF-pkg::maxPool_} ..................................................*/
/*! number of initialized event pools */
extern uint_fast8_t QF_maxPool_;

/*${QF::QF-pkg::readySet_} .................................................*/
/*! "Ready-set" of all threads used in the built-in kernels
* @static @private @memberof QF
*/
extern QPSet QF_readySet_;

/*${QF::QF-pkg::readySet_inv_} .............................................*/
#ifndef Q_UNSAFE
/*! Inverted copy of QF_readySet_ (duplicate storage, part of Self-Monitoring)
* @static @private @memberof QF
*/
extern QPSet QF_readySet_inv_;
#endif /* ndef Q_UNSAFE */

/*${QF::QF-pkg::bzero} .....................................................*/
/*! Clear a specified region of memory to zero.
* @static @public @memberof QF
*
* @details
* Clears a memory buffer by writing zeros byte-by-byte.
*
* @param[in]  start  pointer to the beginning of a memory buffer.
* @param[in]  len    length of the memory buffer to clear (in bytes)
*
* @note The main application of this function is clearing the internal QF
* variables upon startup. This is done to avoid problems with non-standard
* startup code provided with some compilers and toolsets (e.g., TI DSPs or
* Microchip MPLAB), which does not zero the uninitialized variables, as
* required by the ANSI C standard.
*
* @trace
* - @tr{DVR-QP-MC3-R11_05}
*/
void QF_bzero(
    void * const start,
    uint_fast16_t const len);
/*$enddecl${QF::QF-pkg} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

/*==========================================================================*/
/* The following bitmasks are for the fields of the @c refCtr_ attribute
* of the QTimeEvt struct (inherited from QEvt). This attribute is NOT used
* for reference counting in time events, because the @c poolId_ attribute
* is zero ("immutable events").
*/
#define QTE_IS_LINKED      (1U << 7U)
#define QTE_WAS_DISARMED   (1U << 6U)
#define QTE_TICK_RATE      0x0FU

/*! @brief structure representing a free block in the Native QF Memory Pool */
typedef struct QFreeBlock {
    struct QFreeBlock * volatile next;
} QFreeBlock;

/* internal helper macros ==================================================*/

/*! increment the refCtr of a const event (requires casting `const` away)
* @private @memberof QEvt
*
* @trace
<<<<<<< HEAD
* @tr{PQP11_8}
=======
* - @tr{DVR-QP-MC3-R11_08}
>>>>>>> 503419cfc7b6785562856d24396f6bbe6d9cf4a3
*/
static inline void QEvt_refCtr_inc_(QEvt const *me) {
    ++((QEvt *)me)->refCtr_;
}

/*! decrement the refCtr of a const event (requires casting `const` away)
* @private @memberof QEvt
*
* @trace
<<<<<<< HEAD
* @tr{PQP11_8}
=======
* - @tr{DVR-QP-MC3-R11_08}
>>>>>>> 503419cfc7b6785562856d24396f6bbe6d9cf4a3
*/
static inline void QEvt_refCtr_dec_(QEvt const *me) {
    --((QEvt *)me)->refCtr_;
}

#endif /* QF_PKG_H_ */
