/*============================================================================
* QF/C port to ARM Cortex-R, QK, GNU-ARM
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
* @date Last updated on: 2023-05-23
* @version Last updated for: @ref qpc_7_3_0
*
* @file
* @brief QF/C port to Cortex-R, preemptive QK kernel, GNU-ARM toolset
*/
#ifndef QF_PORT_H_
#define QF_PORT_H_

/* The maximum number of active objects in the application, see NOTE1 */
#define QF_MAX_ACTIVE           32U

/* The maximum number of system clock tick rates */
#define QF_MAX_TICK_RATE        2U

/* QF interrupt disable/enable, see NOTE2 */
#ifdef __thumb__  /* THUMB mode? */

    #define QF_INT_DISABLE()    __asm volatile ("cpsid i")
    #define QF_INT_ENABLE()     __asm volatile ("cpsie i")
    #define QF_INT_ENABLE_ALL() __asm volatile ("cpsie if")

#elif (defined __arm__) /* ARM mode? */

    #define QF_INT_DISABLE()   \
        __asm volatile ("MSR cpsr_c,#(0x1F | 0x80)" ::: "cc")
    #define QF_INT_ENABLE() \
        __asm volatile ("MSR cpsr_c,#(0x1F)" ::: "cc")
    #define QF_INT_ENABLE_ALL() \
        __asm volatile ("MSR cpsr_c,#(0x1F | 0x80 | 0x40)" ::: "cc")
#else
    #error Incorrect CPU mode. Must be either __arm__ or __thumb__.
#endif

/* Cortex-R provide the CLZ instruction for fast LOG2 */
#define QF_LOG2(n_) ((uint8_t)(32U - __builtin_clz(n_)))

/* QF critical section entry/exit, see NOTE3 */
#define QF_CRIT_STAT_           uint32_t cpsr_;
#define QF_CRIT_E_() do { \
    __asm volatile ("MRS %0,cpsr" : "=r" (cpsr_) :: "cc"); \
    QF_INT_DISABLE(); \
} while (false)

#define QF_CRIT_X_() \
    __asm volatile ("MSR cpsr_c,%0" :: "r" (cpsr_) : "cc")

#define QF_CRIT_EXIT_NOP()     __asm volatile ("ISB")

#include "qep_port.h"   /* QEP port */
#include "qk_port.h"    /* QK port */
#include "qf.h"         /* QF platform-independent public interface */

/*****************************************************************************
* NOTE1:
* The maximum number of active objects QF_MAX_ACTIVE can be increased
* up to 63U, if necessary. Here it is set to a lower level to save some RAM.
*
* NOTE2:
* The FIQ-type interrupts are NEVER disabled in this port, so the FIQ is
* a "kernel-unaware" interrupt. If the FIQ is ever used in the application,
* it must be an "ARM FIQ"-type function. For this to work, the FIQ
* stack needs to be initialized.
*
* NOTE3:
* This port implements the "save and restore" interrupt status policy,
* which again never disables the FIQ-type interrupts. This policy allows
* for nesting critical sections, which is necessary inside IRQ-type
* interrupts that run with interrupts (IRQ) disabled.
*/

#endif /* QF_PORT_H_ */

