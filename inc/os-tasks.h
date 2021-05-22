/**
 * \file
 *
 * Example project that demonstrates usage of queues in TNeo.
 */

#ifndef _OS_TASKS_H
#define _OS_TASKS_H

/*******************************************************************************
 *    INCLUDED FILES
 ******************************************************************************/

//-- include architecture-specific things,
//   at least, there is SOFTWARE_BREAK macro
//#include "queue_example_arch.h"
//#include "stm32f4xx_nucleo.h"
#include "tn.h"

#define SOFTWARE_BREAK() while(1) {}
/*******************************************************************************
 *    EXTERN TYPES
 ******************************************************************************/

/// Application common event group (see `enum E_QueExampleFlag`)
struct TN_EventGrp;



/*******************************************************************************
 *    PUBLIC TYPES
 ******************************************************************************/

enum E_QueProConFlag {
   ///
   /// Flag indicating that consumer task is initialized
   QUE_PROCON_FLAG__TASK_CONSUMER_INIT = (1 << 0),
   ///
   /// Flag indicating that producer task is initialized
   QUE_PROCON_FLAG__TASK_PRODUCER_INIT = (1 << 1),
};



/*******************************************************************************
 *    DEFINITIONS
 ******************************************************************************/

/**
 * Macro for checking value returned from system service.
 *
 * If you ignore the value returned by any system service, it is almost always
 * a BAD idea: if something goes wrong, the sooner you know it, the better.
 * But, checking error for every system service is a kind of annoying, so,
 * simple macro was invented for that. 
 *
 * In most situations, any values other than `#TN_RC_OK` or `#TN_RC_TIMEOUT`
 * are not allowed (i.e. should never happen in normal device operation).
 *
 * So, if error value is neither `#TN_RC_OK` nor `#TN_RC_TIMEOUT`, this macro
 * issues a software break.
 *
 * If you need to allow `#TN_RC_OK` only, use `SYSRETVAL_CHECK()` instead (see
 * below)
 *
 * Usage is as follows:
 *
 * \code{.c}
 *    enum TN_RCode rc 
 *       = SYSRETVAL_CHECK_TO(tn_queue_send(&my_queue, p_data, MY_TIMEOUT));
 *
 *    switch (rc){
 *       case TN_RC_OK:
 *          //-- handle successfull operation
 *          break;
 *       case TN_RC_TIMEOUT:
 *          //-- handle timeout
 *          break;
 *       default:
 *          //-- should never be here
 *          break;
 *    }
 * \endcode
 * 
 */
#define SYSRETVAL_CHECK_TO(x)                                     \
   ({                                                             \
      int __rv = (x);                                             \
      if (__rv != TN_RC_OK && __rv != TN_RC_TIMEOUT){             \
         SOFTWARE_BREAK();                                        \
      }                                                           \
      /* like, return __rv */                                     \
      __rv;                                                       \
    })

/**
 * The same as `SYSRETVAL_CHECK_TO()`, but it allows `#TN_RC_OK` only.
 *
 * Since there is only one return code allowed, usage is simple:
 *
 * \code{.c}
 *    SYSRETVAL_CHECK(tn_queue_send(&my_queue, p_data, MY_TIMEOUT));
 * \endcode
 */
#define SYSRETVAL_CHECK(x)                                        \
   ({                                                             \
      int __rv = (x);                                             \
      if (__rv != TN_RC_OK){                                      \
         SOFTWARE_BREAK();                                        \
      }                                                           \
      /* like, return __rv */                                     \
      __rv;                                                       \
    })




/**
 * Returns pointer to the application event group.
 * See `enum E_QueExampleFlag`
 *
 * Do note that you must call `queue_example_init()` before
 * you can get eventgrp.
 */
struct TN_EventGrp *queue_procon_eventgrp_get(void);

void os_tasks_init(void);

#endif //_OS_TASKS_H


/*******************************************************************************
 *    end of file
 ******************************************************************************/


