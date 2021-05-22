/**
 * \file
 *
 * Example project that demonstrates usage of queues in TNeo.
 */


/*******************************************************************************
 *    INCLUDED FILES
 ******************************************************************************/

#include "os-tasks.h"

#include "tn.h"



/*******************************************************************************
 *    PRIVATE DATA
 ******************************************************************************/

static struct TN_EventGrp que_procon_events;




/*******************************************************************************
 *    PUBLIC FUNCTIONS
 ******************************************************************************/


/**
 * See comments in the header file
 */
void os_tasks_init(void)
{
   //-- create application events 
   //   (see enum E_QueExampleFlag in the header)
   SYSRETVAL_CHECK(tn_eventgrp_create(&que_procon_events, (0)));

   //-- init architecture-dependent stuff
//   queue_example_arch_init();
}

/**
 * See comments in the header file
 */
struct TN_EventGrp *queue_procon_eventgrp_get(void)
{
   return &que_procon_events;
}

/*******************************************************************************
 *    end of file
 ******************************************************************************/



