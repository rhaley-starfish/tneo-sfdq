//
// Created by russh on 6/9/2021.
//

#include "fastcode-tneo-wrapper.h"
#include "fastcode.h"
#include "tn.h"
#include "os-tasks.h"
//-- task stack size
#define  TASK_FASTCODE_STACK_SIZE      (TN_MIN_STACK_SIZE + 96)

//-- priority of consumer task: the highest one
#define  TASK_FASTCODE_PRIORITY        0

//-- number of items in the consumer message queue
//#define  CONS_QUE_BUF_SIZE    4

//-- max timeout for waiting for memory and free message
//#define  WAIT_TIMEOUT         10

//-- define array for task stack
TN_STACK_ARR_DEF(task_fastcode_stack, TASK_FASTCODE_STACK_SIZE);

//-- task descriptor: it's better to explicitly zero it
static struct TN_Task task_fastcode = {};


int task_fastcode_create(void)
{

    SYSRETVAL_CHECK(
            tn_task_create(
                    &task_fastcode,
                    fastcode,
                    TASK_FASTCODE_PRIORITY,
                    task_fastcode_stack,
                    TASK_FASTCODE_STACK_SIZE,
                    TN_NULL,
                    (TN_TASK_CREATE_OPT_START)
            )
    );
}