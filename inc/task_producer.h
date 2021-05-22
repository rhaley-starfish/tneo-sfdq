/**
 * \file
 *
 * Example project that demonstrates usage of queues in TNeo.
 */


#ifndef _TASK_PRODUCER_H
#define _TASK_PRODUCER_H

/*******************************************************************************
 *    INCLUDED FILES
 ******************************************************************************/
//#include <cat24C256.h>
#include <math.h>
#include <ad7172.h>
#include <adcManager.h>
#include <bootloader.h>
#include <byteQ.h>
#include <comms.h>
#include <dac.h>
#include <encoder.h>
#include <error.h>
#include <fastcode.h>
#include <fastcodeUtil.h>
#include <feedbackControl.h>
#include <i2c.h>
#include <pac1710.h>
#include <packetReceiver.h>
#include <portExpanderManager.h>
#include <ports.h>
#include <powerOutputs.h>
#include <pwm.h>
#include <sfdqPackets.h>
#include <sfm3019.h>
#include <sigGen.h>
#include <spi.h>
#include <stdbool.h>
#include <stepperMotor.h>
#include <stm32f4xx.h>
#include <stm32f4xx_iwdg.h>
#include <stm32f4xx_rcc.h>
#include <sys/_stdint.h>
#include <system_stm32f4xx.h>
#include <thermistors.h>
#include <vRailMonitor.h>
#include <backupRam.h>
#include <appData.h>
#include <lsm6ds3.h>
#include <eeprom.h>
#include <statusLeds.h>

#include <max31865.h>
#include <amt22.h>
/*******************************************************************************
 *    PUBLIC TYPES
 ******************************************************************************/

/*******************************************************************************
 *    GLOBAL VARIABLES
 ******************************************************************************/

/*******************************************************************************
 *    DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 *    PUBLIC FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * Create producer task.
 */
void init_task_create(void);

#endif // _TASK_PRODUCER_H


/*******************************************************************************
 *    end of file
 ******************************************************************************/


