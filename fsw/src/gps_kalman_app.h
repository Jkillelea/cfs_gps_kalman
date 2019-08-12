/*=======================================================================================
** File Name:  gps_kalman_app.h
**
** Title:  Header File for GPS_KALMAN Application
**
** $Author:    Jacob Killelea
** $Revision: 1.1 $
** $Date:      2019-06-28
**
** Purpose:  To define GPS_KALMAN's internal macros, data types, global variables and
**           function prototypes
**
** Modification History:
**   Date | Author | Description
**   ---------------------------
**   2019-06-28 | Jacob Killelea | Build #: Code Started
**
**=====================================================================================*/
    
#ifndef _GPS_KALMAN_APP_H_
#define _GPS_KALMAN_APP_H_

/*
** Include Files
*/
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "gps_kalman_platform_cfg.h"
#include "gps_kalman_mission_cfg.h"
#include "gps_kalman_private_ids.h"
#include "gps_kalman_private_types.h"
#include "gps_kalman_perfids.h"
#include "gps_kalman_msgids.h"
#include "gps_kalman_msg.h"

/*
** Local Defines
*/
#define GPS_KALMAN_TIMEOUT_MSEC    1000

/*
** Local Structure Declarations
*/
typedef struct
{
    /* CFE Event table */
    CFE_EVS_BinFilter_t  EventTbl[GPS_KALMAN_EVT_CNT];

    /* CFE scheduling pipe */
    CFE_SB_PipeId_t  SchPipeId; 
    uint16           usSchPipeDepth;
    char             cSchPipeName[OS_MAX_API_NAME];

    /* CFE command pipe */
    CFE_SB_PipeId_t  CmdPipeId;
    uint16           usCmdPipeDepth;
    char             cCmdPipeName[OS_MAX_API_NAME];
    
    /* CFE telemetry pipe */
    CFE_SB_PipeId_t  TlmPipeId;
    uint16           usTlmPipeDepth;
    char             cTlmPipeName[OS_MAX_API_NAME];

    /* Task-related */
    uint32  uiRunStatus;
    
    /* Input data - from I/O devices or subscribed from other apps' output data.
       Data structure should be defined in gps_kalman/fsw/src/gps_kalman_private_types.h */
    GPS_KALMAN_InData_t   InData;

    /* Output data - to be published at the end of a Wakeup cycle.
       Data structure should be defined in gps_kalman/fsw/src/gps_kalman_private_types.h */
    GPS_KALMAN_OutData_t  OutData;

    /* Housekeeping telemetry - for downlink only.
       Data structure should be defined in gps_kalman/fsw/src/gps_kalman_msg.h */
    GPS_KALMAN_HkTlm_t  HkTlm;

    /* TODO:  Add declarations for additional private data here */
} GPS_KALMAN_AppData_t;

/*
** External Global Variables
*/

/*
** Global Variables
*/

/*
** Local Variables
*/

/*
** Local Function Prototypes
*/
int32  GPS_KALMAN_InitEvent(void);
int32  GPS_KALMAN_InitPipe(void);
int32  GPS_KALMAN_InitData(void);
int32  GPS_KALMAN_InitApp(void);

void  GPS_KALMAN_CleanupCallback(void);

int32  GPS_KALMAN_RcvMsg(int32 iBlocking);

void  GPS_KALMAN_ProcessNewData(void);
void  GPS_KALMAN_ProcessNewCmds(void);
void  GPS_KALMAN_ProcessNewAppCmds(CFE_SB_Msg_t*);

int32 GPS_KALMAN_RunFilter(void);

void  GPS_KALMAN_ReportHousekeeping(void);
void  GPS_KALMAN_SendOutData(void);

boolean  GPS_KALMAN_VerifyCmdLength(CFE_SB_Msg_t*, uint16);

void  GPS_KALMAN_AppMain(void);

#endif /* _GPS_KALMAN_APP_H_ */

/*=======================================================================================
** End of file gps_kalman_app.h
**=====================================================================================*/
    
