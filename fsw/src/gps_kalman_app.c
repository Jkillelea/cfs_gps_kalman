/*=======================================================================================
** File Name:  gps_kalman_app.c
**
** Title:  Function Definitions for GPS_KALMAN Application
**
** $Author:    Jacob Killelea
** $Revision: 1.1 $
** $Date:      2019-06-28
**
** Purpose:  This source file contains necessary function definitions to run GPS_KALMAN
**           application.
**
** Functions Defined:
**    Function X - Brief purpose of function X
**    Function Y - Brief purpose of function Y
**    Function Z - Brief purpose of function Z
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to all functions in the file.
**    2. List the external source(s) and event(s) that can cause the funcs in this
**       file to execute.
**    3. List known limitations that apply to the funcs in this file.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Modification History:
**   Date | Author | Description
**   ---------------------------
**   2019-06-28 | Jacob Killelea | Build #: Code Started
**
**=====================================================================================*/

/*
** Include Files
*/
#include <string.h>
#include <math.h>

#include "cfe.h"
#include "cfe_msg.h"

#include "cfe_sb.h"
#include "gps_kalman_platform_cfg.h"
#include "gps_kalman_mission_cfg.h"
#include "gps_kalman_app.h"
#include "gps_kalman_data.h"
#include "gps_kalman_msg.h"
#include "gps_reader_msgids.h"
#include "gps_reader_msgs.h"

#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

/*
** Local Defines
*/
#define GPS_KALMAN_FILTER_LEN (3)

/*
** Global Variables
*/
GPS_KALMAN_AppData_t  g_GPS_KALMAN_AppData;

/*
** Local Variables
*/

/*=====================================================================================
** Name: GPS_KALMAN_InitEvent
**
** Purpose: To initialize and register event table for GPS_KALMAN application
**
** Arguments:
**    None
**
** Returns:
**    int32 iStatus - Status of initialization
**
** Routines Called:
**    CFE_EVS_Register
**    CFE_ES_WriteToSysLog
**
** Called By:
**    GPS_KALMAN_InitApp
**
** Global Inputs/Reads:
**    TBD
**
** Global Outputs/Writes:
**    g_GPS_KALMAN_AppData.EventTbl
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
int32 GPS_KALMAN_InitEvent()
{
    int32  iStatus = CFE_SUCCESS;

    /* Create the event table */
    memset((void*) g_GPS_KALMAN_AppData.EventTbl, 0x00, sizeof(g_GPS_KALMAN_AppData.EventTbl));

    g_GPS_KALMAN_AppData.EventTbl[0].EventID = GPS_KALMAN_RESERVED_EID;
    g_GPS_KALMAN_AppData.EventTbl[1].EventID = GPS_KALMAN_INF_EID;
    g_GPS_KALMAN_AppData.EventTbl[2].EventID = GPS_KALMAN_INIT_INF_EID;
    g_GPS_KALMAN_AppData.EventTbl[3].EventID = GPS_KALMAN_ILOAD_INF_EID;
    g_GPS_KALMAN_AppData.EventTbl[4].EventID = GPS_KALMAN_CDS_INF_EID;
    g_GPS_KALMAN_AppData.EventTbl[5].EventID = GPS_KALMAN_CMD_INF_EID;

    g_GPS_KALMAN_AppData.EventTbl[ 6].EventID = GPS_KALMAN_ERR_EID;
    g_GPS_KALMAN_AppData.EventTbl[ 7].EventID = GPS_KALMAN_INIT_ERR_EID;
    g_GPS_KALMAN_AppData.EventTbl[ 8].EventID = GPS_KALMAN_ILOAD_ERR_EID;
    g_GPS_KALMAN_AppData.EventTbl[ 9].EventID = GPS_KALMAN_CDS_ERR_EID;
    g_GPS_KALMAN_AppData.EventTbl[10].EventID = GPS_KALMAN_CMD_ERR_EID;
    g_GPS_KALMAN_AppData.EventTbl[11].EventID = GPS_KALMAN_PIPE_ERR_EID;
    g_GPS_KALMAN_AppData.EventTbl[12].EventID = GPS_KALMAN_MSGID_ERR_EID;
    g_GPS_KALMAN_AppData.EventTbl[13].EventID = GPS_KALMAN_MSGLEN_ERR_EID;

    /* Register the table with CFE */
    iStatus = CFE_EVS_Register(g_GPS_KALMAN_AppData.EventTbl,
                               GPS_KALMAN_EVT_CNT, CFE_EVS_EventFilter_BINARY);
    if (iStatus != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("GPS_KALMAN - Failed to register with EVS (0x%08X)\n", iStatus);
    }

    return (iStatus);
}

/*=====================================================================================
** Name: GPS_KALMAN_InitPipe
**
** Purpose: To initialize all message pipes and subscribe to messages for GPS_KALMAN application
**
** Arguments:
**    None
**
** Returns:
**    int32 iStatus - Status of initialization
**
** Routines Called:
**    CFE_SB_CreatePipe
**    CFE_SB_Subscribe
**    CFE_ES_WriteToSysLog
**
** Called By:
**    GPS_KALMAN_InitApp
**
** Global Inputs/Reads:
**    None
**
** Global Outputs/Writes:
**    g_GPS_KALMAN_AppData.usSchPipeDepth
**    g_GPS_KALMAN_AppData.cSchPipeName
**    g_GPS_KALMAN_AppData.SchPipeId
**    g_GPS_KALMAN_AppData.usCmdPipeDepth
**    g_GPS_KALMAN_AppData.cCmdPipeName
**    g_GPS_KALMAN_AppData.CmdPipeId
**    g_GPS_KALMAN_AppData.usTlmPipeDepth
**    g_GPS_KALMAN_AppData.cTlmPipeName
**    g_GPS_KALMAN_AppData.TlmPipeId
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
int32 GPS_KALMAN_InitPipe()
{
    int32  iStatus=CFE_SUCCESS;

    /* Init schedule pipe */
    g_GPS_KALMAN_AppData.usSchPipeDepth = GPS_KALMAN_SCH_PIPE_DEPTH;
    memset((void*) g_GPS_KALMAN_AppData.cSchPipeName, '\0', sizeof(g_GPS_KALMAN_AppData.cSchPipeName));
    strncpy(g_GPS_KALMAN_AppData.cSchPipeName, "KALMAN_SCH_PIPE", OS_MAX_API_NAME-1);

    /* Subscribe to Wakeup messages */
    iStatus = CFE_SB_CreatePipe(&g_GPS_KALMAN_AppData.SchPipeId,
                                 g_GPS_KALMAN_AppData.usSchPipeDepth,
                                 g_GPS_KALMAN_AppData.cSchPipeName);
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GPS_KALMAN_WAKEUP_MID), g_GPS_KALMAN_AppData.SchPipeId);

        if (iStatus != CFE_SUCCESS)
        {
            CFE_ES_WriteToSysLog("GPS_KALMAN - Sch Pipe failed to subscribe to GPS_KALMAN_WAKEUP_MID. (0x%08X)\n", iStatus);
            goto GPS_KALMAN_InitPipe_Exit_Tag;
        }

    }
    else
    {
        CFE_ES_WriteToSysLog("GPS_KALMAN - Failed to create SCH pipe (0x%08X)\n", iStatus);
        goto GPS_KALMAN_InitPipe_Exit_Tag;
    }

    /* Init command pipe */
    g_GPS_KALMAN_AppData.usCmdPipeDepth = GPS_KALMAN_CMD_PIPE_DEPTH;
    memset((void*) g_GPS_KALMAN_AppData.cCmdPipeName, '\0', sizeof(g_GPS_KALMAN_AppData.cCmdPipeName));
    strncpy(g_GPS_KALMAN_AppData.cCmdPipeName, "KALMAN_CMD_PIPE", OS_MAX_API_NAME-1);

    /* Subscribe to command messages */
    iStatus = CFE_SB_CreatePipe(&g_GPS_KALMAN_AppData.CmdPipeId,
                                 g_GPS_KALMAN_AppData.usCmdPipeDepth,
                                 g_GPS_KALMAN_AppData.cCmdPipeName);
    if (iStatus == CFE_SUCCESS)
    {
        /* Subscribe to command messages */
        iStatus = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GPS_KALMAN_CMD_MID), g_GPS_KALMAN_AppData.CmdPipeId);

        if (iStatus != CFE_SUCCESS)
        {
            CFE_ES_WriteToSysLog("GPS_KALMAN - CMD Pipe failed to subscribe to GPS_KALMAN_CMD_MID. (0x%08X)\n", iStatus);
            goto GPS_KALMAN_InitPipe_Exit_Tag;
        }

        iStatus  = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GPS_KALMAN_SEND_HK_MID),   g_GPS_KALMAN_AppData.CmdPipeId);

        if (iStatus != CFE_SUCCESS)
        {
            CFE_ES_WriteToSysLog("GPS_KALMAN - CMD Pipe failed to subscribe to GPS_KALMAN_SEND_HK_MID. (0x%08X)\n", iStatus);
            goto GPS_KALMAN_InitPipe_Exit_Tag;
        }

    }
    else
    {
        CFE_ES_WriteToSysLog("GPS_KALMAN - Failed to create CMD pipe (0x%08X)\n", iStatus);
        goto GPS_KALMAN_InitPipe_Exit_Tag;
    }

    /* Init telemetry pipe */
    g_GPS_KALMAN_AppData.usTlmPipeDepth = GPS_KALMAN_TLM_PIPE_DEPTH;
    memset((void*)g_GPS_KALMAN_AppData.cTlmPipeName, '\0', sizeof(g_GPS_KALMAN_AppData.cTlmPipeName));
    strncpy(g_GPS_KALMAN_AppData.cTlmPipeName, "KALMAN_TLM_PIPE", OS_MAX_API_NAME-1);

    /* Subscribe to telemetry messages on the telemetry pipe */
    iStatus = CFE_SB_CreatePipe(&g_GPS_KALMAN_AppData.TlmPipeId,
                                 g_GPS_KALMAN_AppData.usTlmPipeDepth,
                                 g_GPS_KALMAN_AppData.cTlmPipeName);
    if (iStatus == CFE_SUCCESS)
    {
        /* TODO:  Add CFE_SB_Subscribe() calls for other apps' output data here.
        **
        ** Examples:
        **     CFE_SB_Subscribe(GNCEXEC_OUT_DATA_MID, g_GPS_KALMAN_AppData.TlmPipeId);
        */

        /* GPS Reader messages */
        CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GPS_READER_GPS_INFO_MSG), g_GPS_KALMAN_AppData.TlmPipeId);
        /* CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GPS_READER_GPS_GPGGA_MSG), g_GPS_KALMAN_AppData.TlmPipeId); */
        /* CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GPS_READER_GPS_GPGSA_MSG), g_GPS_KALMAN_AppData.TlmPipeId); */
        /* CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GPS_READER_GPS_GPGSV_MSG), g_GPS_KALMAN_AppData.TlmPipeId); */
        /* CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GPS_READER_GPS_GPRMC_MSG), g_GPS_KALMAN_AppData.TlmPipeId); */
        /* CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GPS_READER_GPS_GPVTG_MSG), g_GPS_KALMAN_AppData.TlmPipeId); */

    }
    else
    {
        CFE_ES_WriteToSysLog("GPS_KALMAN - Failed to create TLM pipe (0x%08X)\n", iStatus);
        goto GPS_KALMAN_InitPipe_Exit_Tag;
    }

GPS_KALMAN_InitPipe_Exit_Tag:
    return (iStatus);
}

/*=====================================================================================
** Name: GPS_KALMAN_InitData
**
** Purpose: To initialize global variables used by GPS_KALMAN application
**
** Arguments:
**    None
**
** Returns:
**    int32 iStatus - Status of initialization
**
** Routines Called:
**    CFE_SB_InitMsg
**
** Called By:
**    GPS_KALMAN_InitApp
**
** Global Inputs/Reads:
**    TBD
**
** Global Outputs/Writes:
**    g_GPS_KALMAN_AppData.InData
**    g_GPS_KALMAN_AppData.OutData
**    g_GPS_KALMAN_AppData.HkTlm
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
int32 GPS_KALMAN_InitData()
{
    int32  iStatus = CFE_SUCCESS;

    /* Init input data */
    memset((void*) &g_GPS_KALMAN_AppData.InData, 0x00,
            sizeof(g_GPS_KALMAN_AppData.InData));

    /* Init output data */
    memset((void*) &g_GPS_KALMAN_AppData.OutData, 0x00,
            sizeof(g_GPS_KALMAN_AppData.OutData));
    CFE_MSG_Init((CFE_MSG_Message_t *) &g_GPS_KALMAN_AppData.OutData,
            CFE_SB_ValueToMsgId(GPS_KALMAN_OUT_DATA_MID),
            sizeof(g_GPS_KALMAN_AppData.OutData));

    /* Init housekeeping packet */
    memset((void*)&g_GPS_KALMAN_AppData.HkTlm, 0x00,
            sizeof(g_GPS_KALMAN_AppData.HkTlm));
    CFE_MSG_Init((CFE_MSG_Message_t *) &g_GPS_KALMAN_AppData.HkTlm,
            CFE_SB_ValueToMsgId(GPS_KALMAN_HK_TLM_MID),
            sizeof(g_GPS_KALMAN_AppData.HkTlm));

    /* initalize all the kalman filter elements */
    GPS_KALMAN_Init_Matrix_Data();
    gsl_matrix_set_identity(FMatrix);
    gsl_matrix_set_zero(TmpMatrix);
    gsl_matrix_set_zero(TmpMatrix2);
    gsl_matrix_set_identity(PMatrix);
    gsl_matrix_scale(PMatrix, 999999.0);
    gsl_matrix_set_identity(QMatrix);
    gsl_matrix_scale(QMatrix, 0.1);
    gsl_matrix_set_identity(HMatrix);
    gsl_matrix_set_identity(SigmaExpectMatrix);
    gsl_matrix_set_identity(SigmaActualMatrix);
    gsl_matrix_set_zero(KMatrix);

    return (iStatus);
}

/*=====================================================================================
** Name: GPS_KALMAN_InitApp
**
** Purpose: To initialize all data local to and used by GPS_KALMAN application
**
** Arguments:
**    None
**
** Returns:
**    int32 iStatus - Status of initialization
**
** Routines Called:
**    CFE_ES_WriteToSysLog
**    CFE_EVS_SendEvent
**    OS_TaskInstallDeleteHandler
**    GPS_KALMAN_InitEvent
**    GPS_KALMAN_InitPipe
**    GPS_KALMAN_InitData
**
** Called By:
**    GPS_KALMAN_AppMain
**
** Global Inputs/Reads:
**    TBD
**
** Global Outputs/Writes:
**    TBD
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**     - GSL allocates properly: this is currently not checked
**    2. List the external source(s) and event(s) that can cause this function to execute.
**     - Called by GPS_KALMAN_AppMain
**    3. List known limitations that apply to this function.
**     - Requires dynamic memory allocation
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
int32 GPS_KALMAN_InitApp()
{
    int32  iStatus = CFE_SUCCESS;

    g_GPS_KALMAN_AppData.uiRunStatus = CFE_ES_RunStatus_APP_RUN;

    if ((GPS_KALMAN_InitEvent() != CFE_SUCCESS) ||
        (GPS_KALMAN_InitPipe() != CFE_SUCCESS) ||
        (GPS_KALMAN_InitData() != CFE_SUCCESS))
    {
        iStatus = -1;
        goto GPS_KALMAN_InitApp_Exit_Tag;
    }

    /* Install the cleanup callback */
    OS_TaskInstallDeleteHandler(GPS_KALMAN_CleanupCallback);

GPS_KALMAN_InitApp_Exit_Tag:
    if (iStatus == CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GPS_KALMAN_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "GPS_KALMAN - Application initialized");
    }
    else
    {
        CFE_ES_WriteToSysLog("GPS_KALMAN - Application failed to initialize\n");
    }

    return (iStatus);
}

/*=====================================================================================
** Name: GPS_KALMAN_CleanupCallback
**
** Purpose: To handle any neccesary cleanup prior to application exit
**
** Arguments:
**    None
**
** Returns:
**    None
**
** Routines Called:
**    - gsl_vector_free
**    - gsl_matrix_free
**    - gsl_permutation_free
**
** Called By:
**    - Called by the OS
**
** Global Inputs/Reads:
**    TBD
**
** Global Outputs/Writes:
**    TBD
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**      - Called when the Application is destroyed
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
void GPS_KALMAN_CleanupCallback()
{
    /* TODO:  Add code to cleanup memory and other cleanup here */
}

/*=====================================================================================
** Name: GPS_KALMAN_RcvMsg
**
** Purpose: To receive and process messages for GPS_KALMAN application
**
** Arguments:
**    None
**
** Returns:
**    int32 iStatus - Status of initialization
**
** Routines Called:
**    CFE_SB_ReceiveBuffer
**    CFE_MSG_GetMsgId
**    CFE_EVS_SendEvent
**    CFE_ES_PerfLogEntry
**    CFE_ES_PerfLogExit
**    GPS_KALMAN_ProcessNewCmds
**    GPS_KALMAN_ProcessNewData
**    GPS_KALMAN_SendOutData
**
** Called By:
**    GPS_KALMAN_Main
**
** Global Inputs/Reads:
**    g_GPS_KALMAN_AppData.SchPipeId
**
** Global Outputs/Writes:
**    g_GPS_KALMAN_AppData.uiRunStatus
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
int32 GPS_KALMAN_RcvMsg(int32 iBlocking)
{
    int32            iStatus = CFE_SUCCESS;
    CFE_SB_Buffer_t *MsgPtr  = NULL;
    CFE_SB_MsgId_t   MsgId   = CFE_SB_ValueToMsgId(0);

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(GPS_KALMAN_MAIN_TASK_PERF_ID);

    /* Wait for WakeUp messages from scheduler */
    iStatus = CFE_SB_ReceiveBuffer(&MsgPtr, g_GPS_KALMAN_AppData.SchPipeId, iBlocking);

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(GPS_KALMAN_MAIN_TASK_PERF_ID);

    if (iStatus == CFE_SUCCESS)
    {
        CFE_MSG_GetMsgId(&MsgPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId))
        {
        case GPS_KALMAN_WAKEUP_MID:
            GPS_KALMAN_ProcessNewCmds();
            GPS_KALMAN_ProcessNewData();

            /* TODO:  Add more code here to handle other things when app wakes up */
            GPS_KALMAN_RunFilter();

            /* The last thing to do at the end of this Wakeup cycle should be to
               automatically publish new output. */
            GPS_KALMAN_SendOutData();
            break;

        default:
            CFE_EVS_SendEvent(GPS_KALMAN_MSGID_ERR_EID,
                    CFE_EVS_EventType_ERROR,
                    "GPS_KALMAN - Recvd invalid SCH msgId (0x%08X)", CFE_SB_MsgIdToValue(MsgId));
        }
    }
    else if (iStatus == CFE_SB_NO_MESSAGE || iStatus == CFE_SB_TIME_OUT)
    {
        /* If there's no incoming message, you can do something here, or nothing */
    }
    else
    {
        /* This is an example of exiting on an error.
        ** Note that a SB read error is not always going to result in an app quitting.
        */
        CFE_EVS_SendEvent(GPS_KALMAN_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
            "GPS_KALMAN: SB pipe read error (0x%08X), app will exit", iStatus);

        g_GPS_KALMAN_AppData.uiRunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    return (iStatus);
}

bool GPS_KALMAN_IsFixOk(GpsInfoMsg_t *infoMsg)
{
    bool fixOk = false;

    if (infoMsg != NULL)
    {
        /* fix = Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) */
        fixOk = (infoMsg->gpsInfo.fix >= GPS_FIX_2D)
            /* sig = GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
            && (infoMsg->gpsInfo.sig >= GPS_SIG_FIX)
            /* 99.99 is used for undetermined/null */
            && (g_GPS_KALMAN_AppData.InData.gpsDOP < 99.99);
    }

    return fixOk;
}

/*=====================================================================================
 ** Name: GPS_KALMAN_ProcessNewData
 **
 ** Purpose: To process incoming data subscribed by GPS_KALMAN application
 **
 ** Arguments:
 **    None
 **
 ** Returns:
 **    None
 **
 ** Routines Called:
 **    CFE_SB_ReceiveBuffer
 **    CFE_MSG_GetMsgId
 **    CFE_EVS_SendEvent
 **
 ** Called By:
 **    GPS_KALMAN_RcvMsg
 **
 ** Global Inputs/Reads:
 **    None
 **
 ** Global Outputs/Writes:
 **    None
 **
 ** Limitations, Assumptions, External Events, and Notes:
 **    1. List assumptions that are made that apply to this function.
 **    2. List the external source(s) and event(s) that can cause this function to execute.
 **    3. List known limitations that apply to this function.
 **    4. If there are no assumptions, external events, or notes then enter NONE.
 **       Do not omit the section.
 **
 ** Algorithm:
 **    Psuedo-code or description of basic algorithm
 **
 ** Author(s):  Jacob Killelea
 **
 ** History:  Date Written  2019-06-28
 **           Unit Tested   yyyy-mm-dd
 **=====================================================================================*/
void GPS_KALMAN_ProcessNewData()
{
    int iStatus = CFE_SUCCESS;
    CFE_SB_Buffer_t *TlmMsgPtr = NULL;
    CFE_SB_MsgId_t  TlmMsgId;
    bool newFilterDataRecieved = false;

    /* Process telemetry messages till the pipe is empty */
    while (1)
    {
        iStatus = CFE_SB_ReceiveBuffer(&TlmMsgPtr, g_GPS_KALMAN_AppData.TlmPipeId, CFE_SB_POLL);
        if (iStatus == CFE_SUCCESS)
        {
            CFE_MSG_GetMsgId(&TlmMsgPtr->Msg, &TlmMsgId);
            switch (CFE_SB_MsgIdToValue(TlmMsgId))
            {
            case GPS_READER_GPS_INFO_MSG:
                /* CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_INFORMATION, "GPS_INFO messgage"); */
                newFilterDataRecieved = true;
                GpsInfoMsg_t *infoMsg = (GpsInfoMsg_t *) TlmMsgPtr;

                /* Lat and Lon are +/- in decimal format */
                g_GPS_KALMAN_AppData.InData.gpsLat  = decimal_minutes2decimal_decimal(infoMsg->gpsInfo.lat);
                g_GPS_KALMAN_AppData.InData.gpsLon  = decimal_minutes2decimal_decimal(infoMsg->gpsInfo.lon);
                /* kph */
                g_GPS_KALMAN_AppData.InData.gpsVel  = infoMsg->gpsInfo.speed;
                /* degrees true */
                g_GPS_KALMAN_AppData.InData.gpsHdg = infoMsg->gpsInfo.direction;
                g_GPS_KALMAN_AppData.InData.gpsDOP = infoMsg->gpsInfo.HDOP; /* Horizontal Dilution Of Precision */

                /* Determine whether GPS fix is good based on reported signals and PDOP */
                g_GPS_KALMAN_AppData.InData.gpsFixOk = GPS_KALMAN_IsFixOk(infoMsg);


                /* TODO: replace with actual filtering */
                g_GPS_KALMAN_AppData.OutData.filterHdg = infoMsg->gpsInfo.direction;
                break;

            default:
                CFE_EVS_SendEvent(GPS_KALMAN_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "GPS_KALMAN - Recvd invalid TLM msgId (0x%08X)", CFE_SB_MsgIdToValue(TlmMsgId));
                break;
            }
        }
        else if (iStatus == CFE_SB_NO_MESSAGE)
        {
            break;
        }
        else
        {
            CFE_EVS_SendEvent(GPS_KALMAN_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                  "GPS_KALMAN: CMD pipe read error (0x%08X)", iStatus);
            g_GPS_KALMAN_AppData.uiRunStatus = CFE_ES_RunStatus_APP_ERROR;
            break;
        }
    }

    if (newFilterDataRecieved)
    {
        if (!g_GPS_KALMAN_AppData.InData.gpsFixOk)
        {
            CFE_EVS_SendEvent(GPS_KALMAN_ERR_EID, CFE_EVS_EventType_ERROR, "GPS data not good");
        }
        else
        {
            OS_printf("[GPS_KALMAN] Input Lat  %11.7f\n",
                    g_GPS_KALMAN_AppData.InData.gpsLat);
            OS_printf("[GPS_KALMAN] Input Lon  %11.7f\n",
                    g_GPS_KALMAN_AppData.InData.gpsLon);
            OS_printf("[GPS_KALMAN] Input Spd  %11.7f\n",
                    g_GPS_KALMAN_AppData.InData.gpsVel);
            OS_printf("[GPS_KALMAN] Input Hdg  %11.7f\n",
                    g_GPS_KALMAN_AppData.InData.gpsHdg);
            OS_printf("[GPS_KALMAN] Input PDOP %11.7f\n",
                    g_GPS_KALMAN_AppData.InData.gpsDOP);
        }
    }
}

/*=====================================================================================
** Name: GPS_KALMAN_ProcessNewCmds
**
** Purpose: To process incoming command messages for GPS_KALMAN application
**
** Arguments:
**    None
**
** Returns:
**    None
**
** Routines Called:
**    CFE_SB_ReceiveBuffer
**    CFE_MSG_GetMsgId
**    CFE_EVS_SendEvent
**    GPS_KALMAN_ProcessNewAppCmds
**    GPS_KALMAN_ReportHousekeeping
**
** Called By:
**    GPS_KALMAN_RcvMsg
**
** Global Inputs/Reads:
**    None
**
** Global Outputs/Writes:
**    None
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
void GPS_KALMAN_ProcessNewCmds()
{
    int iStatus = CFE_SUCCESS;
    CFE_SB_Buffer_t *CmdMsgBuffer;
    CFE_SB_MsgId_t  CmdMsgId;

    /* Process command messages till the pipe is empty */
    while (1)
    {
        iStatus = CFE_SB_ReceiveBuffer(&CmdMsgBuffer, g_GPS_KALMAN_AppData.CmdPipeId, CFE_SB_POLL);
        if(iStatus == CFE_SUCCESS)
        {
            CFE_MSG_GetMsgId(&CmdMsgBuffer->Msg, &CmdMsgId);
            switch (CFE_SB_MsgIdToValue(CmdMsgId))
            {
            case GPS_KALMAN_CMD_MID:
                GPS_KALMAN_ProcessNewAppCmds(&CmdMsgBuffer->Msg);
                break;

            case GPS_KALMAN_SEND_HK_MID:
                GPS_KALMAN_ReportHousekeeping();
                break;

            /* TODO:  Add code to process other subscribed commands here
            **
            ** Example:
            **     case CFE_TIME_DATA_CMD_MID:
            **         GPS_KALMAN_ProcessTimeDataCmd(CmdMsgPtr);
            **         break;
            */

            default:
                CFE_EVS_SendEvent(GPS_KALMAN_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "GPS_KALMAN - Recvd invalid CMD msgId (0x%08X)", CFE_SB_MsgIdToValue(CmdMsgId));
                break;
            }
        }
        else if (iStatus == CFE_SB_NO_MESSAGE)
        {
            break;
        }
        else
        {
            CFE_EVS_SendEvent(GPS_KALMAN_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                  "GPS_KALMAN: CMD pipe read error (0x%08X)", iStatus);
            g_GPS_KALMAN_AppData.uiRunStatus = CFE_EVS_EventType_ERROR;
            break;
        }
    }
}

/*=====================================================================================
** Name: GPS_KALMAN_ProcessNewAppCmds
**
** Purpose: To process command messages targeting GPS_KALMAN application
**
** Arguments:
**    CFE_SB_Msg_t*  MsgPtr - new command message pointer
**
** Returns:
**    None
**
** Routines Called:
**    CFE_MSG_GetFcnCode
**    CFE_EVS_SendEvent
**
** Called By:
**    GPS_KALMAN_ProcessNewCmds
**
** Global Inputs/Reads:
**    None
**
** Global Outputs/Writes:
**    g_GPS_KALMAN_AppData.HkTlm.usCmdCnt
**    g_GPS_KALMAN_AppData.HkTlm.usCmdErrCnt
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
void GPS_KALMAN_ProcessNewAppCmds(CFE_MSG_Message_t* MsgPtr)
{
    CFE_MSG_FcnCode_t cmdCode = 0;

    if (MsgPtr != NULL)
    {
        CFE_MSG_GetFcnCode(MsgPtr, &cmdCode);

        switch (cmdCode)
        {
        case GPS_KALMAN_NOOP_CC:
            g_GPS_KALMAN_AppData.HkTlm.usCmdCnt++;
            CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_EventType_INFORMATION, "GPS_KALMAN - Recvd NOOP cmd (%d)", cmdCode);
            break;

        case GPS_KALMAN_RESET_CC:
            GPS_KALMAN_InitData(); // zero all the filters and the input and output structs
            g_GPS_KALMAN_AppData.HkTlm.usCmdCnt = 0;
            g_GPS_KALMAN_AppData.HkTlm.usCmdErrCnt = 0;
            CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_EventType_INFORMATION, "GPS_KALMAN - Recvd RESET cmd (%d)", cmdCode);
            break;

        /* TODO:  Add code to process the rest of the GPS_KALMAN commands here */

        default:
            g_GPS_KALMAN_AppData.HkTlm.usCmdErrCnt++;
            CFE_EVS_SendEvent(GPS_KALMAN_MSGID_ERR_EID, CFE_EVS_EventType_INFORMATION, "GPS_KALMAN - Recvd invalid cmdId (%d)", cmdCode);
            break;
        }
    }
}

/*=====================================================================================
** Name: GPS_KALMAN_RunFilter
**
** Purpose: Run the Kalman Filter
**
** Arguments:
**    None
**
** Returns:
**    None
**
** Routines Called:
**    - GSL vector and matrix math
**
** Called By:
**    GPS_KALMAN_ProcessNewData
**
** Global Inputs/Reads:
**    - The kalman related vectors and matrices
**
** Global Outputs/Writes:
**    - The kalman related vectors and matrices
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Runs a kalman filter on latitude, longitude, and velocity
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-07-11
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
int32 GPS_KALMAN_RunFilter(void) {
    int32 status = CFE_SUCCESS;
    int signum;
    double measured_lat = g_GPS_KALMAN_AppData.InData.gpsLat;
    double measured_lon = g_GPS_KALMAN_AppData.InData.gpsLon;
    double measured_vel = g_GPS_KALMAN_AppData.InData.gpsVel;
    double measured_dop = g_GPS_KALMAN_AppData.InData.gpsDOP;

    /* TODO: calculate delta t, initalize all matrices */

    /* Initialize state vector with last filter results */
    gsl_vector_set(XHat, 0, g_GPS_KALMAN_AppData.OutData.filterLat);
    gsl_vector_set(XHat, 1, g_GPS_KALMAN_AppData.OutData.filterLon);
    gsl_vector_set(XHat, 2, g_GPS_KALMAN_AppData.OutData.filterVel);

    /* Predict the next state */
    /* DGEMV: y = alpha*op(A)*x + Beta*y */
    /* With CblasNoTrans, op(A) = A */
    /* x_k+1 = F_k * x_k */
    gsl_blas_dgemv(CblasNoTrans, 1.0, FMatrix, XHat, 0.0, XHatNext);

    /* Next covariance: P = F * P * F' + Q */
    /* DGEMM: C = alpha*opa(A)*opb(B) + beta*C */
    /* P = 1:(F * P) * F' + Q <- TODO: check matrix order of operations. Should (P * F') be first?*/
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, FMatrix, PMatrix, 0.0, TmpMatrix);
    /* P = 2:(1:(tmp) * F') + Q */
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, TmpMatrix, FMatrix, 0.0, PMatrix);
    /* P = 3:(2:(1:(tmp) * F') + Q) */
    gsl_matrix_add(PMatrix, QMatrix);

    /* If GPS data is available, run the update section of the kalman algorithm */
    if (g_GPS_KALMAN_AppData.InData.gpsFixOk)
    {
        /* MuExpected = H * XHatNext */
        gsl_blas_dgemv(CblasNoTrans, 1.0, HMatrix, XHatNext, 0.0, MuExpected);
        /* SigmaExpectMatrix = H * P * H' */
        /* SigmaExpectMatrix = 1:(H * P) * H' */
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, HMatrix, PMatrix,
                0.0, TmpMatrix);
        /* SigmaExpectMatrix = 2:(1:(tmp) * H') */
        gsl_blas_dgemm(CblasNoTrans, CblasTrans,   1.0, TmpMatrix, HMatrix,
                0.0, SigmaExpectMatrix);

        /* MuActual = Actual measurement */
        gsl_vector_set(MuActual, 0, measured_lat);
        gsl_vector_set(MuActual, 1, measured_lon);
        gsl_vector_set(MuActual, 2, measured_vel);

        /* SigmaActualMatrix has DOP for lat and lon currently, 0.1 for speed */
        gsl_matrix_set_identity(SigmaActualMatrix);
        gsl_matrix_scale(SigmaActualMatrix, fabs(measured_dop));
        gsl_matrix_set(SigmaActualMatrix, 2, 2, 0.1);

        /* K = SigmaExpectMatrix * (SigmaExpectMatrix + SigmaActualMatrix)^-1 */
        /* (1) K = SigmaExpectMatrix * (1:(SigmaExpectMatrix + SigmaActualMatrix))^-1 */
        gsl_matrix_memcpy(TmpMatrix, SigmaExpectMatrix); /* tmp <-  sigma0 */
        gsl_matrix_add(SigmaExpectMatrix, SigmaActualMatrix); /* sigma0 <- sigma0 + sigma1 */
        gsl_matrix_swap(TmpMatrix, SigmaExpectMatrix); /* sigma0 <-> tmp */

        /*  TmpMatrix = $1 = SigmaExpectMatrix + SigmaActualMatrix */
        /* (2) K = SigmaExpectMatrix * 2:($1^-1) */
        gsl_linalg_LU_decomp(TmpMatrix, GSLPermutation, &signum);
        gsl_linalg_LU_invert(TmpMatrix, GSLPermutation, TmpMatrix2);

        /* TmpMatrix2 = $2 */
        /* (3) K = 3:(SigmaExpectMatrix * $2) */
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, SigmaExpectMatrix, TmpMatrix2,
                0.0, KMatrix);

        /* state_next = state_next + K * (mu1 - mu0) */
        /* (1) state_next = state_next + K * 1:(mu1 - mu0) */
        gsl_vector_sub(MuActual, MuExpected);
        /* mu1 = $1 */
        /* (2) state_next = 2:(K * 1:(mu1 - mu0) + state_next) */
        gsl_blas_dgemv(CblasNoTrans, 1.0, KMatrix, MuActual, 1.0, XHatNext);

        /* P = K * H * P - P */
        /* (1) P = K * 1:(H * P) - P */
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, HMatrix, PMatrix,
                0.0, TmpMatrix);
        /* TmpMatrix = $1 */
        /* (2) P = 2:(K * 1:(H * P) - P) */
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, KMatrix, TmpMatrix,
                1.0, PMatrix);
    }

    /* state <- state_next */
    gsl_vector_memcpy(XHat, XHatNext);

    g_GPS_KALMAN_AppData.OutData.filterLat = gsl_vector_get(XHatNext, 0);
    g_GPS_KALMAN_AppData.OutData.filterLon = gsl_vector_get(XHatNext, 1);
    g_GPS_KALMAN_AppData.OutData.filterVel = gsl_vector_get(XHatNext, 2);

    return status;
}

/*=====================================================================================
** Name: GPS_KALMAN_ReportHousekeeping
**
** Purpose: To send housekeeping message
**
** Arguments:
**    None
**
** Returns:
**    None
**
** Routines Called:
**    TBD
**
** Called By:
**    GPS_KALMAN_ProcessNewCmds
**
** Global Inputs/Reads:
**    None
**
** Global Outputs/Writes:
**    TBD
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  GSFC, Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
void GPS_KALMAN_ReportHousekeeping()
{
    /* TODO:  Add code to update housekeeping data, if needed, here.  */

    CFE_SB_TimeStampMsg((CFE_MSG_Message_t*) &g_GPS_KALMAN_AppData.HkTlm);
    CFE_SB_TransmitMsg((CFE_MSG_Message_t*) &g_GPS_KALMAN_AppData.HkTlm, true);
}

/*=====================================================================================
** Name: GPS_KALMAN_SendOutData
**
** Purpose: To publish 1-Wakeup cycle output data
**
** Arguments:
**    None
**
** Returns:
**    None
**
** Routines Called:
**    TBD
**
** Called By:
**    GPS_KALMAN_RcvMsg
**
** Global Inputs/Reads:
**    None
**
** Global Outputs/Writes:
**    TBD
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
void GPS_KALMAN_SendOutData()
{
    /* TODO:  Add code to update output data, if needed, here.  */

    CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_EventType_INFORMATION,
        "%10.7f %10.7f %10.7f +/- %10.7f %10.7f %10.7f",
        g_GPS_KALMAN_AppData.OutData.filterLat,
        g_GPS_KALMAN_AppData.OutData.filterLon,
        g_GPS_KALMAN_AppData.OutData.filterVel,
        gsl_matrix_get(PMatrix, 0, 0),
        gsl_matrix_get(PMatrix, 1, 1),
        gsl_matrix_get(PMatrix, 2, 2));

    CFE_SB_TimeStampMsg((CFE_MSG_Message_t*) &g_GPS_KALMAN_AppData.OutData);
    CFE_SB_TransmitMsg((CFE_MSG_Message_t*) &g_GPS_KALMAN_AppData.OutData, true);
}

/*=====================================================================================
** Name: GPS_KALMAN_VerifyCmdLength
**
** Purpose: To verify command length for a particular command message
**
** Arguments:
**    CFE_SB_Msg_t*  MsgPtr      - command message pointer
**    uint16         usExpLength - expected command length
**
** Returns:
**    boolean bResult - result of verification
**
** Routines Called:
**    TBD
**
** Called By:
**    GPS_KALMAN_ProcessNewCmds
**
** Global Inputs/Reads:
**    None
**
** Global Outputs/Writes:
**    TBD
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
bool GPS_KALMAN_VerifyCmdLength(CFE_MSG_Message_t* MsgPtr, uint16 usExpectedLen)
{
    bool verified = false;
    CFE_MSG_Size_t usMsgLen = 0;

    if (MsgPtr != NULL)
    {
        CFE_MSG_GetSize(MsgPtr, &usMsgLen);

        if (usExpectedLen != usMsgLen)
        {
            CFE_SB_MsgId_t MsgId;
            CFE_MSG_FcnCode_t usCmdCode = 0;
            CFE_MSG_GetFcnCode(MsgPtr, &usCmdCode);
            CFE_MSG_GetMsgId(MsgPtr, &MsgId);

            CFE_EVS_SendEvent(GPS_KALMAN_MSGLEN_ERR_EID, CFE_EVS_EventType_ERROR,
                              "GPS_KALMAN - Rcvd invalid msgLen: msgId=0x%08X, cmdCode=%d, "
                              "msgLen=%zu, expectedLen=%d",
                              CFE_SB_MsgIdToValue(MsgId), usCmdCode, usMsgLen, usExpectedLen);
            g_GPS_KALMAN_AppData.HkTlm.usCmdErrCnt++;
        }
        else
        {
        verified = true;
        }
    }

    return verified;
}

/*=====================================================================================
** Name: GPS_KALMAN_AppMain
**
** Purpose: To define GPS_KALMAN application's entry point and main process loop
**
** Arguments:
**    None
**
** Returns:
**    None
**
** Routines Called:
**    CFE_ES_RunLoop
**    CFE_ES_PerfLogEntry
**    CFE_ES_PerfLogExit
**    CFE_ES_ExitApp
**    CFE_ES_WaitForStartupSync
**    GPS_KALMAN_InitApp
**    GPS_KALMAN_RcvMsg
**
** Called By:
**    TBD
**
** Global Inputs/Reads:
**    TBD
**
** Global Outputs/Writes:
**    TBD
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to this function.
**    2. List the external source(s) and event(s) that can cause this function to execute.
**    3. List known limitations that apply to this function.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Algorithm:
**    Psuedo-code or description of basic algorithm
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-06-28
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
void GPS_KALMAN_AppMain()
{
    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(GPS_KALMAN_MAIN_TASK_PERF_ID);

    /* Perform application initializations */
    if (GPS_KALMAN_InitApp() != CFE_SUCCESS)
    {
        g_GPS_KALMAN_AppData.uiRunStatus = CFE_ES_RunStatus_APP_ERROR;
    }
    else
    {
        /* Do not perform performance monitoring on startup sync */
        CFE_ES_PerfLogExit(GPS_KALMAN_MAIN_TASK_PERF_ID);
        CFE_ES_WaitForStartupSync(GPS_KALMAN_TIMEOUT_MSEC);
        CFE_ES_PerfLogEntry(GPS_KALMAN_MAIN_TASK_PERF_ID);
    }

    /* Application main loop */
    while (CFE_ES_RunLoop(&g_GPS_KALMAN_AppData.uiRunStatus) == true)
    {
        GPS_KALMAN_RcvMsg(1000);
    }

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(GPS_KALMAN_MAIN_TASK_PERF_ID);

    /* Exit the application */
    CFE_ES_ExitApp(g_GPS_KALMAN_AppData.uiRunStatus);
}

/*=======================================================================================
** End of file gps_kalman_app.c
**=====================================================================================*/

