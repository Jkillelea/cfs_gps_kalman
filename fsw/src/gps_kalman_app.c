/*=======================================================================================
** File Name:  gps_kalman_app.c
**
** Title:  Function Definitions for GPS_KALMAN Application
**
** $Author:    Jacob Killelea
** $Revision: 1.1 $
** $Date:      2019-06-28
**
** Purpose:  This source file contains all necessary function definitions to run GPS_KALMAN
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

#include "cfe.h"

#include "gps_kalman_platform_cfg.h"
#include "gps_kalman_mission_cfg.h"
#include "gps_kalman_app.h"
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
gsl_vector *x_hat       = NULL; // kalman state vector
gsl_vector *x_hat_next  = NULL; // kalman state vector
gsl_matrix *F_mat       = NULL; // kalman system matrix
gsl_matrix *Tmp_mat     = NULL; // Temporary matrix
gsl_matrix *Tmp_mat2    = NULL; // Temporary matrix
gsl_matrix *P_mat       = NULL; // kalman state covariance matrix (identity for now)
gsl_matrix *Q_mat       = NULL; // kalman state covariance uncertainty matrix (zero for now)
gsl_matrix *H_mat       = NULL; // kalman measurement matrix (identity for now)

gsl_vector *mu_expect    = NULL; // expected measurement
gsl_matrix *Sigma_expect = NULL; // expected covariance

gsl_vector *mu_actual    = NULL; // actual measurement
gsl_matrix *Sigma_actual = NULL; // actual covariance

gsl_matrix *K_mat = NULL; // kalman gain

gsl_permutation *permut = NULL; // used for inverting matrices

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
                               GPS_KALMAN_EVT_CNT, CFE_EVS_BINARY_FILTER);
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
    strncpy(g_GPS_KALMAN_AppData.cSchPipeName, "GPS_KALMAN_SCH_PIPE", OS_MAX_API_NAME-1);

    /* Subscribe to Wakeup messages */
    iStatus = CFE_SB_CreatePipe(&g_GPS_KALMAN_AppData.SchPipeId,
                                 g_GPS_KALMAN_AppData.usSchPipeDepth,
                                 g_GPS_KALMAN_AppData.cSchPipeName);
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = CFE_SB_SubscribeEx(GPS_KALMAN_WAKEUP_MID, g_GPS_KALMAN_AppData.SchPipeId, CFE_SB_Default_Qos, 1);

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
    strncpy(g_GPS_KALMAN_AppData.cCmdPipeName, "GPS_KALMAN_CMD_PIPE", OS_MAX_API_NAME-1);

    /* Subscribe to command messages */
    iStatus = CFE_SB_CreatePipe(&g_GPS_KALMAN_AppData.CmdPipeId,
                                 g_GPS_KALMAN_AppData.usCmdPipeDepth,
                                 g_GPS_KALMAN_AppData.cCmdPipeName);
    if (iStatus == CFE_SUCCESS)
    {
        /* Subscribe to command messages */
        iStatus = CFE_SB_Subscribe(GPS_KALMAN_CMD_MID, g_GPS_KALMAN_AppData.CmdPipeId);

        if (iStatus != CFE_SUCCESS)
        {
            CFE_ES_WriteToSysLog("GPS_KALMAN - CMD Pipe failed to subscribe to GPS_KALMAN_CMD_MID. (0x%08X)\n", iStatus);
            goto GPS_KALMAN_InitPipe_Exit_Tag;
        }

        iStatus  = CFE_SB_Subscribe(GPS_KALMAN_SEND_HK_MID,   g_GPS_KALMAN_AppData.CmdPipeId);

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
    strncpy(g_GPS_KALMAN_AppData.cTlmPipeName, "GPS_KALMAN_TLM_PIPE", OS_MAX_API_NAME-1);

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
        CFE_SB_Subscribe(GPS_READER_GPS_INFO_MSG,  g_GPS_KALMAN_AppData.TlmPipeId);
        CFE_SB_Subscribe(GPS_READER_GPS_GPGGA_MSG, g_GPS_KALMAN_AppData.TlmPipeId);
        CFE_SB_Subscribe(GPS_READER_GPS_GPGSA_MSG, g_GPS_KALMAN_AppData.TlmPipeId);
        CFE_SB_Subscribe(GPS_READER_GPS_GPGSV_MSG, g_GPS_KALMAN_AppData.TlmPipeId);
        CFE_SB_Subscribe(GPS_READER_GPS_GPRMC_MSG, g_GPS_KALMAN_AppData.TlmPipeId);
        CFE_SB_Subscribe(GPS_READER_GPS_GPVTG_MSG, g_GPS_KALMAN_AppData.TlmPipeId);

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
    CFE_SB_InitMsg(&g_GPS_KALMAN_AppData.OutData, 
            GPS_KALMAN_OUT_DATA_MID, 
            sizeof(g_GPS_KALMAN_AppData.OutData), 
            TRUE);

    /* Init housekeeping packet */
    memset((void*)&g_GPS_KALMAN_AppData.HkTlm, 0x00, 
            sizeof(g_GPS_KALMAN_AppData.HkTlm));
    CFE_SB_InitMsg(&g_GPS_KALMAN_AppData.HkTlm, 
            GPS_KALMAN_HK_TLM_MID, 
            sizeof(g_GPS_KALMAN_AppData.HkTlm), TRUE);

    // initalize all the kalman filter elements
    x_hat         =  gsl_vector_calloc(GPS_KALMAN_FILTER_LEN);
    x_hat_next    =  gsl_vector_calloc(GPS_KALMAN_FILTER_LEN);
    mu_expect     =  gsl_vector_calloc(GPS_KALMAN_FILTER_LEN);
    mu_actual     =  gsl_vector_calloc(GPS_KALMAN_FILTER_LEN);
    F_mat         =  gsl_matrix_calloc(GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    Tmp_mat       =  gsl_matrix_calloc(GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    Tmp_mat2      =  gsl_matrix_calloc(GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    P_mat         =  gsl_matrix_calloc(GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    Q_mat         =  gsl_matrix_calloc(GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    H_mat         =  gsl_matrix_calloc(GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    Sigma_expect  =  gsl_matrix_calloc(GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    Sigma_actual  =  gsl_matrix_calloc(GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    K_mat         =  gsl_matrix_calloc(GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    permut        =  gsl_permutation_calloc(GPS_KALMAN_FILTER_LEN);

    gsl_matrix_set_identity(F_mat);
    gsl_matrix_set_zero(Tmp_mat);
    gsl_matrix_set_zero(Tmp_mat2);
    gsl_matrix_set_identity(P_mat);
    gsl_matrix_set_zero(Q_mat);
    gsl_matrix_set_identity(H_mat);
    gsl_matrix_set_zero(Sigma_expect);
    gsl_matrix_set_zero(Sigma_actual);
    gsl_matrix_set_zero(K_mat);

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
**    CFE_ES_RegisterApp
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

    g_GPS_KALMAN_AppData.uiRunStatus = CFE_ES_APP_RUN;

    iStatus = CFE_ES_RegisterApp();
    if (iStatus != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("GPS_KALMAN - Failed to register the app (0x%08X)\n", iStatus);
        goto GPS_KALMAN_InitApp_Exit_Tag;
    }

    if ((GPS_KALMAN_InitEvent() != CFE_SUCCESS) || 
        (GPS_KALMAN_InitPipe() != CFE_SUCCESS) || 
        (GPS_KALMAN_InitData() != CFE_SUCCESS))
    {
        iStatus = -1;
        goto GPS_KALMAN_InitApp_Exit_Tag;
    }

    /* Install the cleanup callback */
    OS_TaskInstallDeleteHandler((void*) &GPS_KALMAN_CleanupCallback);

GPS_KALMAN_InitApp_Exit_Tag:
    if (iStatus == CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GPS_KALMAN_INIT_INF_EID, CFE_EVS_INFORMATION,
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
    // initalize all the kalman filter elements
    gsl_vector_free(x_hat);
    gsl_vector_free(x_hat_next);
    gsl_vector_free(mu_expect);
    gsl_vector_free(mu_actual);
    gsl_matrix_free(F_mat);
    gsl_matrix_free(Tmp_mat);
    gsl_matrix_free(Tmp_mat2);
    gsl_matrix_free(P_mat);
    gsl_matrix_free(Q_mat);
    gsl_matrix_free(H_mat);
    gsl_matrix_free(Sigma_expect);
    gsl_matrix_free(Sigma_actual);
    gsl_matrix_free(K_mat);
    gsl_permutation_free(permut);
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
**    CFE_SB_RcvMsg
**    CFE_SB_GetMsgId
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
    int32           iStatus = CFE_SUCCESS;
    CFE_SB_Msg_t*   MsgPtr = NULL;
    CFE_SB_MsgId_t  MsgId;

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(GPS_KALMAN_MAIN_TASK_PERF_ID);

    /* Wait for WakeUp messages from scheduler */
    iStatus = CFE_SB_RcvMsg(&MsgPtr, g_GPS_KALMAN_AppData.SchPipeId, iBlocking);

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(GPS_KALMAN_MAIN_TASK_PERF_ID);

    if (iStatus == CFE_SUCCESS)
    {
        MsgId = CFE_SB_GetMsgId(MsgPtr);
        switch (MsgId) 
        {
            case GPS_KALMAN_WAKEUP_MID:
                GPS_KALMAN_ProcessNewCmds();
                GPS_KALMAN_ProcessNewData();

                /* TODO:  Add more code here to handle other things when app wakes up */

                /* The last thing to do at the end of this Wakeup cycle should be to
                   automatically publish new output. */
                GPS_KALMAN_SendOutData();
                break;

            default:
                CFE_EVS_SendEvent(GPS_KALMAN_MSGID_ERR_EID, CFE_EVS_ERROR,
                                  "GPS_KALMAN - Recvd invalid SCH msgId (0x%08X)", MsgId);
        }
    }
    else if (iStatus == CFE_SB_NO_MESSAGE)
    {
        /* If there's no incoming message, you can do something here, or nothing */
    }
    else
    {
        /* This is an example of exiting on an error.
        ** Note that a SB read error is not always going to result in an app quitting.
        */
        CFE_EVS_SendEvent(GPS_KALMAN_PIPE_ERR_EID, CFE_EVS_ERROR,
			  "GPS_KALMAN: SB pipe read error (0x%08X), app will exit", iStatus);
        g_GPS_KALMAN_AppData.uiRunStatus= CFE_ES_APP_ERROR;
    }

    return (iStatus);
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
**    CFE_SB_RcvMsg
**    CFE_SB_GetMsgId
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
    CFE_SB_Msg_t*   TlmMsgPtr=NULL;
    CFE_SB_MsgId_t  TlmMsgId;
    boolean newFilterDataRecieved = FALSE;

    /* Process telemetry messages till the pipe is empty */
    while (1)
    {
        iStatus = CFE_SB_RcvMsg(&TlmMsgPtr, g_GPS_KALMAN_AppData.TlmPipeId, CFE_SB_POLL);
        if (iStatus == CFE_SUCCESS)
        {
            TlmMsgId = CFE_SB_GetMsgId(TlmMsgPtr);
            switch (TlmMsgId)
            {
                case GPS_READER_GPS_INFO_MSG:
                    CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_INFORMATION, "GPS_INFO messgage");
                    GpsInfoMsg_t *infoMsg = (GpsInfoMsg_t *) TlmMsgPtr;
                    g_GPS_KALMAN_AppData.InData.gpsLat = infoMsg->gpsInfo.lat;
                    g_GPS_KALMAN_AppData.InData.gpsLon = infoMsg->gpsInfo.lon;
                    newFilterDataRecieved = TRUE;
                    /* Speed and Heading not updated by this message */
                    break;

                case GPS_READER_GPS_GPGGA_MSG:
                    CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_INFORMATION, "GPS_GPGGA messgage");
                    GpsGpggaMsg_t *gpggaMsg = (GpsGpggaMsg_t *) TlmMsgPtr;
                    g_GPS_KALMAN_AppData.InData.gpsLat = gpggaMsg->gpsGpgga.lat;
                    g_GPS_KALMAN_AppData.InData.gpsLon = gpggaMsg->gpsGpgga.lon;
                    newFilterDataRecieved = TRUE;
                    /* Speed and Heading not updated by this message */
                    break;
                    
                case GPS_READER_GPS_GPGSA_MSG:
                    CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_INFORMATION, "GPS_GPGSA messgage");
                    /* no position information */
                    break;

                case GPS_READER_GPS_GPGSV_MSG:
                    CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_INFORMATION, "GPS_GPGSV messgage");
                    /* no position information */
                    break;

                case GPS_READER_GPS_GPRMC_MSG:
                    CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_INFORMATION, "GPS_GPRMC messgage");
                    GpsGprmcMsg_t *gprmcMsg = (GpsGprmcMsg_t *) TlmMsgPtr;
                    g_GPS_KALMAN_AppData.InData.gpsLat = gprmcMsg->gpsGprmc.lat;
                    g_GPS_KALMAN_AppData.InData.gpsLon = gprmcMsg->gpsGprmc.lon;
                    g_GPS_KALMAN_AppData.InData.gpsVel = gprmcMsg->gpsGprmc.speed;
                    g_GPS_KALMAN_AppData.InData.gpsHdg = gprmcMsg->gpsGprmc.declination;
                    newFilterDataRecieved = TRUE;
                    break;

                case GPS_READER_GPS_GPVTG_MSG:
                    CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_INFORMATION, "GPS_GPVTG messgage");
                    /* no position information */
                    break;

                default:
                    CFE_EVS_SendEvent(GPS_KALMAN_MSGID_ERR_EID, CFE_EVS_ERROR,
                                      "GPS_KALMAN - Recvd invalid TLM msgId (0x%08X)", TlmMsgId);
                    break;
            }
        }
        else if (iStatus == CFE_SB_NO_MESSAGE)
        {
            break;
        }
        else
        {
            CFE_EVS_SendEvent(GPS_KALMAN_PIPE_ERR_EID, CFE_EVS_ERROR,
                  "GPS_KALMAN: CMD pipe read error (0x%08X)", iStatus);
            g_GPS_KALMAN_AppData.uiRunStatus = CFE_ES_APP_ERROR;
            break;
        }
    }

    if (newFilterDataRecieved == TRUE) {
        GPS_KALMAN_RunFilter();
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
**    CFE_SB_RcvMsg
**    CFE_SB_GetMsgId
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
    CFE_SB_Msg_t*   CmdMsgPtr=NULL;
    CFE_SB_MsgId_t  CmdMsgId;

    /* Process command messages till the pipe is empty */
    while (1)
    {
        iStatus = CFE_SB_RcvMsg(&CmdMsgPtr, g_GPS_KALMAN_AppData.CmdPipeId, CFE_SB_POLL);
        if(iStatus == CFE_SUCCESS)
        {
            CmdMsgId = CFE_SB_GetMsgId(CmdMsgPtr);
            switch (CmdMsgId)
            {
                case GPS_KALMAN_CMD_MID:
                    GPS_KALMAN_ProcessNewAppCmds(CmdMsgPtr);
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
                    CFE_EVS_SendEvent(GPS_KALMAN_MSGID_ERR_EID, CFE_EVS_ERROR,
                                      "GPS_KALMAN - Recvd invalid CMD msgId (0x%08X)", CmdMsgId);
                    break;
            }
        }
        else if (iStatus == CFE_SB_NO_MESSAGE)
        {
            break;
        }
        else
        {
            CFE_EVS_SendEvent(GPS_KALMAN_PIPE_ERR_EID, CFE_EVS_ERROR,
                  "GPS_KALMAN: CMD pipe read error (0x%08X)", iStatus);
            g_GPS_KALMAN_AppData.uiRunStatus = CFE_ES_APP_ERROR;
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
**    CFE_SB_GetCmdCode
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
void GPS_KALMAN_ProcessNewAppCmds(CFE_SB_Msg_t* MsgPtr)
{
    uint32 uiCmdCode = 0;

    if (MsgPtr != NULL)
    {
        uiCmdCode = CFE_SB_GetCmdCode(MsgPtr);
        switch (uiCmdCode)
        {
            case GPS_KALMAN_NOOP_CC:
                g_GPS_KALMAN_AppData.HkTlm.usCmdCnt++;
                CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_INFORMATION,
                                  "GPS_KALMAN - Recvd NOOP cmd (%d)", uiCmdCode);
                break;

            case GPS_KALMAN_RESET_CC:
                g_GPS_KALMAN_AppData.HkTlm.usCmdCnt = 0;
                g_GPS_KALMAN_AppData.HkTlm.usCmdErrCnt = 0;
                CFE_EVS_SendEvent(GPS_KALMAN_CMD_INF_EID, CFE_EVS_INFORMATION,
                                  "GPS_KALMAN - Recvd RESET cmd (%d)", uiCmdCode);
                break;

            /* TODO:  Add code to process the rest of the GPS_KALMAN commands here */

            default:
                g_GPS_KALMAN_AppData.HkTlm.usCmdErrCnt++;
                CFE_EVS_SendEvent(GPS_KALMAN_MSGID_ERR_EID, CFE_EVS_ERROR,
                                  "GPS_KALMAN - Recvd invalid cmdId (%d)", uiCmdCode);
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
**    TBD
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
    double measured_lat = g_GPS_KALMAN_AppData.InData.gpsLat;
    double measured_lon = g_GPS_KALMAN_AppData.InData.gpsLon;
    double measured_vel = g_GPS_KALMAN_AppData.InData.gpsVel;

    // TODO: calculate delta t, initalize all matrices

    // Initialize state vector with last filter results
    gsl_vector_set(x_hat, 0, g_GPS_KALMAN_AppData.OutData.filterLat);
    gsl_vector_set(x_hat, 1, g_GPS_KALMAN_AppData.OutData.filterLon);
    gsl_vector_set(x_hat, 2, g_GPS_KALMAN_AppData.OutData.filterVel);

    // Predict the next state
    // DGEMV: y = alpha*op(A)*x + Beta*y
    // With CblasNoTrans, op(A) = A
    // x_k+1 = F_k * x_k
    gsl_blas_dgemv(CblasNoTrans, 1.0, F_mat, x_hat, 0.0, x_hat_next);

    // Next covariance: P = F * P * F' + Q
    // DGEMM: C = alpha*opa(A)*opb(B) + beta*C
    // P = 1:(F * P) * F' + Q
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,   F_mat, P_mat, 0.0, Tmp_mat);
    // P = 2:(1:(tmp) * F') + Q
    gsl_blas_dgemm(CblasNoTrans,   CblasTrans, 1.0, Tmp_mat, F_mat, 0.0,   P_mat);
    // P = 3:(2:(1:(tmp) * F') + Q)
    gsl_matrix_add(P_mat, Q_mat);

    // mu_expect = H * x_hat_next
    gsl_blas_dgemv(CblasNoTrans, 1.0, H_mat, x_hat, 0.0, mu_expect);
    // Sigma_expect = H * P * H'
    // Sigma_expect = 1:(H * P) * H'
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, H_mat,   P_mat, 0.0,      Tmp_mat);
    // Sigma_expect = 2:(1:(tmp) * H')
    gsl_blas_dgemm(CblasNoTrans, CblasTrans,   1.0, Tmp_mat, H_mat, 0.0, Sigma_expect);

    // mu_actual = Actual measurement
    gsl_vector_set(mu_actual, 0, measured_lat);
    gsl_vector_set(mu_actual, 1, measured_lon);
    gsl_vector_set(mu_actual, 2, measured_vel);

    gsl_matrix_memcpy(Sigma_actual, P_mat);


    // K = Sigma_expect * (Sigma_expect + Sigma_actual)^-1
    // (1) K = Sigma_expect * (1:(Sigma_expect + Sigma_actual))^-1
    gsl_matrix_memcpy(Tmp_mat,   Sigma_expect); // tmp <-  sigma0
    gsl_matrix_add(Sigma_expect, Sigma_actual); // sigma0 <- sigma0 + sigma1
    gsl_matrix_swap(Tmp_mat,     Sigma_expect); // sigma0 <-> tmp

    //  Tmp_mat = $1 = Sigma_expect + Sigma_actual
    // (2) K = Sigma_expect * 2:($1^-1)
    int signum;
    gsl_linalg_LU_decomp(Tmp_mat, permut, &signum);
    gsl_linalg_LU_invert(Tmp_mat, permut, Tmp_mat2);

    // Tmp_mat2 = $2
    // (3) K = 3:(Sigma_expect * $2)
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Sigma_expect, Tmp_mat2, 0.0, K_mat);

    g_GPS_KALMAN_AppData.OutData.filterLat = gsl_vector_get(x_hat_next, 0);
    g_GPS_KALMAN_AppData.OutData.filterLon = gsl_vector_get(x_hat_next, 1);
    g_GPS_KALMAN_AppData.OutData.filterVel = gsl_vector_get(x_hat_next, 2);

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

    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&g_GPS_KALMAN_AppData.HkTlm);
    CFE_SB_SendMsg((CFE_SB_Msg_t*)&g_GPS_KALMAN_AppData.HkTlm);
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

    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*) &g_GPS_KALMAN_AppData.OutData);
    CFE_SB_SendMsg((CFE_SB_Msg_t*) &g_GPS_KALMAN_AppData.OutData);
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
boolean GPS_KALMAN_VerifyCmdLength(CFE_SB_Msg_t* MsgPtr,
                           uint16 usExpectedLen) {
    boolean bResult = FALSE;
    uint16  usMsgLen = 0;

    if (MsgPtr != NULL) {
        usMsgLen = CFE_SB_GetTotalMsgLength(MsgPtr);

        if (usExpectedLen != usMsgLen) {
            CFE_SB_MsgId_t MsgId = CFE_SB_GetMsgId(MsgPtr);
            uint16 usCmdCode = CFE_SB_GetCmdCode(MsgPtr);

            CFE_EVS_SendEvent(GPS_KALMAN_MSGLEN_ERR_EID, CFE_EVS_ERROR,
                              "GPS_KALMAN - Rcvd invalid msgLen: msgId=0x%08X, cmdCode=%d, "
                              "msgLen=%d, expectedLen=%d",
                              MsgId, usCmdCode, usMsgLen, usExpectedLen);
            g_GPS_KALMAN_AppData.HkTlm.usCmdErrCnt++;
        }
    }

    return (bResult);
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
**    CFE_ES_RegisterApp
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
    /* Register the application with Executive Services */
    CFE_ES_RegisterApp();

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(GPS_KALMAN_MAIN_TASK_PERF_ID);

    /* Perform application initializations */
    if (GPS_KALMAN_InitApp() != CFE_SUCCESS) {
        g_GPS_KALMAN_AppData.uiRunStatus = CFE_ES_APP_ERROR;
    } else {
        /* Do not perform performance monitoring on startup sync */
        CFE_ES_PerfLogExit(GPS_KALMAN_MAIN_TASK_PERF_ID);
        CFE_ES_WaitForStartupSync(GPS_KALMAN_TIMEOUT_MSEC);
        CFE_ES_PerfLogEntry(GPS_KALMAN_MAIN_TASK_PERF_ID);
    }

    /* Application main loop */
    while (CFE_ES_RunLoop(&g_GPS_KALMAN_AppData.uiRunStatus) == TRUE) {
        GPS_KALMAN_RcvMsg(CFE_SB_PEND_FOREVER);
    }

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(GPS_KALMAN_MAIN_TASK_PERF_ID);

    /* Exit the application */
    CFE_ES_ExitApp(g_GPS_KALMAN_AppData.uiRunStatus);
} 
    
/*=======================================================================================
** End of file gps_kalman_app.c
**=====================================================================================*/
    
