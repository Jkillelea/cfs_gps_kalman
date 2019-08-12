/*=======================================================================================
** File Name:  gps_kalman_msg.h
**
** Title:  Message Definition Header File for GPS_KALMAN Application
**
** $Author:    Jacob Killelea
** $Revision: 1.1 $
** $Date:      2019-06-28
**
** Purpose:  To define GPS_KALMAN's command and telemetry message defintions 
**
** Modification History:
**   Date | Author | Description
**   ---------------------------
**   2019-06-28 | Jacob Killelea | Build #: Code Started
**
**=====================================================================================*/
    
#ifndef _GPS_KALMAN_MSG_H_
#define _GPS_KALMAN_MSG_H_

/*
** Pragmas
*/

/*
** Include Files
*/



/*
** Local Defines
*/

/*
** GPS_KALMAN command codes
*/
#define GPS_KALMAN_NOOP_CC                 0
#define GPS_KALMAN_RESET_CC                1

/*
** Local Structure Declarations
*/
typedef struct
{
    uint8              TlmHeader[CFE_SB_TLM_HDR_SIZE];
    uint8              usCmdCnt;
    uint8              usCmdErrCnt;

    /* TODO:  Add declarations for additional housekeeping data here */

} GPS_KALMAN_HkTlm_t;


#endif /* _GPS_KALMAN_MSG_H_ */

/*=======================================================================================
** End of file gps_kalman_msg.h
**=====================================================================================*/
    