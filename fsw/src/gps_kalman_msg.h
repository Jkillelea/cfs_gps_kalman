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
**   2019-09-02 | Jacob Killelea | Build #: Move GPS_KALMAN_OutData_t to this file
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
#include "cfe.h"
#include "common_types.h"


/*
** Local Defines
*/

/*
** GPS_KALMAN command codes
*/
#define GPS_KALMAN_NOOP_CC  0
#define GPS_KALMAN_RESET_CC 1

/*
** Local Structure Declarations
*/
typedef struct
{
    uint8  TlmHeader[CFE_SB_TLM_HDR_SIZE];
    uint8  usCmdCnt;
    uint8  usCmdErrCnt;

    /* TODO:  Add declarations for additional housekeeping data here */

} GPS_KALMAN_HkTlm_t;

/* Filter output data */
typedef struct
{
    uint8   ucTlmHeader[CFE_SB_TLM_HDR_SIZE];
    uint32  uiCounter;
    double  filterLat; /* Kalman Filter Lattidue */
    double  filterLon; /* Kalman Filter Longitude */
    double  filterVel; /* Kalman Filter Velocity */
    double  filterHdg; /* Kalman Filter Heading (true) TODO: actually filter? */
} GPS_KALMAN_OutData_t;

#endif /* _GPS_KALMAN_MSG_H_ */

/*=======================================================================================
** End of file gps_kalman_msg.h
**=====================================================================================*/

