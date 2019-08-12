/*=======================================================================================
** File Name:  gps_kalman_private_ids.h
**
** Title:  ID Header File for GPS_KALMAN Application
**
** $Author:    Jacob Killelea
** $Revision: 1.1 $
** $Date:      2019-06-28
**
** Purpose:  This header file contains declarations and definitions of GPS_KALMAN's private IDs.
**
** Modification History:
**   Date | Author | Description
**   ---------------------------
**   2019-06-28 | Jacob Killelea | Build #: Code Started
**
**=====================================================================================*/
    
#ifndef _GPS_KALMAN_PRIVATE_IDS_H_
#define _GPS_KALMAN_PRIVATE_IDS_H_

/*
** Pragmas
*/

/*
** Include Files
*/

/*
** Local Defines
*/

/* Event IDs */
#define GPS_KALMAN_RESERVED_EID  0

#define GPS_KALMAN_INF_EID        1
#define GPS_KALMAN_INIT_INF_EID   2
#define GPS_KALMAN_ILOAD_INF_EID  3
#define GPS_KALMAN_CDS_INF_EID    4
#define GPS_KALMAN_CMD_INF_EID    5

#define GPS_KALMAN_ERR_EID         51
#define GPS_KALMAN_INIT_ERR_EID    52
#define GPS_KALMAN_ILOAD_ERR_EID   53
#define GPS_KALMAN_CDS_ERR_EID     54
#define GPS_KALMAN_CMD_ERR_EID     55
#define GPS_KALMAN_PIPE_ERR_EID    56
#define GPS_KALMAN_MSGID_ERR_EID   57
#define GPS_KALMAN_MSGLEN_ERR_EID  58

#define GPS_KALMAN_EVT_CNT  14

/*
** Local Structure Declarations
*/

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

#endif /* _GPS_KALMAN_PRIVATE_IDS_H_ */

/*=======================================================================================
** End of file gps_kalman_private_ids.h
**=====================================================================================*/
    