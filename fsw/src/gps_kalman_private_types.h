/*=======================================================================================
** File Name:  gps_kalman_private_types.h
**
** Title:  Type Header File for GPS_KALMAN Application
**
** $Author:    Jacob Killelea
** $Revision: 1.1 $
** $Date:      2019-06-28
**
** Purpose:  This header file contains declarations and definitions of all GPS_KALMAN's private
**           data structures and data types.
**
** Modification History:
**   Date | Author | Description
**   ---------------------------
**   2019-06-28 | Jacob Killelea | Build #: Code Started
**
**=====================================================================================*/
    
#ifndef _GPS_KALMAN_PRIVATE_TYPES_H_
#define _GPS_KALMAN_PRIVATE_TYPES_H_

/*
** Pragmas
*/

/*
** Include Files
*/
#include "cfe.h"

/*
** Local Defines
*/

/*
** Local Structure Declarations
*/

typedef struct
{
    CFE_MSG_CommandHeader_t ucCmdHeader;
} GPS_KALMAN_NoArgCmd_t;


typedef struct
{
    uint32  counter;

    /* TODO:  Add input data to this application here, such as raw data read from I/O
    **        devices or data subscribed from other apps' output data.
    */
    bool    gpsFixOk; /* is the data any good? */
    double  gpsLat;   /* GPS Lattidue */
    double  gpsLon;   /* GPS Longitude */
    double  gpsVel;   /* GPS Velocity */
    double  gpsHdg;   /* GPS Heading */
    double  gpsDOP;   /* GPS Position Dilution of Position */
} GPS_KALMAN_InData_t;

/* NOTE:  Moved GPS_KALMAN_OutData_t to mission_inc/gps_kalman_msg.h. */

/* TODO:  Add more private structure definitions here, if necessary. */

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

#endif /* _GPS_KALMAN_PRIVATE_TYPES_H_ */

/*=======================================================================================
** End of file gps_kalman_private_types.h
**=====================================================================================*/

