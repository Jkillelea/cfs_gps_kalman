/*=======================================================================================
** File Name:  gps_kalman_msgids.h
**
** Title:  Message ID Header File for GPS_KALMAN Application
**
** $Author:    Jacob Killelea
** $Revision: 1.1 $
** $Date:      2019-06-28
**
** Purpose:  This header file contains declartions and definitions of all GPS_KALMAN's 
**           Message IDs.
**
** Modification History:
**   Date | Author | Description
**   ---------------------------
**   2019-06-28 | Jacob Killelea | Build #: Code Started
**   2019-06-28 | Jacob Killelea | Msg ids made (hopefully) unique
**
**=====================================================================================*/
    
#ifndef _GPS_KALMAN_MSGIDS_H_
#define _GPS_KALMAN_MSGIDS_H_

/***** TODO:  These Message ID values are default and may need to be changed by the developer  *****/
#define GPS_KALMAN_CMD_MID            	0x18E0
#define GPS_KALMAN_SEND_HK_MID        	0x18E1
#define GPS_KALMAN_WAKEUP_MID        	0x18F0
#define GPS_KALMAN_OUT_DATA_MID        	0x18F1

#define GPS_KALMAN_HK_TLM_MID		0x08CC

    


#endif /* _GPS_KALMAN_MSGIDS_H_ */

/*=======================================================================================
** End of file gps_kalman_msgids.h
**=====================================================================================*/
    
