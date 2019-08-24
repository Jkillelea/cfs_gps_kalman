/*=======================================================================================
** File Name:  gps_kalman_utils.c
**
** Title:  Utility Function Definitions for GPS_KALMAN Application
**
** $Author:    Jacob Killelea
** $Revision: 1.1 $
** $Date:      2019-06-28
**
** Purpose:  This file contains extra math based subroutines needed by GPS_KALMAN
**
** Functions Defined:
**    Function decimal_minutes2decimal_decimal: converts a decimal-minutes formatted number to pure decimal
**
** Limitations, Assumptions, External Events, and Notes:
**    1. List assumptions that are made that apply to all functions in the file.
**       - These functions should be pure, or at least deterministic
**    2. List the external source(s) and event(s) that can cause the funcs in this
**       file to execute.
**       - Called to perform specialized calculations by portions of gps_kalman_app.c
**    3. List known limitations that apply to the funcs in this file.
**    4. If there are no assumptions, external events, or notes then enter NONE.
**       Do not omit the section.
**
** Modification History:
**   Date | Author | Description
**   ---------------------------
**   2019-08-19 | Jacob Killelea | Build #: Code Started
**
**=====================================================================================*/


/* convert from DDDMM.mmmmm (decimal minutes) to DDD.dddddd (plain decimal) format */
/* TODO: double check this! */
double decimal_minutes2decimal_decimal(const double decimal_minutes) {
    double degrees = ((int) (decimal_minutes/100.0)); /* DDD */
    double minutes = decimal_minutes - 100*degrees;   /* MM.mmmmmm */
    double decimal = minutes / 60;                    /* 0.ddddddd */
    return (degrees + decimal);                       /* DDD.dddddd */
}


