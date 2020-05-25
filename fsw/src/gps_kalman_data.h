#ifndef GPS_KALMAN_DATA_H_
/*=======================================================================================
** File Name:  gps_kalman_data.h
**
** Title:  Header File for GPS_KALMAN Data definitions
**
** $Author:    Jacob Killelea
** $Revision: 1.1 $
** $Date:      2019-06-28
**
** Purpose: To define matrices for the kalman filter and statically allocate memory
**
** Modification History:
**   Date | Author | Description
**   ---------------------------
**   2019-09-12 | Jacob Killelea | Build #: Code Started
**
**=====================================================================================*/

#define GPS_KALMAN_DATA_H_

#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>


/* Size of the vectors (1x3 and matrices (3x3) in the filter */
#define GPS_KALMAN_FILTER_LEN (3)

extern gsl_vector *XHat;                /* kalman state vector */
extern gsl_vector *XHatNext;            /* kalman state vector */
extern gsl_vector *MuExpected;          /* expected measurement */
extern gsl_vector *MuActual;            /* actual measurement */
extern gsl_matrix *FMatrix;             /* kalman system matrix */
extern gsl_matrix *HMatrix;             /* kalman measurement matrix (identity for now) */
extern gsl_matrix *KMatrix;             /* kalman gain */
extern gsl_matrix *PMatrix;             /* kalman state covariance matrix */
extern gsl_matrix *QMatrix;             /* kalman state covariance uncertainty matrix (0.1*identity for now) */
extern gsl_matrix *SigmaActualMatrix;   /* actual covariance */
extern gsl_matrix *SigmaExpectMatrix;   /* expected covariance */
extern gsl_matrix *TmpMatrix2;          /* Temporary matrix */
extern gsl_matrix *TmpMatrix;           /* Temporary matrix */
extern gsl_permutation *GSLPermutation; /* used for inverting matrices */

/* Initialize all the pointer above from static memory (no deallocation needed) */
void GPS_KALMAN_Init_Matrix_Data(void);

#endif /* end of include guard: GPS_KALMAN_DATA_H_ */

