/*=======================================================================================
** File Name:  gps_kalman_data.c
**
** Title:  Source File for GPS_KALMAN Data definitions
**
** $Author:   Jacob Killelea
** $Revision: 1.1 $
** $Date:     2019-06-28
**
** Purpose: To define matrices for the kalman filter and statically allocate memory
**
** Modification History:
**   Date | Author | Description
**   ---------------------------
**   2019-09-12 | Jacob Killelea | Build #: Code Started
**
**=====================================================================================*/

#include "cfe.h"
#include "gps_kalman_data.h"

/* Vector data */
/* State vector */
static double XHatData[GPS_KALMAN_FILTER_LEN] = {0.0};
/* Next state vector */
static double XHatNextData[GPS_KALMAN_FILTER_LEN] = {0.0};
/* Expected measurement */
static double MuExpectedData[GPS_KALMAN_FILTER_LEN] = {0.0};
/* Actual measurement */
static double MuActualData[GPS_KALMAN_FILTER_LEN] = {0.0};

/* Matrix data */
/* State prediction matrix */
static double FMatrixData[GPS_KALMAN_FILTER_LEN * GPS_KALMAN_FILTER_LEN];
/* State covariance matrix */
static double PMatrixData[GPS_KALMAN_FILTER_LEN * GPS_KALMAN_FILTER_LEN] = {0.0};
/* State covariance increment matrix */
static double QMatrixData[GPS_KALMAN_FILTER_LEN * GPS_KALMAN_FILTER_LEN] = {0.0};
/* Measurement matrix */
static double HMatrixData[GPS_KALMAN_FILTER_LEN * GPS_KALMAN_FILTER_LEN] = {0.0};
/* Expected measuremnet covariance matrix */
static double SigmaExpectMatrixData[GPS_KALMAN_FILTER_LEN * GPS_KALMAN_FILTER_LEN] = {0.0};
/* Actual measuremnet covariance matrix */
static double SigmaActualMatrixData[GPS_KALMAN_FILTER_LEN * GPS_KALMAN_FILTER_LEN] = {0.0};
/* Kalman gain matrix */
static double KMatrixData[GPS_KALMAN_FILTER_LEN * GPS_KALMAN_FILTER_LEN] = {0.0};
/* Temporary matrix */
static double TmpMatrixData[GPS_KALMAN_FILTER_LEN * GPS_KALMAN_FILTER_LEN] = {0.0};
/* Temporary matrix */
static double TmpMatrix2Data[GPS_KALMAN_FILTER_LEN * GPS_KALMAN_FILTER_LEN] = {0.0};
/* Permutation matrix */
static gsl_permutation GSLPermutationData;


/* Use views to translate between arrays and vector/matrix objects */
static gsl_vector_view XHatView;
static gsl_vector_view XHatNextView;
static gsl_vector_view MuExpectedView;
static gsl_vector_view MuActualView;

static gsl_matrix_view FMatrixView;
static gsl_matrix_view HMatrixView;
static gsl_matrix_view KMatrixView;
static gsl_matrix_view PMatrixView;
static gsl_matrix_view QMatrixView;
static gsl_matrix_view SigmaActualMatrixView;
static gsl_matrix_view SigmaExpectMatrixView;
static gsl_matrix_view TmpMatrix2View;
static gsl_matrix_view TmpMatrixView;

gsl_vector *XHat = NULL; /* kalman state vector */
gsl_vector *XHatNext = NULL; /* kalman state vector */
gsl_vector *MuExpected = NULL; /* expected measurement */
gsl_vector *MuActual = NULL; /* actual measurement */

gsl_matrix *FMatrix = NULL; /* kalman system matrix */
gsl_matrix *HMatrix = NULL; /* kalman measurement matrix (identity for now) */
gsl_matrix *KMatrix = NULL; /* kalman gain */
gsl_matrix *PMatrix = NULL; /* kalman state covariance matrix */
gsl_matrix *QMatrix = NULL; /* kalman state covariance uncertainty matrix (0.1*identity for now) */
gsl_matrix *SigmaActualMatrix = NULL; /* actual covariance */
gsl_matrix *SigmaExpectMatrix = NULL; /* expected covariance */
gsl_matrix *TmpMatrix2 = NULL; /* Temporary matrix */
gsl_matrix *TmpMatrix = NULL; /* Temporary matrix */
gsl_permutation *GSLPermutation = NULL; /* used for inverting matrices */

/*=====================================================================================
** Name: GPS_KALMAN_Init_Matrix_Data
**
** Purpose: To initialize pointers to each matrix and vector from statically 
**          allocated arrays, as well as the gsl_permutation data
**
** Arguments:
**    None
**
** Returns:
**    None
**
** Routines Called:
**    gsl_matrix_view_array
**    gsl_vector_view_array
**
** Called By:
**    GPS_KALMAN_InitData
**
** Global Inputs/Reads:
**    None
**
** Global Outputs/Writes:
**    all gsl_matrix, gsl_vector, and gsl_permutation pointers
**
** Limitations, Assumptions, External Events, and Notes:
**    1. GSL will not fail silently but will throw some kind of exception
**
** Algorithm:
**    For the matrices and vectors: Create a view object from each array, assign the 
**    corresponding pointer to each view's internal matrix/vector
**
**    For the permutation: assign the permutation pointer to the interior, statically
**    allocated permutation
**
** Author(s):  Jacob Killelea
**
** History:  Date Written  2019-09-12
**           Unit Tested   yyyy-mm-dd
**=====================================================================================*/
void GPS_KALMAN_Init_Matrix_Data() {
    XHatView = gsl_vector_view_array(XHatData, GPS_KALMAN_FILTER_LEN);
    XHat = &XHatView.vector;

    XHatNextView = gsl_vector_view_array(XHatNextData, GPS_KALMAN_FILTER_LEN);
    XHatNext = &XHatNextView.vector;

    MuExpectedView = gsl_vector_view_array(MuExpectedData, GPS_KALMAN_FILTER_LEN);
    MuExpected = &MuExpectedView.vector;

    MuActualView = gsl_vector_view_array(MuActualData, GPS_KALMAN_FILTER_LEN);
    MuActual = &MuActualView.vector;

    FMatrixView = gsl_matrix_view_array(FMatrixData,
            GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    FMatrix = &FMatrixView.matrix;

    HMatrixView = gsl_matrix_view_array(HMatrixData,
            GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    HMatrix = &HMatrixView.matrix;

    KMatrixView = gsl_matrix_view_array(KMatrixData,
            GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    KMatrix = &KMatrixView.matrix;

    PMatrixView = gsl_matrix_view_array(PMatrixData,
            GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    PMatrix = &PMatrixView.matrix;

    QMatrixView = gsl_matrix_view_array(QMatrixData,
            GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    QMatrix = &QMatrixView.matrix;

    SigmaActualMatrixView = gsl_matrix_view_array(SigmaActualMatrixData,
            GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    SigmaActualMatrix = &SigmaActualMatrixView.matrix;

    SigmaExpectMatrixView = gsl_matrix_view_array(SigmaExpectMatrixData,
            GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    SigmaExpectMatrix = &SigmaExpectMatrixView.matrix;

    TmpMatrixView = gsl_matrix_view_array(TmpMatrixData,
            GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    TmpMatrix = &TmpMatrixView.matrix;

    TmpMatrix2View = gsl_matrix_view_array(TmpMatrix2Data,
            GPS_KALMAN_FILTER_LEN, GPS_KALMAN_FILTER_LEN);
    TmpMatrix2 = &TmpMatrix2View.matrix;

    GSLPermutation = &GSLPermutationData;
}


