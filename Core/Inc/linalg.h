/*
 * linalg.h
 *
 *  Created on: Jun 5, 2026
 *      Author: Tristan
 */

#ifndef INC_LINALG_H_
#define INC_LINALG_H_

// Constants
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Function Prototypes
void printArray(float arr[], int len);
float dot(float arr1[], float arr2[], int size);
void transposeMat(float A[][3]);
void matmult(int n, int m, float mat1[n][n], float mat2[n][m], float AB[n][m]);
void computeDCM(float th, char ax, float R[3][3]);
void rpyDCM(float r,float p,float y, float R_py[3][3]);
void geodetic2ecef(float latpt, float longpt, float h,float r[3][1]);
void geodetic2ned(float lat_t,float long_t,float h_t,float lat_p,float long_p,float h_p, float r_tp[3][1]);

#endif /* INC_LINALG_H_ */
