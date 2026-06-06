/*
 * linalg.c
 *
 *  Created on: Jun 5, 2026
 *      Author: Tristan
 */

#include <stdio.h>
#include <math.h>

//Define pi macro
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float d2r = M_PI*(1.0/180.0);

void printArray(float arr[], int len){
    for (int i=0;i<len;i++) {
        printf("%0.3f ",arr[i]);
    }
}

float dot(float arr1[],float arr2[],int size){
    /*
    This function computes the dot product between two arrays.
    Arrays can only be passed as pointers and must be allocated to
    the heap to ensure that the modification of the array is NOT internal to
    the function.
    */
    //Inputs: two arrays and the size of either (both must be the same)
    //Testing Computation of the Dot Product
    float dot_sum = 0;
    for (int i=0;i<size;i++){
        dot_sum += arr1[i]*arr2[i];
    }
    return dot_sum;
}

void transposeMat(float A[][3]) {
    //Tranposes a matrix (A)
    //Start by pre-allocating memory for resultant matrix
    float Res[3][3] = {{0,0,0},
                       {0,0,0},
                       {0,0,0}};
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            Res[i][j] = A[j][i];
        }
    }
    //Copy elements (I know this is horribly inefficient)
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            A[i][j] = Res[i][j];
        }
    }
}

void matmult(int n, int m, float mat1[n][n], float mat2[n][m], float AB[n][m]){
    /* -----------------------------------------------------------------------------------
    Multiplies an nxn matrix by an nxm matrix. The result (and what needs to be preallocated in memory),
    is an nxm dimension matrix.

    Note: to reduce memory overhead, AB is a preallocated array of nxm size, and is updated
    by this function if it is defined globally. This reduces the need for complex memory allocation
    which would be a significant headache cause we will need to do this quite a bit.
    --------------------------------------------------------------------------------------*/
    for (int i=0;i<n;i++){ //iterating by row
        for (int j=0;j<m;j++){ //iterating by column
            AB[i][j] = mat1[i][0]*mat2[0][j]
                    +mat1[i][1]*mat2[1][j]
                    +mat1[i][2]*mat2[2][j];
        }
    }
}

void computeDCM(float th, char ax, float R[3][3]) {
    //Trigonometric functions require degrees, so use conversion factor
    th *= d2r;
    switch(ax) {
        case ('x'): //Rotation about X
            R[0][1] = 1; R[0][1]= 0; R[0][2]= 0;
            R[1][0] = 0; R[1][1] = cos(th); R[1][2]= sin(th);
            R[2][0] =0; R[2][1]= -sin(th); R[2][2]= cos(th);
            break;
        case ('y'): //Rotation about Y
            R[0][1] = cos(th); R[0][1]= 0; R[0][2]= -sin(th);
            R[1][0] = 0; R[1][1] = 1; R[1][2]= 0;
            R[2][0] = sin(th); R[2][1]= 0; R[2][2]= cos(th);
            break;
        case ('z'): //Rotation about Z
            R[0][1] = 1; R[0][1]= 0; R[0][2]= 0;
            R[1][0] = 0; R[1][1] = cos(th); R[1][2]= sin(th);
            R[2][0] =0; R[2][1]= -sin(th); R[2][2]= cos(th);
            break;
    }

}
/*
    ---------- Rotation Matrix r,p,y generator  ----------
    Computes the roll, pitch, yaw Rotation Matrix that goes from NED to body frame
    This requires the user to specify the axis that the rotation occurs about in a string

    Inputs: (roll, pitch, yaw) as floats
    Outputs: 3x3 array compound rotation matrix used to transform NED to body frame
*/
void rpyDCM(float r,float p,float y, float R_py[3][3]) {
    r *= d2r;
    p *= d2r;
    y *= d2r;
    printf("%0.2f\n",r);
    printf("%0.2f\n",p);
    printf("%0.2f\n",y);
    //C(y,p,r) = R(y)R(p)R(r) the result below are all components of the compound transformation matrix
    R_py[0][0] = cosf(y)*cosf(p); R_py[0][1]= cosf(y)*sinf(p)*sinf(r)-sinf(y)*cosf(r); R_py[0][2]= cosf(y)*sinf(p)*cosf(r)+sinf(y)*sinf(r);
    R_py[1][0] = sinf(y)*cosf(p); R_py[1][1] = sinf(y)*sinf(p)*sinf(r)+cosf(y)*cosf(r); R_py[1][2]= sinf(y)*sinf(p)*cosf(r)-cosf(y)*sinf(r);
    R_py[2][0] = -sinf(p); R_py[2][1]= cosf(p)*sinf(r); R_py[2][2]= cosf(p)*cosf(r);
}

void geodetic2ecef(float latpt, float longpt, float h,float r[3][1]) {
    /*
    #Description: Finds the position vector of a specific point in on Earth using Earth-Centered-Earth-Fixed
    #(ECEF) coordinates. This uses equations that can be found on ESA's website, but it mainly comes from the
    # local geometry of the WGS-84 ellisoid.

    #Variables:
    #1) latpt,longpt: latitude and longitude of the point
    #2) h: mean height above ellipsoid
    */
    float a = 6378137.0; //equatorial radius of the Erf
    float f = 1.0/298.257223563;
    float e2 = f*(2-f);
    //Make sure to convert to radians before using transformations
    latpt *= d2r;
    longpt *= d2r;
    float N = a/sqrtf(1-e2*sinf(latpt*latpt)); //local curvature of the ellipsoid
    float x = (N+h)*cosf(latpt)*cosf(longpt); //Coordinates of point in ECEF
    float y = (N+h)*cosf(latpt)*sinf(longpt);
    float z = (N*(1-e2)+h)*sinf(latpt);

    r[0][0] = x;
    r[1][0] = y;
    r[2][0] = z;
}

void geodetic2ned(float lat_t,float long_t,float h_t,float lat_p,float long_p,float h_p, float r_tp[3][1]){
    /*
    #geodetic2ned: Finds the relative position vector between the target geot = (lat_t,long_t,h_s)
    #and the paraglider geop = (lat_p,long_p,h_p) in a North, East, Down coordinates.
    #Uses the WGS-84 ellipsoid model of earth to get position of target and glider in ECEF coordinates
    #then transforms relative vector to NED frame with a rotation matrix

    #Variables:
    #1) lat_t,long_t: latitude and longitude of the target (fixed)
    #2) long_p,long_p: latitude and longitude of the paraglider
    #3) h_t,h_p: mean height above ellipsoid
    */
    float r_t[3][1] = {{0},{0},{0}};
    float r_p[3][1] = {{0},{0},{0}};
    geodetic2ecef(lat_t,long_t,h_t,r_t);
    geodetic2ecef(lat_p,long_p,h_p,r_p);

    //Now that both vectors have been defined, we can compute the relative position vector in ECEF frame.
    float r_rel[3][1] = {{0},{0},{0}};
    r_rel[0][0] = r_t[0][0]-r_p[0][0];
    r_rel[1][0] = r_t[1][0]-r_p[1][0];
    r_rel[2][0] = r_t[2][0]-r_p[2][0];

    //This need to be transformed into the NED frame. So we make a transformation matrix and perform matrix multiplication
    //Make transformation matrix
    float R_2ned[3][3] = {{0,0,0},
                       {0,0,0},
                       {0,0,0}};
    //make sure to convert all latitude and longitude to radians
    lat_t *= d2r;
    long_t *= d2r;
    lat_p *= d2r;
    long_p *= d2r;
    R_2ned[0][0] = -sinf(lat_t)*cosf(long_t); R_2ned[0][1] = -sinf(lat_t)*sinf(long_t); R_2ned[0][2] = cosf(lat_t);
    R_2ned[1][0] = -sinf(long_t); R_2ned[1][1] = cosf(long_t); R_2ned[1][2] = 0.0;
    R_2ned[2][0] = -cosf(lat_t)*cosf(long_t); R_2ned[2][1] = -cosf(lat_t)*sinf(long_t); R_2ned[2][2] = -sinf(lat_t);

    matmult(3,1,R_2ned,r_rel,r_tp);


}
