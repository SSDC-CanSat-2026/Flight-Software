/*
 * GNC.c
 *
 *  Created on: Jun 5, 2026
 *      Author: Tristan
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "linalg.h"
#include "GNC.h"

#include "stm32g4xx_hal.h"

//Define pi macro
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
const float DEG2RAD = M_PI*(1.0/180.0);
//GLOBAL CONSTANTS - these are useful constant variables that the GNC algorithm will need
const float eps = 30.0;
#define DEL2US 5.379632835f //this directly converts a deflection command to microseconds

//6 axis mounting angles (replace depending on what sensor mounting orientation is)
#define IMUsens_yaw 90.0f //sensor mounting yaw
#define IMUsens_pitch 0.0f //sensor mounting pitch
#define IMUsens_roll 180.0f

#define clock_length 0.313f // seconds

const float phi_max = 10.0; //Maximum bank angle based on servo rotation limits

float calculateTgo(float phi, float R, float Vg) {
    /*----Description----
        calculateTgo() - Computes the time to go (t_go) using
        an analytical expression derived by Dhananjay and Ghose.
        Credits:
        Inputs:
        %1) Heading Error (phi)
        %2) Range (R)
        %3) Navigation Gain (N=3 for analytical solution)
        %4) Glider Velocity (Vg)
    */
    float N = 3.0;
    phi *= DEG2RAD;
    phi = fabsf(phi); //prevent negative sqrt
    float denom = powf(fabsf(sin(phi)),(1.0f/(N-1.0f)));
    float K = R/denom;
    //Expression in the paper for N=3 is used:
    float sqrtphi = powf(phi,0.5f);
    float tw14 = powf(12.0f,0.25f);
    float num = sqrtphi+tw14;
    denom = sqrtphi-tw14;
    //Calculate time to go using analytical equation
    float t_go = (K/(2.0f*Vg))*(tw14)*((0.5f*log(fabsf(num/denom)))+atan(sqrtphi/tw14));

    return t_go;
}

void findTarget(Nav *nav, float gps[3]){
    //Inputs:
    //Nav nav = navigation structure
    //gps = {latitude,longitude,altitude}
    /*---- Description -----
    findTarget() - selects a target point within the target drop zone
    to navigate towards based on the following critieria
    /*Notes:
        1) Because most linear algebra functions in the linalg.c library use void functions,
        arrays are is pre-allocated internally within this function.
        2) trgt is maintained by the navigation function
    */
    float trgtprev = nav->trgt[0][0]; //keeping track of current trgt
    float rtp[3][1] = {{0},{0},{0}}; //target to glider NED vector preallocated.
    geodetic2ned(nav->trgtnodes[0][0],nav->trgtnodes[0][1],nav->trgtnodes[0][2],gps[0],gps[2],nav->pos_G[2][0],rtp); //get target to glider NED
    float arr1[2] = {rtp[0][0],rtp[1][0]}; //just want NE distance
    float dist = sqrtf(dot(arr1,arr1,2)); //compute north east distance to center target position
    float distprev = dist;

    //We iterate on each target, and check which one has the closest altitude time to intercept time
    if (nav->slack == 0) { //no slack time, searching for target during coast
        for (int i=1;i<5;i++) {
            //Check the distance from each target
            geodetic2ned(nav->trgtnodes[i][0],nav->trgtnodes[i][1],nav->trgtnodes[i][2],gps[0],gps[1],nav->pos_G[2][0],rtp); //get target to glider NED
            float arr1[2] = {rtp[0][0],rtp[1][0]};
            dist = dot(arr1,arr1,2);
            if (dist<distprev && dist > eps) { //if a closer target is found that satisfies the curvature limit
                //UPDATE - NEED TO HAVE STRUCT POINTER PASSED TO FUNCTION
                nav->trgt[0][0] = nav->trgtnodes[i][0];
                nav->trgt[1][0] = nav->trgtnodes[i][1];
                nav->trgt[2][0] = nav->trgtnodes[i][2];
                printf("FOUND TARGET\n");
            }
        }
    }
    else { //target intercepted with time remaining until altitude, now we wait for target
        //Check the time to go of each target and look for the one with the smallest POSITIVE time difference
        float arr1[2] = {rtp[0][0],rtp[1][0]};

        float vel_L[3][1] = {{0.0},{0.0},{0.0}}; //assign nav arrays to avoid passing more pointers
        vel_L[0][0] = nav->vel_L[0][0];
        vel_L[1][0] = nav->vel_L[1][0];
        vel_L[2][0] = nav->vel_L[2][0];

        float pos_G[3][1] = {{0.0},{0.0},{0.0}};
        pos_G[0][0] = nav->pos_G[0][0];
        pos_G[1][0] = nav->pos_G[1][0];
        pos_G[2][0] = nav->pos_G[2][0];

        nav->HE = calculateHE(pos_G,vel_L);
        float talt = (nav->pos_G[2][0]-nav->deployHeight)/nav->vel_L[2][0];
        float Vgarr[2] = {nav->vel_L[0][0],nav->vel_L[1][0]};
        float Vg = dot(Vgarr,Vgarr,2);
        dist = sqrt(dot(arr1,arr1,2));
        float tgo = calculateTgo(nav->HE,dist,Vg);
        float tgo_prev = tgo;
        float tdiff;
        float tdiffprev;
        for (int i=1;i<5;i++) {
            //Check the time to go from each target
            geodetic2ned(nav->trgtnodes[i][0],nav->trgtnodes[i][1],nav->trgtnodes[i][2],gps[0],gps[1],gps[2],rtp); //get target to glider NED
            float arr1[2] = {rtp[0][0],rtp[1][0]}; //getting NE distance
            nav->HE = calculateHE(rtp,nav->vel_L);
            float Vgarr[2] = {nav->vel_L[0][0],nav->vel_L[1][0]}; //getting velocity in North-East plane
            Vg = dot(Vgarr,Vgarr,2);
            dist = dot(arr1,arr1,2);
            tgo = calculateTgo(nav->HE,dist,Vg); //use calculate tgo function
            tdiff = tgo-talt; //how close is target intercept time to reaching the altitude?

            if (tdiff<tdiffprev && tdiff > 0.0f) {
                //Found a target that has a closer intercept time but is still positive in time difference
                //this ensures that the
                //UPDATE - NEED TO HAVE STRUCT POINTER PASSED TO FUNCTION
                nav->trgt[0][0] = nav->trgtnodes[i][0];
                nav->trgt[1][0] = nav->trgtnodes[i][1];
                nav->trgt[2][0] = nav->trgtnodes[i][2];
                printf("Target found!\n");
                nav->pursue = 1;
            }
            else if (fabsf(tdiff)<fabsf(tdiffprev)){ // otherwise fing the target with a smaller time regardless of sign
                nav->trgt[0][0] = nav->trgtnodes[i][0];
                nav->trgt[1][0] = nav->trgtnodes[i][1];
                nav->trgt[2][0] = nav->trgtnodes[i][2];
                printf("FOUND TARGET\n");
                nav->pursue = 1;
            }
            //otherwise, don't assign a new target
            tdiffprev = tdiff;
        }
        //Check if the trgt changed, if it did, we found a target to pursue.
        if (nav->pursue == 1) {
            printf("FOUND TARGET\n");
            nav->timeFound = HAL_GetTick();
            nav->timeIntercept = (uint32_t)(tgo/1000.0)+HAL_GetTick();
        }
        else {
            printf("Did not find a new target\n");
        }
    }
}

float calculateHE(float pos_G[3][1], float vel_L[3][1]) {
    printf("TEST PRINT\n");
    /*Description: calculateHE() returns the heading error between the line of sight
    of the target with respect to the glider and the North-East velocity vector of the glider.
    Notes:
    1) It is assummed that r_tp and vel_L are global navigation states. Both are
    3x1 arrays.
    */
    float gamma = atan2f(vel_L[1][0],vel_L[0][0]); //compute the flight path angle of the glider
    float los = atan2f(-pos_G[1][0],-pos_G[0][0]); //compute the line of sight angle of the glider
    float HE = (gamma-los)/DEG2RAD;
    return HE;
}

float computeCommand(Nav *nav, AutoPilot *ap) {
    /*
    Calculates a PWM signal to the motor based on given roll command and current roll
    The current method just uses a proportional controller to bank the glider in
    the direction of desired bank angle.
    */
    float Kp = 2.0f; //proportional gain term
    float err = ap->phi_cmd-nav->rpy[0][0]*1.0/DEG2RAD; //error in roll
    float cmd = DEL2US*Kp*err;

    cmd += 135.0; //add servo command to servo home at 1500 us
    if (cmd > 270.0) {
        cmd = 270.0; //clip to max rotation
    }
    if (cmd < 0.0) {
        cmd = 0.0; //clip to min rotation
    }
    //printf("Servo Command (us): %0.3d\n",cmd);
    return cmd;
}

void Update_Autopilot(Guidance *guid, Nav *nav, AutoPilot *ap){
    if (nav->mode == 0) { //Coasting phase - glider moves in a straight line
        ap->phi_cmd = 0.0; //autopilot just keeps paraglider at a level bank
    }
    else {
        ap->phi_cmd = atan2f(guid->accel_cmd_B[1][0], -guid->accel_cmd_B[2][0]+0.001f); //coordinated turn bank equation
        ap->phi_cmd *= 1/DEG2RAD;
        if (fabsf(ap->phi_cmd) > phi_max) {
            ap->phi_cmd = (ap->phi_cmd/ap->phi_cmd)*phi_max; // clip to max bank and keep the sign
        }
    }
}

void Update_Guidance(Nav *nav, Guidance *guid) { //computes Guidance commands depending on mode
    printf("px: %0.2f\n",nav->pos_G[0][0]);
    float R_BL[3][3] = {{0,0,0},  //form rotation matrix from local NED frame to body frame
                        {0,0,0},
                        {0,0,0}};
    float R_LB[3][3] = {{0,0,0},  //transposed matrix (so we don't change the original one)
                        {0,0,0},
                        {0,0,0}};
    rpyDCM(nav->rpy[0][0],nav->rpy[1][0],nav->rpy[2][0],R_BL);
    rpyDCM(nav->rpy[0][0],nav->rpy[1][0],nav->rpy[2][0],R_LB);
    transposeMat(R_LB);
    if (nav->mode == 0) {
        printf("Coasting Phase\n");
        //Don't do anything, just coasting (using gravity in to avoid accidentally entering a conditional where we divide by 0.0)
        guid->accel_cmd_L[0][0] = 0.0;
        guid->accel_cmd_L[0][1] = 0.0;
        guid->accel_cmd_L[0][2] = -9.80556;
        matmult(3,1,R_LB,guid->accel_cmd_L,guid->accel_cmd_B);
    }
    else { //nav->mode == 1 (Pro-Nav)
        //---------- PROPORTIONAL NAVIGATION ----------
        //Parameters:
        float N = 3.0; //proportional gain (2-5 is typical)
        matmult(3,1,R_LB,nav->vel_L,nav->vel_B);
        //Relative positions and velocities
        float RtgN = -nav->pos_G[0][0]; //target is origin so just negation of glider pos.
        float RtgE = -nav->pos_G[1][0];
        float R[2] = {RtgN,RtgE};


        float VtgN = -nav->vel_L[0][0];
        float VtgE = -nav->vel_L[1][0];
        float RTG = sqrtf(dot(R,R,2)); //Get planar North-East distance from target to glider
        //Line of sight / line of sight rate
        float los = atan2f(RtgE,RtgN);
        float los_dot = (RtgN*VtgE-RtgE*VtgN)/(powf(RTG,2.0));

        //Control Law (Pure Proportional Navigation)
        //Since using pure pro-nav, we do not use the closing velocity
        //instead, we use the velocity along the x axis of the body frame.
        guid->accel_cmd_B[1][0] = N*nav->vel_B[1][0]*los_dot;
        guid->accel_cmd_B[2][0] = -9.79774; //gravity at CANSAT location
        float ay = N*(nav->vel_B[0][0])*los_dot;
        float az = -9.79774;
        printf("los_dot: %0.5f\n",powf(RTG,2.0));
        printf("Accel CMD, az: %0.5f\n",az);
    }
}

Nav init_Navigation(float gps[3], float gyro[3][1], float accel[3][1]) {
    //Inputs:
        // *nav - pointer to navigation struct
        // gps = {latitude,longitude,height}
        // gyro = {{roll_rate},{pitch_rate},{yaw_rate}}
        // accel = {{accel_x},{accel_y},{accel_z}}
//    gps[0] = 38.37564166666667;
//    gps[1] = -79.60739444444444;
//    gps[2] = 1050.0;
    //Right now not using gyro and accel. But they should be used for determining offsets

    /*This function commputes the navigation states from initial sensor data on the launch pad before launch
    init_Navigation is only called once to initialize the navigation struct.
    Would be better to add averaged values of each sensor estimate needed by each nav state rather than instantaneous values.
    If this could be done similar to altitude calibration, that would be ideal, but for now just using instantaneous readings.
    Unless the sensor measurements already have an offset applied to them already.
    */
    Nav nav = {0};
    nav.trgt[0][0] = 38.3760167f; //this is the target directly in the center of the drop zone
    nav.trgt[1][0] = -79.6078722f;
    nav.trgt[2][0] = gps[2]; //current altitude measured by the GPS (mean height above ellipsoid)
    nav.deployHeight = gps[2]+2.0; //set the deployHeight to 2 meters above the launch pad height via rules
    const float trgtnodes[5][3] = {{38.37606944444445,-79.60810277777777,nav.deployHeight}, //5 points on the drop zone (2 up, 1 center, 2 down)
            {38.376127777777775,-79.60805277777777,nav.deployHeight},
            {38.376016666666665, -79.60787222222221,nav.deployHeight},
            {38.375905555555555, -79.607675,nav.deployHeight},
            {38.375952777777776, -79.60763055555555,nav.deployHeight}};

    geodetic2ned(nav.trgt[0][0],nav.trgt[1][0],gps[2],gps[0],gps[1],gps[2],nav.pos_G);

    nav.vel_L[0][0] = 0.0; //not moving, so just initialize velocities to zero
    nav.vel_L[1][0] = 0.0; //only offsets that would be applicable is if we were using PSTMPV
    nav.vel_L[2][0] = 0.0;

    nav.vel_B[0][0] = 0.0;
    nav.vel_B[1][0] = 0.0;
    nav.vel_B[2][0] = 0.0;

    nav.gyro_old_r = 0.0; //gyro rates stored from the previous iteration
    nav.gyro_old_p = 0.0;
    nav.gyro_old_y = 0.0;

    nav.HE = 0.0; //heading error between glider and the target
    nav.slack = 0; //slack determines if there is residual time when glider reaches the target position
    nav.mode = 0; //mode determines if glider is in coast or in pro-nav
    nav.pursue = 0; //pursue is a boolean that determines whether a target is pursued or not
    nav.activateGNC = 0; //this variable determines when the main GNC loop takes place
    //nav.time = HAL_GetTick();
    nav.time = 0;
    nav.timeFound = 0; // time a target was found
    nav.timeIntercept = 0; //time a target has been intercepted
    return nav;
}

void Update_Navigation(Nav *nav, float gps[3], float gyro[3][1], float accel[3][1], float gpsVelocity[3][1]) {
    //Inputs:
        // *nav - pointer to navigation struct
        // gps = {latitude,longitude,height}
        // gyro = {{roll_rate},{pitch_rate},{yaw_rate}}
        // accel = {{accel_x},{accel_y},{accel_z}}

    //------------------- ATTITUDE (roll,pitch,yaw in degrees) -------------------
    //NOTE: CONDITIONALS MUST BE IMPLEMENTED TO CHECK GPS STATUS
    //Integrate gyro and combine with accel (no mag yet, this is bare minimum)
    float w1 = 0.9; //complementary filter weights
    float w2 = 0.1;

    //Conversion from IMU sensor frame to body frame due to mounting misalignment
    float R_IB[3][3] = {{0.0,0.0,0.0},
                        {0.0,0.0,0.0},
                        {0.0,0.0,0.0}}; //convert IMU sensor frame to body frame (to account for sensor orientation)
    rpyDCM(IMUsens_roll,IMUsens_pitch,IMUsens_yaw,R_IB); //compute the rotation matrix from sensor frame to body frame

    //TIME VARIABLES
    uint32_t currtime = HAL_GetTick();
    float delta_t = (float)(currtime-nav->time) / 1000.0; //get time difference
    nav->time = currtime;

    //GYRO ANGLES - integrate previous gyro angles over time step times the current gyro rate
    float gyro_rpy[3][1] = {{0.0},{0.0},{0.0}};
    gyro_rpy[0][0] = (nav->gyro_old_r+gyro[0][0]*delta_t)*DEG2RAD; //gyro roll, pitch, yaw angles
    gyro_rpy[1][0] = (nav->gyro_old_p+gyro[1][0]*delta_t)*DEG2RAD;
    gyro_rpy[2][0] = (nav->gyro_old_y+gyro[2][0]*delta_t)*DEG2RAD;


    nav->gyro_old_r = gyro_rpy[0][0]/DEG2RAD;
    nav->gyro_old_p = gyro_rpy[1][0]/DEG2RAD;
    nav->gyro_old_y = gyro_rpy[2][0]/DEG2RAD;


    //ACCELEROMETER TRIGONOMETRY - Using trigonometry to determine roll and pitch with respect to gravity
    //Note we cannot get yaw from an accelerometer, there is no reference acceleration we can base measurements off of.
    matmult(3,1,R_IB,gyro_rpy,gyro_rpy);
    matmult(3,1,R_IB,accel,accel);

    float ayz[2] = {accel[1][0],accel[2][0]};
    float ayz_mag = sqrtf(dot(ayz,ayz,2));
    float accel_rang = atan2f(-accel[0][0],ayz_mag); //accelerometer roll and pitch angles
    float accel_pang = atan2f(accel[1][0],accel[2][0]);

    //Final Attitude estimation - combine using a complementary filter
    nav->rpy[0][0] = gyro_rpy[0][0]; // (w1*gyro_rpy[0][0])+(w2*accel_rang); //roll estimation
    nav->rpy[1][0] = gyro_rpy[1][0]; //(w1*gyro_rpy[1][0])+(w2*accel_pang); //pitch estimation

    //Yaw Angles - If the magnetometer is available can add ecompass() algorithm with accel+mag later
    //If we can get GPS velocity in North, East directions, we can calculate GPS heading and add to gyro.

    //Magnetometer needs to be projected in the NED frame for heading

    //float mag_NED[3][1] = {{mag_data->x},{mag_data->y},{mag_data->z}};
    //matmult(3,1,R_MB,mag_NED,mag_NED);

    //float mag_yaw = atan2f(mag_NED[1][0],mag_NED[0][0]);
    //mag_yaw = mag_yaw+mag_decl;
    float gpsYaw = atan2f(gpsVelocity[1][0],gpsVelocity[0][0]);
    nav->rpy[2][0] = gyro_rpy[2][0]; // (w1*gyro_rpy[2][0])+(w2*gpsYaw); //can only use gyro yaw

    //------------------- VELOCITY (North, East, Down in m/s) -------------------
    //VELOCITY
    // float w1_vel = 0.96;
    // float w2_vel = 0.04;
    //Note, GPS may not be ready, add conditional that integrates accelerometer completely if this is the case
    float vel_accelx = nav->vel_L[0][0]+accel[0][0]*delta_t;
    float vel_accely = nav->vel_L[1][0]+accel[1][0]*delta_t;
    float vel_accelz = nav->vel_L[2][0]+accel[2][0]*delta_t;

    nav->vel_L[0][0] = (w1*gpsVelocity[0][0])+(w2*vel_accelx);
    nav->vel_L[1][0] = (w1*gpsVelocity[1][0])+(w2*vel_accely);
    nav->vel_L[2][0] = (w1*gpsVelocity[2][0])+(w2*vel_accelz);

    //------------------ POSITION (North, East, Down in m) -------------------
    float pos_GPS[3][1] = {{0.0},{0.0},{0.0}};
    geodetic2ned(nav->trgt[0][0],nav->trgt[1][0],nav->trgt[2][0],gps[0],gps[1],gps[2], pos_GPS);
    //integrate the navigation velocity to get new position estimate
    //Combine with the pure GPS lat,long estimate using a complementary filter
    float pos_vel_x = nav->pos_G[0][0]+nav->vel_L[0][0]*delta_t;
    float pos_vel_y = nav->pos_G[1][0]+nav->vel_L[1][0]*delta_t;
    float pos_vel_z = nav->pos_G[2][0]+nav->vel_L[2][0]*delta_t;

    nav->pos_G[0][0] = (w1*pos_GPS[0][0])+(w2*pos_vel_x);
    nav->pos_G[1][0] = (w1*pos_GPS[1][0])+(w2*pos_vel_y);
    nav->pos_G[2][0] = (w1*pos_GPS[2][0])+(w2*pos_vel_z);

    //check target intercept
    if (fabsf(nav->pos_G[0][0])<12.192f && fabsf(nav->pos_G[1][0])<12.192f && nav->mode == 1){ //about the width of the drop zone
        printf("At the target! Switching to coast mode\n");
        nav->mode = 0; //enter back into coasting
        nav->pursue = 0; //target is not pursued until another is found
        nav->slack = 0;
        //CHECK EGG DROP - if altitude close to deployment height, releeze ze babee!
        //In main contol loop we check DROPNOW state over and over and release egg when it's 1.
        if ((nav->pos_G[2][0]-nav->deployHeight)<5.0) { //include some offset between release height and glider height due to release delay
            nav->DROPNOW = 1; // we are go for drop
        }
        else if ((nav->time > nav->timeIntercept)){
            nav->DROPNOW = 0; //not ready for deployment
            nav->slack = 1; //slack time available to loiter in drop zone
        }
    }
}
