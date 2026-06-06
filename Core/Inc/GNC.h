/*
 * GNC.h
 *
 *  Created on: Jun 5, 2026
 *      Author: Tristan
 */

#ifndef INC_GNC_H_
#define INC_GNC_H_

#include <stdint.h>
//Define pi macro
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//Define Structs
/*Navigation struct (the navigation states of the glider used in the guidance laws to steer the paraglider)
  Most likely will need to use the global mission struct and pull from relevant sensors ready
  for data acquisition (e.g. GPS and accel can be used for velocity computations, but accel may only be available at
  certain times, (this functionality must be programmed in the updateNavigation() function))*/
typedef struct {
    float pos_G[3][1]; //position of the glider in NED coordinates
    float vel_L[3][1]; //this is the local velocity in NED coordinates
    float vel_B[3][1]; //velocity in the body frame (obtained by doing coordinate transform from local NED to body)
    float rpy[3][1]; //attitude of the paraglider to do body to NED (or vice versa) conversion
    float HE; //Heading error of the paraglider in (deg)
    float trgt[3][1]; //the target that the glider is actively pursuing
    float trgtnodes[5][3]; //collection of target positions on the launch pad
    float deployHeight; //this is the altitude difference where the glider drops
    float gyro_old_r; //gyro rates stored from each previous iteration of the main loop
    float gyro_old_p;
    float gyro_old_y;
    int slack; //this is a boolean that determines if the glider has reached a target too early
    int mode; //integer that is 0 during (coast) and 1 during (pro-nav)
    int pursue; //boolean that determines if a target can be pursued
    int DROPNOW; //boolean that determines when the egg gets dropped
    int activateGNC; //boolean that determines when GNC main loop turns on
    int tgo;
    uint32_t time;
    uint32_t timeFound; // time a target was found
    uint32_t timeIntercept; //time a target has been intercepted
} Nav;
/*
Guidance struct contains the acceleration commands determined by a specified guidance law depending on the mode
in the navigation struct. This is then converted to a roll angle command in the autopilot function.
*/
typedef struct {
    float accel_cmd_L[3][1]; //position of the glider in NED coordinates
    float accel_cmd_B[3][1]; //this is the local velocity in NED coordinates
} Guidance;

typedef struct {
    float phi_cmd;
} AutoPilot;

//Function Definitions
float calculateTgo(float phi, float R, float Vg);
void findTarget(Nav *nav, float gps[3]);
float calculateHE(float pos_G[3][1], float vel_L[3][1]);
uint16_t computeCommand(Nav *nav, AutoPilot *ap);
Nav init_Navigation(float gps[3], float gyro[3][1], float accel[3][1]);
void Update_Autopilot(Guidance *guid, Nav *nav, AutoPilot *ap);
void Update_Guidance(Nav *nav, Guidance *guid);
void Update_Navigation(Nav *nav, float gps[3], float gyro[3][1], float accel[3][1], float gpsVelocity[3][1]);

#endif /* INC_GNC_H_ */
