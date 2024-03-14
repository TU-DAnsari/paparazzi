/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/our_avoider/our_avoider.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
<<<<<<< HEAD
#include <math.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"
#include "../../boards/ardrone2.h"
=======

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"
>>>>>>> 4dbfe1347 (add orange avoidance control)

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

<<<<<<< HEAD
float CalcDifferenceInHeading(float dronex, float droney, float goalx, float goaly);
float computePIDheading(float droneheading, float targetheading);

=======
>>>>>>> 4dbfe1347 (add orange avoidance control)
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
<<<<<<< HEAD
  OUT_OF_BOUNDS,
  TOP_LINE,
  RIGHT_LINE,
  BOTTOM_LINE,
  LEFT_LINE
};

<<<<<<< HEAD
// xy velocity settings
float forward_velocity = .7f;
float turning_rate = .5f;

// yaw rate settings
float k_outer = .2f;
float k_inner = .4f;

// green detection settings
float floor_count_frac = 0.1;

=======
// define settings
float forward_velocity = .5f;
float k_outer = .2f;
float k_inner = .4f;

>>>>>>> 3ff52f613 (fix segmentation issues for very simple autonomous flight)
int16_t color_count_a = 0;
int16_t color_count_b = 0; 
int16_t color_count_c = 0; 
int16_t color_count_d = 0; 
<<<<<<< HEAD

int32_t floor_count = 0;

enum navigation_state_t navigation_state = SAFE;

=======
>>>>>>> 3ff52f613 (fix segmentation issues for very simple autonomous flight)

=======
  OUT_OF_BOUNDS
};

// define settings
float forward_velocity = .5f;
float k_outer = .2f;
float k_inner = .4f;

int16_t color_count_a = 0;
int16_t color_count_b = 0; 
int16_t color_count_c = 0; 
int16_t color_count_d = 0; 

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
>>>>>>> 4dbfe1347 (add orange avoidance control)
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
<<<<<<< HEAD
<<<<<<< HEAD
                               int16_t count_region_a, int16_t count_region_b,
                               int16_t count_region_c, int16_t count_region_d,
=======
                               int32_t count_region_a, int32_t count_region_b,
                               int32_t count_region_c, int32_t count_region_d,
>>>>>>> 4dbfe1347 (add orange avoidance control)
=======
                               int16_t count_region_a, int16_t count_region_b,
                               int16_t count_region_c, int16_t count_region_d,
>>>>>>> 0857f044b (fix segmentation issues for very simple autonomous flight)
                               int32_t __attribute__((unused)) quality, int16_t __attribute__((unused)) extra)
{
  color_count_a = count_region_a;
  color_count_b = count_region_b;
  color_count_c = count_region_c;
  color_count_d = count_region_d;
}


<<<<<<< HEAD
#ifndef FLOOR_VISUAL_DETECTION_ID
#define FLOOR_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) count_region_a, int16_t __attribute__((unused)) count_region_b,
                               int16_t __attribute__((unused)) count_region_c, int16_t __attribute__((unused)) count_region_d,
                               int32_t total_count, int16_t __attribute__((unused)) extra)
{
  floor_count = total_count;
}


void our_avoider_init(void)
{
  srand(time(NULL));

  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
=======
void our_avoider_init(void)
{
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
>>>>>>> 4dbfe1347 (add orange avoidance control)
}


void our_avoider_periodic(void)
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> f6a3cd2fa (fix segmentation issues for very simple autonomous flight)
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
=======
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
>>>>>>> 3ff52f613 (fix segmentation issues for very simple autonomous flight)
    return;
  };

<<<<<<< HEAD
  int32_t floor_count_threshold = floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
=======

  guidance_h_set_body_vel(forward_velocity, 0);
>>>>>>> 3ff52f613 (fix segmentation issues for very simple autonomous flight)



<<<<<<< HEAD

  float speed_sp = forward_velocity;
  
  float anglewrtEnu = -20;
  float newx = +cos(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->x - sin(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->y;
  float newy = sin(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->x + cos(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->y;


  VERBOSE_PRINT("State: %d\n", navigation_state);
  VERBOSE_PRINT("current position LEFT RIGHT: %f\n", newx); //LEFT MINUS RIGHT PLUS
  VERBOSE_PRINT("current position UP DOWN ENU: %f\n", newy); //DOWN MINUS UP PLUS  
  

  // SOMETHING IS WRONG WITH THE POS VALIES THEY JUST DONT MAKE SENSE
  float heading = stateGetNedToBodyEulers_f()->psi;
  //FLOAT_ANGLE_NORMALIZE(heading);
  float heading_deg = DegOfRad(heading) - anglewrtEnu;
  if (heading_deg > 180){
    heading_deg = 360 - heading_deg;
  }
  VERBOSE_PRINT("heading normalized angle: %f\n", heading_deg);
  float headingReq = CalcDifferenceInHeading(newx, newy, 1, 0);
  float heading_rate = computePIDheading(heading_deg, headingReq);
  VERBOSE_PRINT("Heading REQ: %f heading diff: %f heading rate: %f\n", headingReq, headingReq-heading_deg, heading_rate);

  // zero up %%% 180 down %%% positive right %%% negative left

  // have a look at the angles they seem off by about 20 deg
  // the positions seem off as well
  // have the headng impact how much we turn when we come to an edge

  // heading pass trough true - to make sure we use optitrack


  switch (navigation_state){
    case SAFE:
      //make sure that the priority is good, might wanna change it a bit
      if (fabsf(newx) > 3.2 || fabsf(newy) > 3.2){
        if(newy >= 3.2 && ((0 <= heading_deg && heading_deg <= 90) || (-90 <= heading_deg && heading_deg <= 0))) {
          //top side
          navigation_state = OUT_OF_BOUNDS;
        } else if(newy <= -3.2 && ((90 <= heading_deg  && heading_deg <= 180) || (-180 <= heading_deg && heading_deg <= -90))) {
          //bottom side 
          navigation_state = OUT_OF_BOUNDS;
        } else if(newx <= -3.2 && -180 <= heading_deg && heading_deg <= 0) {
          //left side
          navigation_state = OUT_OF_BOUNDS;
        } else if(newx >= 3.2 && 0 <= heading_deg && heading_deg <= 180) {
          //right side
          navigation_state = OUT_OF_BOUNDS;
        } else {
          guidance_h_set_body_vel(speed_sp, 0);
          guidance_h_set_heading_rate(RadOfDeg(0));
        }
      } else if (newy >= 2.5 && ((0 <= heading_deg && heading_deg <= 90) || (-90 <= heading_deg && heading_deg <= 0))){
        // close to edge
        navigation_state = TOP_LINE;
      } else if (newx >= 2.5 && 0 <= heading_deg && heading_deg <= 180){
        // close to edge
        navigation_state = RIGHT_LINE;
      } else if (newy <= -2.5 && ((90 <= heading_deg  && heading_deg <= 180) || (-180 <= heading_deg && heading_deg <= -90))){
        // close to edge
        navigation_state = BOTTOM_LINE;
      } else if (newx <= -2.5 && -180 <= heading_deg && heading_deg <= 0){
        // close to edge
        navigation_state = LEFT_LINE;
      } else {
        // guidance_h_set_body_vel(speed_sp, 0);
        // guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(0));

        guidance_h_set_body_vel(speed_sp, 0);

        if(color_count_a + color_count_d != 0 && color_count_b + color_count_c != 0) {
          float norm_outer = color_count_a + color_count_d;
          float norm_inner = color_count_b + color_count_c;

          float diff_outer = color_count_a - color_count_d;
          float diff_inner = color_count_b - color_count_c;

          float heading_rate = k_outer * (diff_outer / norm_outer) + k_inner * (diff_inner / norm_inner);

          guidance_h_set_heading_rate(heading_rate);

        } else {
          guidance_h_set_heading_rate(0);
        }
      }
      break;
    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      // later will use the ref heading

      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      headingReq = CalcDifferenceInHeading(newx, newy, 1, 0);
      float heading_rate = computePIDheading(heading_deg, headingReq);
      VERBOSE_PRINT("heading_rate in the loop: %f", heading_rate);
      guidance_h_set_heading_rate(RadOfDeg(heading_rate));
      if(fabs(headingReq -heading_deg) < 10) {
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_body_vel(0, 0);
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;
    case TOP_LINE:
      if(newy >= 3 && ((0 <= heading_deg && heading_deg <= 90) || (-90 <= heading_deg && heading_deg <= 0))){
          navigation_state = OUT_OF_BOUNDS;
      } else if(heading_deg >= 0 && heading_deg <=90){
        //turn right
        guidance_h_set_body_vel(0.5 * speed_sp, 0.3 * speed_sp);
        guidance_h_set_heading_rate(RadOfDeg(90));
        navigation_state = SAFE;
      } else if (heading_deg <= 0 && heading_deg >= -90){
        //turn left
        guidance_h_set_body_vel(0.5 * speed_sp, -0.3 * speed_sp);
        guidance_h_set_heading_rate(-1.0 * RadOfDeg(90));
        navigation_state = SAFE;
      } else{
        guidance_h_set_body_vel(speed_sp, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        navigation_state = SAFE;
      }
      break;
    case RIGHT_LINE:
      if(newx >= 3 && 0 <= heading_deg && heading_deg <= 180){
          navigation_state = OUT_OF_BOUNDS;
      } else if(heading_deg >= 90 && heading_deg <=180){
        //turn right
        guidance_h_set_body_vel(0.5 * speed_sp, 0.3 * speed_sp);
        guidance_h_set_heading_rate(RadOfDeg(90));
        navigation_state = SAFE;
      } else if (heading_deg <= 90 && heading_deg >= 0){
        //turn left
        guidance_h_set_body_vel(0.5 * speed_sp, -0.3 * speed_sp);
        guidance_h_set_heading_rate(RadOfDeg(-1.0 * 90));
        navigation_state = SAFE;
      } else{
        guidance_h_set_body_vel(speed_sp, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        navigation_state = SAFE;
      }
      break;
    case BOTTOM_LINE:
      if(newy <= -3 && ((90 <= heading_deg  && heading_deg <= 180) || (-180 <= heading_deg && heading_deg <= -90))){
          navigation_state = OUT_OF_BOUNDS;
      } else if(heading_deg >= -180 && heading_deg <=-90){
        //turn right
        guidance_h_set_body_vel(0.5 * speed_sp, 0.3 * speed_sp);
        guidance_h_set_heading_rate(RadOfDeg(90));
        navigation_state = SAFE;
      } else if (heading_deg <= 180 && heading_deg >= 90){
        //turn left
        guidance_h_set_body_vel(0.5 * speed_sp, -0.3 * speed_sp);
        guidance_h_set_heading_rate(RadOfDeg(-1.0 * 90));
        navigation_state = SAFE;
      } else{
        guidance_h_set_body_vel(speed_sp, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        navigation_state = SAFE;
      }
      break;
    case LEFT_LINE:
      if(newx <= -3 && -180 <= heading_deg && heading_deg <= 0){
          navigation_state = OUT_OF_BOUNDS;
      } else if(heading_deg >= -90 && heading_deg <=0){
        //turn right
        guidance_h_set_body_vel(0.5 * speed_sp, 0.3 * speed_sp);
        guidance_h_set_heading_rate(RadOfDeg(90));
        navigation_state = SAFE;
      } else if (heading_deg <= -90 && heading_deg >= -180){
        //turn left
        guidance_h_set_body_vel(0.5 * speed_sp, -0.3 * speed_sp);
        guidance_h_set_heading_rate(RadOfDeg(-1.0 * 90));
        navigation_state = SAFE;
      } else{
        guidance_h_set_body_vel(speed_sp, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        navigation_state = SAFE;
      }
      break;
    default:
      break;
=======
    float diff_outer = color_count_a - color_count_d;
    float diff_inner = color_count_b - color_count_c;

    float heading_rate = k_outer * (diff_outer / norm_outer) + k_inner * (diff_inner / norm_inner);

    printf("heading rate: %.2f", heading_rate);

    guidance_h_set_heading_rate(heading_rate);
  } else {
    guidance_h_set_heading_rate(0.f);
>>>>>>> 3ff52f613 (fix segmentation issues for very simple autonomous flight)
  }

  return;
}



  float CalcDifferenceInHeading(float dronex, float droney, float goalx, float goaly) {
  
    float heading = 0;
    float dx = goalx - dronex;
    float dy = goaly - droney;
    heading = atan2(dx, dy);
      
    // Convert heading from radians to degrees
    heading = DegOfRad(heading);
    VERBOSE_PRINT("dx %f dy %f gx %f gy: %f\n", dronex, droney, goalx, goaly);

    VERBOSE_PRINT("inside the heading function heading: %f\n", heading);
    // Normalize heading to be within [-180, 180) degrees
    if (heading < -180.0) {
        heading += 360.0;
    } else if (heading >= 180.0) {
        heading -= 360.0;
    }
      
    return heading;
  }


// also look at guidance_pid.c
// also i just realised that the two new functions are not fully compatible
// either change first to only calc thewaypoint heading
// or change the pid to take the diff directly instead of computing it
float computePIDheading(float droneheading, float targetheading) {
  //error = difference in heading clockwise positive
  float error = 0;
  if (droneheading >= 0 && targetheading >= 0){
    error = targetheading - droneheading;
  } else if (droneheading >= 0 && targetheading <= 0){
    error = (360 + targetheading) - droneheading;
    if (error > 180){
      error = 360 - error;
    }
  } else if (droneheading <= 0 && targetheading >= 0){
      error = targetheading - droneheading;
      if (error > 180){
        error = error - 360;
      }
  } else if (droneheading <= 0 && targetheading <= 0){
      error = targetheading - droneheading;
  } else {
    return 0.0;
  }

  //VERBOSE_PRINT("IN THE FUNC droneheading: %f targetheading: %f\n", droneheading, targetheading);

  // will tune the pid controller once waypoints are here
  float KP_h = 1.0;     // Proportional gain
  float KI_h = 0.0;     // Integral gain
  float KD_h = 0.0;    // Derivative gain

  static double integral = 0.0;
  static double last_error = 0.0;

  double P = KP_h * error;

  integral += error;
  double I = KI_h * integral;

  double derivative = error - last_error;
  double D = KD_h * derivative;

  double output = P + I + D;

  last_error = error;
  // output should be the yaw rate
  return output;
=======
  if(!autopilot_in_flight()){
=======
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
>>>>>>> 0857f044b (fix segmentation issues for very simple autonomous flight)
    return;
  };


  guidance_h_set_body_vel(forward_velocity, 0);

  if(color_count_a + color_count_d != 0 && color_count_b + color_count_c != 0) {

    float norm_outer = color_count_a + color_count_d;
    float norm_inner = color_count_b + color_count_c;

    float diff_outer = color_count_a - color_count_d;
    float diff_inner = color_count_b - color_count_c;

    float heading_rate = k_outer * (diff_outer / norm_outer) + k_inner * (diff_inner / norm_inner);

    printf("heading rate: %.2f", heading_rate);

    guidance_h_set_heading_rate(heading_rate);
  } else {
    guidance_h_set_heading_rate(0.f);
  }

  return;
>>>>>>> 4dbfe1347 (add orange avoidance control)
}