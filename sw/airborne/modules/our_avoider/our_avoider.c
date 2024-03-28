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
#include <math.h>

#include "modules/our_avoider/probability_map.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"
#include "../../boards/ardrone2.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

int sign(int num);
float CalcDifferenceInHeading(float dronex, float droney, float goalx, float goaly);
float computePIDheading(float droneheading, float targetheading);
float angleDifference(float angle1, float angle2);

enum navigation_state_t {
  SAFETURNING,
  SAFEFORWARD,
  SAFE,
  FRONTAL_OBSTACLE,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  TOP_LINE,
  RIGHT_LINE,
  BOTTOM_LINE,
  LEFT_LINE
};


// arena settings
float OUTER_BOUNDS = 3.2f;
float INNER_BOUNDS = 2.5f;
float SAFE_BOUNDS = 2.3f;
float anglewrtEnu = -35;

// avoidance velocity settings
float safe_xvel = .5f;
float unsafe_xvel = .4f;
float safe_yvel = .5f;
float unsafe_yvel = .5f;
float heading_turn_rate = 2.f;
float heading_search_rate = 0.5f;
float xvel;
float yvel;

float cornering_xvel = 0.2f;
float cornering_yvel = 0.5f;
float cornering_turn_rate = 1.6f;

// yaw rate proportional factors
float k_outer = .4f;
float k_inner = .6f;

float cnn_p_left = 0.0f;
float cnn_p_center = 0.0f;
float cnn_p_right = 0.0f;

// orange count in regions
int16_t color_count_a = 0;
int16_t color_count_b = 0; 
int16_t color_count_c = 0; 
int16_t color_count_d = 0; 

enum navigation_state_t navigation_state = SAFETURNING;


#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t count_region_a, int16_t count_region_b,
                               int16_t count_region_c, int16_t count_region_d,
                               int32_t __attribute__((unused)) quality, int16_t __attribute__((unused)) extra)
{
  color_count_a = count_region_a;
  color_count_b = count_region_b;
  color_count_c = count_region_c;
  color_count_d = count_region_d;
}

#ifndef CNN_OBS_ID
#define CNN_OBS_ID ABI_BROADCAST
#endif
static abi_event cnn_obs_ev;
static void cnn_obs_cb(uint8_t __attribute__((unused)) sender_id,
                       float prob_left,
                       float prob_center,
                       float prob_right)
{
  printf("recieved: l: %f, c: %f, r: %f", prob_left, prob_center, prob_right);
  
  cnn_p_left = prob_left;
  cnn_p_center = prob_center;
  cnn_p_right = prob_right;

  if (cnn_p_center > 0.6f){
      printf("Obstacle straight ahead! | ");
      if (cnn_p_left < 0.5f || cnn_p_right < 0.5f){
        if (cnn_p_left < cnn_p_right) {
          printf("Turn left!\n");
        }
        else {
          printf("Turn right!\n");
        }
      }

      else
        printf("Nowhere to go!, turn around!\n");
    }
    else printf("Fly straight!\n");
}


void our_avoider_init(void)
{
  srand(time(NULL));

  xvel = safe_xvel;
  yvel = safe_yvel;
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgCNN_OBS(CNN_OBS_ID, &cnn_obs_ev, cnn_obs_cb);

}


void our_avoider_periodic(void)
{
  
  Point wp = getGlobalDirection(); 

  printf("AAAAAAAAAAAAAAAAAAAAAAAAAA Waypoint X: %f\n", wp.x);
  printf("AAAAAAAAAAAAAAAAAAAAAAAAAA Waypoint Y: %f\n", wp.y);

  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    return;
  }

  float region_size = front_camera.output_size.w / 3 * front_camera.output_size.h / 4; 
  float slice_size = front_camera.output_size.w / 3 * front_camera.output_size.h; 
  float of = (color_count_a + color_count_b + color_count_c + color_count_d) / slice_size; // total orange fraction (of) in slice
  // float of_a = color_count_a / region_size; // total orange fraction (of) in given region
  // float of_b = color_count_b / region_size; 
  // float of_c = color_count_c / region_size;
  // float of_d = color_count_d / region_size;

  float of_a = 0;
  float of_b = 0;
  float of_c = 0;
  float of_d = 0;

  float newx = +cos(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->x - sin(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->y;
  float newy = sin(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->x + cos(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->y;


  // SOMETHING IS WRONG WITH THE POS VALIES THEY JUST DONT MAKE SENSE
  float heading = stateGetNedToBodyEulers_f()->psi;
  //FLOAT_ANGLE_NORMALIZE(heading);
  float heading_deg = DegOfRad(heading) - anglewrtEnu;
  if (heading_deg > 180){
    heading_deg = 360 - heading_deg;
  }
  
  // float headingReq = CalcDifferenceInHeading(newx, newy, 1, 0);
  // float heading_rate = computePIDheading(heading_deg, headingReq);
  

  VERBOSE_PRINT("\n");
  // VERBOSE_PRINT("of: %f\n", of); 
  // VERBOSE_PRINT("of a: %f\n", of_a); 
  // VERBOSE_PRINT("of b: %f\n", of_b);
  // VERBOSE_PRINT("of c: %f\n", of_c);
  // VERBOSE_PRINT("of d: %f\n", of_d);
  // VERBOSE_PRINT("\n");
  // VERBOSE_PRINT("p left: %f\n", cnn_p_left); 
  // VERBOSE_PRINT("p center: %f\n", cnn_p_center); 
  // VERBOSE_PRINT("p right: %f\n", cnn_p_right);
  VERBOSE_PRINT("position (x, y): %f, %f\n", newx, newy); 
  VERBOSE_PRINT("heading (deg): %f\n", heading_deg);

  // VERBOSE_PRINT("Heading REQ: %f heading diff: %f heading rate: %f\n", headingReq, headingReq-heading_deg, heading_rate);

  // zero up %%% 180 down %%% positive right %%% negative left

  // have a look at the angles they seem off by about 20 deg
  // the positions seem off as well
  // have the headng impact how much we turn when we come to an edge

  // heading pass trough true - to make sure we use optitrack
  float wp_heading;
  float wp_heading_rate;

  float tolerance = 10;

  switch (navigation_state){
    case SAFETURNING:
      // VERBOSE_PRINT("STATE: SAFETURNING\n");
      printf("Safe turning"); 
      wp_heading = CalcDifferenceInHeading(newx, newy, wp.x, wp.y);
      wp_heading_rate = computePIDheading(heading_deg, wp_heading);
      // VERBOSE_PRINT("heading_rate:  %f\n", wp_heading_rate);
      guidance_h_set_heading_rate(RadOfDeg(wp_heading_rate));
      guidance_h_set_body_vel(0, 0);
      // VERBOSE_PRINT("heading_differ:  %f\n", angleDifference(heading_deg, wp_heading));

      if (angleDifference(heading_deg, wp_heading) < tolerance) {
        // VERBOSE_PRINT("wp heading similar to heading");
        guidance_h_set_heading_rate(0);
        guidance_h_set_body_vel(xvel, 0);
        navigation_state = SAFEFORWARD;
      } 
      // VERBOSE_PRINT("wp_heading (deg): %f\n", wp_heading);
      break;
    case SAFEFORWARD:
      // VERBOSE_PRINT("STATE: SAFEFORWARD\n");
      // printf("Safe forward");
      guidance_h_set_heading_rate(0);
      guidance_h_set_body_vel(xvel, 0);
      wp_heading = CalcDifferenceInHeading(newx, newy, wp.x, wp.y);
        
      if (angleDifference(heading_deg, wp_heading) > tolerance){
        wp_heading_rate = computePIDheading(heading_deg, wp_heading);
        guidance_h_set_body_vel(0, 0);
        guidance_h_set_heading_rate(RadOfDeg(wp_heading_rate));
        navigation_state = SAFETURNING;
      }
      // VERBOSE_PRINT("wp_heading (deg): %f\n", wp_heading);
      break;
    case SAFE:
      VERBOSE_PRINT("STATE: SAFE\n");
      // check if drone is out of bounds
      navigation_state = SAFETURNING;
      break;


      if (fabsf(newx) > OUTER_BOUNDS || fabsf(newy) > OUTER_BOUNDS) {
        // if outside outer bounds, check if facing out of bounds
        if(newy >= OUTER_BOUNDS && ((0 <= heading_deg && heading_deg <= 90) || (-90 <= heading_deg && heading_deg <= 0))) {
          // top side
          navigation_state = OUT_OF_BOUNDS;
          break;
        } else if(newy <= -OUTER_BOUNDS && ((90 <= heading_deg  && heading_deg <= 180) || (-180 <= heading_deg && heading_deg <= -90))) {
          // bottom side 
          navigation_state = OUT_OF_BOUNDS;
          break;
        } else if(newx <= -OUTER_BOUNDS && -180 <= heading_deg && heading_deg <= 0) {
          // left side
          navigation_state = OUT_OF_BOUNDS;
          break;
        } else if(newx >= OUTER_BOUNDS && 0 <= heading_deg && heading_deg <= 180) {
          // right side
          navigation_state = OUT_OF_BOUNDS;
          break;
        }
      }

      // if outside inner bounds, turn inside
      if (newy >= INNER_BOUNDS && ((0 <= heading_deg && heading_deg <= 90) || (-90 <= heading_deg && heading_deg <= 0))) {
        // close to edge
        navigation_state = TOP_LINE;
        break;
      } else if (newx >= INNER_BOUNDS && 0 <= heading_deg && heading_deg <= 180) {
        // close to edge
        navigation_state = RIGHT_LINE;
        break;
      } else if (newy <= -INNER_BOUNDS && ((90 <= heading_deg  && heading_deg <= 180) || (-180 <= heading_deg && heading_deg <= -90))) {
        // close to edge
        navigation_state = BOTTOM_LINE;
        break;
      } else if (newx <= -INNER_BOUNDS && -180 <= heading_deg && heading_deg <= 0) {
        // close to edge
        navigation_state = LEFT_LINE;
        break;
      }

      if (fabsf(newx) < SAFE_BOUNDS && fabsf(newy) < SAFE_BOUNDS) {
        xvel = safe_xvel;
        yvel = safe_yvel;
      } else {
        xvel = unsafe_xvel;
        yvel = unsafe_yvel;
      }

      // if inside inner bounds, obstacle avoidance

      if((of_b > 0.4 && of_c > 0.4) || (of_b > 0.75f) || (of_b > 0.75f)) {
        navigation_state = FRONTAL_OBSTACLE;
        break;
      }

      // if(cnn_p_center > 0.8) {
      //   navigation_state = FRONTAL_OBSTACLE;
      //   break;
      // }

      if(of_b < 0.2 && of_c < 0.2) {
        float wp_forward_velocity = (1 - of + 0.2) * xvel;
        wp_heading = CalcDifferenceInHeading(newx, newy, wp.x, wp.y);
        wp_heading_rate = computePIDheading(heading, wp_heading);
        guidance_h_set_body_vel(wp_forward_velocity, sign(wp_heading_rate) * 0.5 * wp_forward_velocity);
        guidance_h_set_heading_rate(RadOfDeg(wp_heading_rate));
        break;
      }

      float or_forward_velocity = (1 - of + 0.2) * xvel;
      float or_lateral_velocity = (k_inner * (of_b - of_c) + k_outer * (of_a - of_d)) * yvel;
      float or_heading_rate =  (k_inner * (of_b - of_c) + k_outer * (of_a - of_d)) * heading_turn_rate;

      float ob_forward_velocity = (1 - (cnn_p_left + cnn_p_center + cnn_p_right) / 3);
      float ob_lateral_velocity = (cnn_p_left - cnn_p_right) * yvel;
      float ob_heading_rate = (cnn_p_left - cnn_p_right) * heading_turn_rate;

      guidance_h_set_body_vel(or_forward_velocity, or_lateral_velocity);
      guidance_h_set_heading_rate(or_heading_rate);
      break;


    case FRONTAL_OBSTACLE:
      VERBOSE_PRINT("STATE: FRONTAL_OBSTACLE\n");
      // stop
      guidance_h_set_body_vel(0, 0);
      guidance_h_set_heading_rate(heading_search_rate);

      if(of_b < 0.4f && of_c < 0.4f) {
        navigation_state = SAFE;
      }

      // if(cnn_p_center < 0.8f) {
      //   navigation_state = SAFE;
      // }

      break;


    case SEARCH_FOR_SAFE_HEADING:
      VERBOSE_PRINT("STATE: SEARCH_FOR_SAFE_HEADING\n");
      float headingReq = CalcDifferenceInHeading(newx, newy, wp.x, wp.y);
      float heading_rate = computePIDheading(heading_deg, headingReq);
      // VERBOSE_PRINT("heading_rate in the loop: %f", heading_rate);
      guidance_h_set_heading_rate(RadOfDeg(heading_rate));

      if(fabs(headingReq - heading_deg) < 20) {
        navigation_state = SAFE;
      }
      break;


    case OUT_OF_BOUNDS:
      VERBOSE_PRINT("STATE: OUT_OF_BOUNDS\n");
      // stop
      guidance_h_set_body_vel(0, 0);
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;


    case TOP_LINE:
      VERBOSE_PRINT("STATE: TOP_LINE\n");
      if(newy >= OUTER_BOUNDS && ((0 <= heading_deg && heading_deg <= 90) || (-90 <= heading_deg && heading_deg <= 0))) {
          navigation_state = OUT_OF_BOUNDS;
      } else if(heading_deg >= 0 && heading_deg <=110) {
        //turn right
        guidance_h_set_body_vel(cornering_xvel, cornering_yvel);
        guidance_h_set_heading_rate(cornering_turn_rate);
        navigation_state = SAFE;
      } else if (heading_deg <= 0 && heading_deg >= -110) {
        //turn left
        guidance_h_set_body_vel(cornering_xvel, -cornering_yvel);
        guidance_h_set_heading_rate(-cornering_turn_rate);
        navigation_state = SAFE;
      } else {
        guidance_h_set_body_vel(cornering_xvel, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        navigation_state = SAFE;
      }
      break;


    case RIGHT_LINE:
      VERBOSE_PRINT("STATE: RIGHT_LINE\n");
      if(newx >= OUTER_BOUNDS && 0 <= heading_deg && heading_deg <= 180) {
          navigation_state = OUT_OF_BOUNDS;
      } else if(heading_deg >= 90 && heading_deg <=180) {
        //turn right
        guidance_h_set_body_vel(cornering_xvel, cornering_yvel);
        guidance_h_set_heading_rate(cornering_turn_rate);
        navigation_state = SAFE;
      } else if (heading_deg <= 90 && heading_deg >= 0) {
        //turn left
        guidance_h_set_body_vel(cornering_xvel, -cornering_yvel);
        guidance_h_set_heading_rate(-cornering_turn_rate);
        navigation_state = SAFE;
      } else {
        guidance_h_set_body_vel(cornering_xvel, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        navigation_state = SAFE;
      }
      break;


    case BOTTOM_LINE:
      VERBOSE_PRINT("STATE: BOTTOM_LINE\n");
      if(newy <= -OUTER_BOUNDS && ((90 <= heading_deg  && heading_deg <= 180) || (-180 <= heading_deg && heading_deg <= -90))) {
          navigation_state = OUT_OF_BOUNDS;
      } else if(heading_deg >= -180 && heading_deg <=-70) {
        //turn right
        guidance_h_set_body_vel(cornering_xvel, cornering_yvel);
        guidance_h_set_heading_rate(cornering_turn_rate);
        navigation_state = SAFE;
      } else if (heading_deg <= 180 && heading_deg >= 70) {
        //turn left
        guidance_h_set_body_vel(cornering_xvel, -cornering_yvel);
        guidance_h_set_heading_rate(-cornering_turn_rate);
        navigation_state = SAFE;
      } else {
        guidance_h_set_body_vel(cornering_xvel, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        navigation_state = SAFE;
      }
      break;


    case LEFT_LINE:
      VERBOSE_PRINT("STATE: LEFT_LINE\n");
      if(newx <= -OUTER_BOUNDS && -180 <= heading_deg && heading_deg <= 0) {
          navigation_state = OUT_OF_BOUNDS;
      } else if(heading_deg >= -90 && heading_deg <=0) {
        //turn right
        guidance_h_set_body_vel(cornering_xvel, cornering_yvel);
        guidance_h_set_heading_rate(cornering_turn_rate);
        navigation_state = SAFE;
      } else if (heading_deg <= -90 && heading_deg >= -180) {
        //turn left
        guidance_h_set_body_vel(cornering_xvel, -cornering_yvel);
        guidance_h_set_heading_rate(-cornering_turn_rate);
        navigation_state = SAFE;
      } else {
        guidance_h_set_body_vel(cornering_xvel, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        navigation_state = SAFE;
      }
      break;


    default:
      break;
  }
  return;
}

int sign(int num) {
  if(num > 0) {
    return 1;
  } 
  return -1;
}


float CalcDifferenceInHeading(float dronex, float droney, float goalx, float goaly) {

  float heading = 0;
  float dx = goalx - dronex;
  float dy = goaly - droney;
  heading = atan2(dx, dy);
    
  // Convert heading from radians to degrees
  heading = DegOfRad(heading);
  // VERBOSE_PRINT("dx %f dy %f gx %f gy: %f\n", dronex, droney, goalx, goaly);

  // VERBOSE_PRINT("inside the heading function heading: %f\n", heading);
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
  float error;
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
  // droneheading = fmodf(droneheading, 360.0f);
  // targetheading = fmodf(targetheading, 360.0f);

  // // Compute the error
  // float error = targetheading - droneheading;

  // // Normalize the error to be within the range [-180, 180)
  // if (error < -180.0f) {
  //     error += 360.0f;
  // } else if (error >= 180.0f) {
  //     error -= 360.0f;
  // }
  //VERBOSE_PRINT("IN THE FUNC droneheading: %f targetheading: %f\n", droneheading, targetheading);

  // will tune the pid controller once waypoints are here
  float KP_h = 0.5;     // Proportional gain
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
}

float angleDifference(float angle1, float angle2){
  float difference = fabs(angle1 - angle2);

  if(difference > 180.0f){
    difference = 360.0f - difference;
  }
  return difference;
}
