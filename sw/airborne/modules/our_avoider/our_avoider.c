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

enum navigation_state_t {
  SAFE,
  FRONTAL_OBSTACLE,
  SEARCH_HEADING,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  TOP_LINE,
  RIGHT_LINE,
  BOTTOM_LINE,
  LEFT_LINE
};

// module settings
int cnn_enabled = 1;

// arena settings
float OUTER_BOUNDS = 3.2f;
float INNER_BOUNDS = 5.f;
float SAFE_BOUNDS = 2.3f;
float anglewrtEnu = -35;

// avoidance velocity settings
// float safe_xvel = .5f;
float unsafe_xvel = .5f;
// float safe_yvel = .5f;
float unsafe_yvel = .5f;
float heading_turn_rate = 2.f;
float heading_search_rate = 0.5f;

float cnn_w_avg = 0.f;
float cnn_sum_r = 0.f;
float cnn_sum_l = 0.f;
float spx = 0.0f;
float spy = 0.0f;

// Global settings - changable in gcs
float slow_mode_safe_xvel = .4f;
float slow_mode_safe_yvel = .4f;
float fast_mode_safe_xvel = .4f;
float fast_mode_safe_yvel = .4f;


float xvel;
float yvel;

float cornering_xvel = 0.1f;
float cornering_yvel = 0.5f;
float cornering_turn_rate = 1.6f;

// yaw rate proportional factors
float k_outer = .4f;
float k_inner = .6f;

// OA STUFF
  float region_size = 0.f; 
  float slice_size = 0.f; 


  float of = 0.f; // total orange fraction (of) in slice
  float of_a = 0.f; // total orange fraction (of) in given region
  float of_b = 0.f; 
  float of_c = 0.f;
  float of_d = 0.f;


// CNN STUFF

float cnn_p_left = 0.0f;
float cnn_p_center = 0.0f;
float cnn_p_right = 0.0f;

#define MOVING_AVERAGE_SIZE 5

float cnn_n_prev_prob_left[MOVING_AVERAGE_SIZE] = {0.0f};
float cnn_n_prev_prob_center[MOVING_AVERAGE_SIZE] = {0.0f};
float cnn_n_prev_prob_right[MOVING_AVERAGE_SIZE] = {0.0f};

// orange count in regions
int16_t color_count_a = 0;
int16_t color_count_b = 0; 
int16_t color_count_c = 0; 
int16_t color_count_d = 0; 

enum navigation_state_t navigation_state = SAFE;


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

  of = (color_count_a + color_count_b + color_count_c + color_count_d) / slice_size;
  of_a = color_count_a / region_size;
  of_b = color_count_b / region_size; 
  of_c = color_count_c / region_size;
  of_d = color_count_d / region_size;
}

// Assumes length MOVING_AVERAGE_SIZE
void move_data_back(float f_array[MOVING_AVERAGE_SIZE]){
  for (int i = 0; i < MOVING_AVERAGE_SIZE-1; i++){
    f_array[i+1] = f_array[i];
  }
}

// Assumes length MOVING_AVERAGE_SIZE
float array_weighted_moving_average(float f_array[MOVING_AVERAGE_SIZE]){
  float wma = 0.0f;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++){
      wma += f_array[i]*(MOVING_AVERAGE_SIZE-i);
  }
  wma /= (MOVING_AVERAGE_SIZE*(MOVING_AVERAGE_SIZE+1)/2);
  return wma;
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
  

  move_data_back(cnn_n_prev_prob_left);
  cnn_n_prev_prob_left[0] = prob_left;
  
  move_data_back(cnn_n_prev_prob_center);
  cnn_n_prev_prob_center[0] = prob_center;

  move_data_back(cnn_n_prev_prob_right);
  cnn_n_prev_prob_right[0] = prob_right;

  cnn_p_left = array_weighted_moving_average(cnn_n_prev_prob_left);
  cnn_p_center = array_weighted_moving_average(cnn_n_prev_prob_center);
  cnn_p_right = array_weighted_moving_average(cnn_n_prev_prob_right);

  printf("recieved: l: %f, c: %f, r: %f", cnn_p_left, cnn_p_center, cnn_p_right);
  // const float testarr[5] = {0.0f,0.0f,0.0f,0.0f,1.0f};
  // printf("Moving average: %f", array_weighted_moving_average(testarr));

//   if (cnn_p_center > 0.6f){
//       printf("Obstacle straight ahead! | ");
//       if (cnn_p_left < 0.5f || cnn_p_right < 0.5f){
//         if (cnn_p_left < cnn_p_right) {
//           printf("Turn left!\n");
//         }
//         else {
//           printf("Turn right!\n");
//         }
//       }

//       else
//         printf("Nowhere to go!, turn around!\n");
//     }
//     else printf("Fly straight!\n");
}


void our_avoider_init(void)
{
  srand(time(NULL));

  region_size = front_camera.output_size.w / 3 * front_camera.output_size.h / 4; 
  slice_size = front_camera.output_size.w / 3 * front_camera.output_size.h; 

  xvel = fast_mode_safe_xvel;
  yvel = fast_mode_safe_yvel;
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgCNN_OBS(CNN_OBS_ID, &cnn_obs_ev, cnn_obs_cb);

}


void our_avoider_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SAFE;
    return;
  }

  //printf("slow mode enabled: %d", cnn_enabled);

  float newx = +cos(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->x - sin(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->y;
  float newy = sin(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->x + cos(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->y;


  // SOMETHING IS WRONG WITH THE POS VALIES THEY JUST DONT MAKE SENSE
  float heading = stateGetNedToBodyEulers_f()->psi;
  //FLOAT_ANGLE_NORMALIZE(heading);
  float heading_deg = DegOfRad(heading) - anglewrtEnu;
  if (heading_deg > 180){
    heading_deg = heading_deg - 360;
  }

  printf("SLOW MODE: %d\n", cnn_enabled);
  
  // float headingReq = CalcDifferenceInHeading(newx, newy, 1, 0);
  // float heading_rate = computePIDheading(heading_deg, headingReq);
  

  // VERBOSE_PRINT("\n");
  // VERBOSE_PRINT("of: %f\n", of); 
  // VERBOSE_PRINT("of a: %f\n", of_a); 
  // VERBOSE_PRINT("of b: %f\n", of_b);
  // VERBOSE_PRINT("of c: %f\n", of_c);
  // VERBOSE_PRINT("of d: %f\n", of_d);
  // VERBOSE_PRINT("\n");
  // VERBOSE_PRINT("p left: %f\n", cnn_p_left); 
  // VERBOSE_PRINT("p center: %f\n", cnn_p_center); 
  // VERBOSE_PRINT("p right: %f\n", cnn_p_right);
  // VERBOSE_PRINT("position (x, y): %f, %f\n", newx, newy); 
  // VERBOSE_PRINT("heading (deg): %f\n", heading_deg);

  // VERBOSE_PRINT("Heading REQ: %f heading diff: %f heading rate: %f\n", headingReq, headingReq-heading_deg, heading_rate);

  // zero up %%% 180 down %%% positive right %%% negative left

  // have a look at the angles they seem off by about 20 deg
  // the positions seem off as well
  // have the headng impact how much we turn when we come to an edge

  // heading pass trough true - to make sure we use optitrack


  switch (navigation_state){
    case SAFE:
      VERBOSE_PRINT("STATE: SAFE\n");
      // check if drone is out of bounds
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
        if (cnn_enabled == 1){
          xvel = slow_mode_safe_xvel;
          yvel = slow_mode_safe_yvel;
        }
        else {
          xvel = fast_mode_safe_xvel;
          yvel = fast_mode_safe_yvel;
        }
      } else {
        xvel = unsafe_xvel;
        yvel = unsafe_yvel;
      }

      // if inside inner bounds, obstacle avoidance

    
      if(cnn_enabled == 0) {
        if((of_b > 0.4 && of_c > 0.4) || (of_b > 0.75f) || (of_b > 0.75f)) {
          navigation_state = FRONTAL_OBSTACLE;
          break;
        }
        float or_forward_velocity = (1 - of + 0.2) * xvel;
        float or_lateral_velocity = (k_inner * (of_b - of_c) + k_outer * (of_a - of_d)) * yvel;
        float or_heading_rate =  (k_inner * (of_b - of_c) + k_outer * (of_a - of_d)) * heading_turn_rate;

        guidance_h_set_body_vel(or_forward_velocity, or_lateral_velocity);
        guidance_h_set_heading_rate(or_heading_rate);
      }
      

      if (cnn_enabled == 1) {
        if((cnn_p_left > 0.8 && cnn_p_center > 0.6) || cnn_p_center > 0.75  || (cnn_p_right > 0.8 && cnn_p_center > 0.6)) {
          navigation_state = FRONTAL_OBSTACLE;
          spx = newx;
          spy = newy;
          break;  
        }

        if((of_b > 0.4 && of_c > 0.4) || (of_b > 0.75f) || (of_b > 0.75f)) {
          navigation_state = FRONTAL_OBSTACLE;
          break;
        }

        cnn_w_avg = (0.25 * cnn_p_left + 0.5 * cnn_p_center + 0.25 * cnn_p_right) / 3;

        float or_forward_velocity = (1 - of) * xvel;
        float or_lateral_velocity = (k_inner * (of_b - of_c) + k_outer * (of_a - of_d)) * yvel;
        float or_heading_rate =  (k_inner * (of_b - of_c) + k_outer * (of_a - of_d)) * heading_turn_rate;

        float ob_forward_velocity = (1 - cnn_w_avg) * xvel;
        float ob_lateral_velocity = 0.4 * (cnn_p_left - cnn_p_right) * yvel;
        float ob_heading_reate = 0.3 * (cnn_p_left - cnn_p_right) * heading_turn_rate;

        float applied_forward_velocity = (or_forward_velocity + or_lateral_velocity) / 2;
        float applied_lateral_velocity = (or_lateral_velocity + ob_lateral_velocity) / 2;
        float applied_heading_rate = (or_heading_rate + ob_heading_reate) / 2;

        cnn_sum_r += cnn_p_right;
        cnn_sum_l += cnn_p_left;

        guidance_h_set_body_vel(ob_forward_velocity, ob_lateral_velocity);
        guidance_h_set_heading_rate(ob_heading_reate);
      } 
      break;


    case FRONTAL_OBSTACLE:
      VERBOSE_PRINT("STATE: FRONTAL_OBSTACLE\n");
      // stop
      guidance_h_set_heading_rate(0.f);
      
      float dvx = spx - newx;
      float dvy = spy - newy;

      float drone_dvx = +cos(RadOfDeg(90 - heading_deg)) * dvx + sin(RadOfDeg(90 - heading_deg)) * dvy;
      float drone_dvy = -sin(RadOfDeg(90 - heading_deg)) * dvx + cos(RadOfDeg(90 - heading_deg)) * dvy;

      float dist = sqrt(powf(spx - newx, 2) + powf(spy - newy, 2));

      guidance_h_set_body_vel(0.5 * drone_dvx, -0.5 * drone_dvy);
      
      printf("DISTAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAANCE: %f\n", dist);

      if(dist < 0.15) {
        guidance_h_set_body_vel(0.f, 0.f);
        navigation_state = SEARCH_HEADING;
      }
      break;


    case SEARCH_HEADING:
      guidance_h_set_heading_rate(sign(cnn_sum_l - cnn_sum_r) * heading_search_rate);

      if(cnn_enabled == 0) {
        if(of_b < 0.4f && of_c < 0.4f) {
          navigation_state = SAFE;
        }
      }

      if(cnn_enabled == 1) {
        if(cnn_p_center < 0.5f) {
          navigation_state = SAFE;
          cnn_sum_r = 0.f;
          cnn_sum_l = 0.f;
        }
      }
      break;


    case SEARCH_FOR_SAFE_HEADING:
      VERBOSE_PRINT("STATE: SEARCH_FOR_SAFE_HEADING\n");
      float headingReq = CalcDifferenceInHeading(newx, newy, 1, 0);
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