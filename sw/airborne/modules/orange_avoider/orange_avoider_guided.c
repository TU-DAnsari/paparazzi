/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>
#include <math.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);
float CalcDifferenceInHeading(float dronex, float droney, float goalx, float goaly);
float computePIDheading(float droneheading, float targetheading);

enum navigation_state_t {
  SAFE,
  CLOSE_TO_OBSTICLE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
  TOP_LINE,
  RIGHT_LINE,
  BOTTOM_LINE,
  LEFT_LINE
};


// define settings
float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.

const int16_t max_trajectory_confidence = 10;  // number of consecutive negative object detections to be sure we are obstacle free

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  VERBOSE_PRINT("Color_count: %d, threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);
  
  //VERBOSE_PRINT("current position LEFT RIGHT: %f\n", stateGetPositionEnu_f() ->x); //LEFT MINUS RIGHT PLUS
  //VERBOSE_PRINT("current position UP DOWN ENU: %f\n", stateGetPositionEnu_f() ->y); //DOWN MINUS UP PLUS  
  float anglewrtEnu = -20;
  float newx = +cos(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->x - sin(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->y;
  float newy = sin(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->x + cos(RadOfDeg(anglewrtEnu))*stateGetPositionEnu_f() ->y;

  VERBOSE_PRINT("current position LEFT RIGHT: %f\n", newx); //LEFT MINUS RIGHT PLUS
  VERBOSE_PRINT("current position UP DOWN ENU: %f\n", newy); //DOWN MINUS UP PLUS  

  // SOMETHING IS WRONG WITH THE POS VALIES THEY JUST DONT MAKE SENSE
  float heading = stateGetNedToBodyEulers_f()->psi;
  FLOAT_ANGLE_NORMALIZE(heading);
  float heading_deg = DegOfRad(heading) - anglewrtEnu; //+25 since north is at an angle with cyberzoo
  VERBOSE_PRINT("heading normalized? angle: %f\n", heading_deg);
  // zero up %%% 180 down %%% positive right %%% negative left

  // have a look at the angles they seem off by about 20 deg
  // the positions seem off as well
  // have the headng impact how much we turn when we come to an edge

  // heading pass trough true - to make sure we use optitrack

  switch (navigation_state){
    case SAFE:
      //make sure that the priority is good, might wanna change it a bit
      if (fabsf(newx) > 3 || fabsf(newy) > 3){
        if(newy >= 3 && ((0 <= heading_deg && heading_deg <= 90) || (-90 <= heading_deg && heading_deg <= 0))){
          //top side
          navigation_state = OUT_OF_BOUNDS;
        } else if(newy <= -3 && ((90 <= heading_deg  && heading_deg <= 180) || (-180 <= heading_deg && heading_deg <= -90))){
          //bottom side 
          navigation_state = OUT_OF_BOUNDS;
        } else if(newx <= -3 && -180 <= heading_deg && heading_deg <= 0){
          //left side
          navigation_state = OUT_OF_BOUNDS;
        } else if(newx >= 3 && 0 <= heading_deg && heading_deg <= 180){
          //right side
          navigation_state = OUT_OF_BOUNDS;
        } else{
          guidance_h_set_body_vel(speed_sp, 0);
          guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(0));
        }
      } else if (obstacle_free_confidence < 9 && color_count > color_count_threshold){
        navigation_state = CLOSE_TO_OBSTICLE;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
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
        guidance_h_set_body_vel(speed_sp, 0);
        guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(0));
      }

      break;
    case CLOSE_TO_OBSTICLE:
      // changes heading randomly for now
      // later will use the ref heading
      guidance_h_set_body_vel(0.5 * speed_sp, 0);
      guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(45));

      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state = REENTER_ARENA;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else if (color_count < 0.5 * color_count_threshold){
        navigation_state = SAFE;
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      // later will use the ref heading
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 3){
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(20));

      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      // could also use position and attitude to return to the center
      // would be nice to make a function that takes drone pos and waypoint pos to calc ref heading
      // force floor center to opposite side of turn to xhead back into arena
      if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }
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
        guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(0));
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
        guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(0));
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
        guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(0));
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
        guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(0));
        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  return false;
}

float CalcDifferenceInHeading(float dronex, float droney, float goalx, float goaly){
  // calc the required change in heading
  // goal is from the path planning
  // difference will be used for the heading rate command

  // heading
  // zero up %%% 180 down %%% positive right %%% negative left
  //positions
  //LEFT MINUS RIGHT PLUS
  //DOWN MINUS UP PLUS  
  // x y
  // so if + + heading 0-90
  // if + - heading 90-180
  // if - - heading -90 - (-180)
  // if - + heading 0 - (-90)
  float heading = 0;
  float diffx = goalx - dronex;
  float diffy = goaly - droney;
  if (diffx > 0 && diffy > 0){
    heading = atan(diffx / diffy);
  } else if (diffx > 0 && diffy < 0){
    heading =  90 + atan((-diffy) / diffx);
  } else if (diffx < 0 && diffy < 0){
    heading = -180 + atan((-diffx)/(-diffy));
  } else if (diffx < 0 && diffy > 0){
    heading = -90 + atan(diffy / (-diffx));
  }
  return heading;
}

// also look at guidance_pid.c
// also i just realised that the two new functions are not fully compatible
// either change first to only calc thewaypoint heading
// or change the pid to take the diff directly instead of computing it
float computePIDheading(float droneheading, float targetheading){
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

  // will tune the pid controller once waypoints are here
  float KP_h = 1.0;     // Proportional gain
  float KI_h = 0.1;     // Integral gain
  float KD_h = 0.01;    // Derivative gain

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

