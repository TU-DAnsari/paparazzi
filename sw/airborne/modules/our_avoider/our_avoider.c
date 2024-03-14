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

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
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


void our_avoider_init(void)
{
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}


void our_avoider_periodic(void)
{
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
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
}