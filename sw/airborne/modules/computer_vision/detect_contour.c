/*
 * Copyright (C) Roland Meertens and Peng Lu
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/detect_contour.c"
 * @author Roland Meertens and Peng Lu
 *
 */
#include "modules/core/abi.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/detect_contour.h"
#include "modules/computer_vision/opencv_contour.h"

#ifndef DETECT_CONTOUR_FPS
#define DETECT_CONTOUR_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(DETECT_CONTOUR_FPS)

float combined_model = 1;

// Function
struct image_t *contour_func(struct image_t *img, uint8_t camera_id);
struct image_t *contour_func(struct image_t *img, uint8_t camera_id)
{

  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
    float l_prob = 0.0f;
    float c_prob = 0.0f;
    float r_prob = 0.0f;
  
    find_contour((char *) img->buf, img->w, img->h, &l_prob, &c_prob, &r_prob, combined_model);
    //printf("sending: l: %f, c: %f, r: %f", l_prob, c_prob, r_prob);
  

    AbiSendMsgCNN_OBS(CNN_OBS_CALC_ID, l_prob, c_prob, r_prob);
  
  }
  return img;
}

void detect_contour_init(void)
{
  printf("detect contour init");
  cv_add_to_device(&DETECT_CONTOUR_CAMERA, contour_func, DETECT_CONTOUR_FPS, 0);
  // in the mavlab, bright
  cont_thres.lower_y = 16;  cont_thres.lower_u = 135; cont_thres.lower_v = 80;
  cont_thres.upper_y = 100; cont_thres.upper_u = 175; cont_thres.upper_v = 165;


  combined_model = 1;
  //
  // for cyberzoo: Y:12-95, U:129-161, V:80-165, turn white.
  //int y1=16;  int u1=129; int v1=80; % cyberzoo, dark
  //int y2=100; int u2=161; int v2=165;
}

