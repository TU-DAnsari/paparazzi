/*
 * Copyright (C) 2016 Roland Meertens and Peng Lu
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/opencv_contour.h
 * Detects contours of an obstacle used in the autonomous drone racing.
 */

#ifndef OPENCV_OBSTACLE_DETECTION_CNN_H
#define OPENCV_OBSTACLE_DETECTION_CNN_H

#ifdef __cplusplus
extern "C" {
#endif

void find_contour(char *img, int width, int height, float *l_prob, float *c_prob, float *r_prob);

#ifdef __cplusplus
}
#endif

#endif