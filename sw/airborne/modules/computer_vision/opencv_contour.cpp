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
 * @file "modules/computer_vision/opencv_contour.cpp"
 * @author Roland Meertens and Peng Lu
 *
 */
#include <time.h>

#include "opencv_contour.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/imgcodecs.hpp>

#include "opencv_image_functions.h"

#include <iostream>

// #include "object_classification_cnn.h"
// #include "MobileTinyv1.h"
// #include "MobileTinyv2.h"
// #include "MobileTinyv3.h"
// #include "MobileNetTinyTrained.h"
// #include "MobileNetTinyv5Trained.h"
// #include "MobileNetTinyv6Trained.h"
#include "MobileNetTinyv6TrainedSim.h"

#include "lib/vision/image.h"
#include <sys/time.h>

using namespace cv;
using namespace std;

struct contour_estimation cont_est;
struct contour_threshold cont_thres;

RNG rng(12345);

#define EULER_NUMBER_F 2.71828182846

float sigmoidf(float n) {
    return (1 / (1 + powf(EULER_NUMBER_F, -n)));
}

// YUV in opencv convert to YUV on Bebop
void yuv_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
//Turn the opencv RGB colored image back in a YUV colored image for the drone
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      // Extract pixel color from image
      cv::Vec3b &c = image.at<cv::Vec3b>(row, col);

      // Set image buffer values
      int i = row * width + col;
      img[2 * i + 1] = c[0]; // y;
      img[2 * i] = col % 2 ? c[1] : c[2]; // u or v
    }
  }
}

void uyvy_opencv_to_yuv_opencv(Mat image, Mat image_in, int width, int height)
{
//Turn the opencv RGB colored image back in a YUV colored image for the drone
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      // Extract pixel color from image
      cv::Vec3b c = image_in.at<cv::Vec3b>(row, col);
      cv::Vec3b c_m1 = image_in.at<cv::Vec3b>(row, col);
      cv::Vec3b c_p1 = image_in.at<cv::Vec3b>(row, col);
      if (col > 0) {
        c_m1 = image_in.at<cv::Vec3b>(row, col - 1);
      }
      if (col < width) {
        c_p1 = image_in.at<cv::Vec3b>(row, col + 1);
      }
      image.at<cv::Vec3b>(row, col)[0] = c[1] ;
      image.at<cv::Vec3b>(row, col)[1] = col % 2 ? c[0] : c_m1[0];
      image.at<cv::Vec3b>(row, col)[2] = col % 2 ? c_p1[0] : c[0];

    }
  }
}

void find_contour(char *img, int width, int height, float *l_prob, float *c_prob, float *r_prob)
{


  struct timeval start_pre, end_pre;
  
  // Get the start time
  gettimeofday(&start_pre, NULL);


  // Transform image buffer img into an OpenCV YUV422 Mat
  Mat M (height, width, CV_8UC2, img);
  // Convert to OpenCV BGR
  Mat image(height, width, CV_8UC3);
  Mat image_rotated;
  cvtColor (M, image, CV_YUV2BGR_Y422);

  // cvtColor (image, grey, COLOR_BGR2GRAY);
  

  rotate(image, image_rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
  // 0, 35, 170, 205)))\n",
  //   "    img2_d = np.asarray(depth.crop((170, 35, 340, 205)))\n",
  //   "    img3_d = np.asarray(depth.crop((340, 35, 510, 205
  Mat cropped_image_left = image_rotated(Range(35,205), Range(0,170));
  Mat cropped_image_mid = image_rotated(Range(35,205), Range(170,340));
  Mat cropped_image_right = image_rotated(Range(35,205), Range(340,510));
  
  

  //cropped_image_mid.setTo(cv::Scalar(100,20,10));

  // if (!imwrite("/tmp/paparazzi/rgbrot.png", cropped_image)) {
  //     cout << "Failed to save the image" << endl;
  // }

  

  //cropped_image.convertTo(cropped_image, CV_32FC3, 1/255.0);


  // Creating the tensor array
  float tensor_input[3][3][85][85];
  
  // memset(&tensor_input[0][0][0][0], 1, sizeof(float) * 3*3*85*85);

  // float copy_tensor[3][85][85];

  
  int nRows = cropped_image_left.rows;
  int nCols = cropped_image_left.cols;
  //printf("Ncols: %d", nCols);
  int i,j;
  uchar *pl, *pm, *pr;
  for( i = 0; i < nRows; i += 2)
  {
      pl = cropped_image_left.ptr<uchar>(i);
      pm = cropped_image_mid.ptr<uchar>(i);
      pr = cropped_image_right.ptr<uchar>(i);

      for ( j = 0; j < nCols; j += 2)
      {
          tensor_input[0][2][j/2][i/2] = (float)pl[(j*3)] /255.0; // Check i, j order
          tensor_input[0][1][j/2][i/2] = (float)pl[(j*3)+1] /255.0;
          tensor_input[0][0][j/2][i/2] = (float)pl[(j*3)+2] /255.0;

          tensor_input[1][2][j/2][i/2] = (float)pm[(j*3)] /255.0; // Check i, j order
          //printf("%f,", (float)pm[j] /255.0);
          tensor_input[1][1][j/2][i/2] = (float)pm[(j*3)+1] /255.0;
          tensor_input[1][0][j/2][i/2] = (float)pm[(j*3)+2] /255.0;

          tensor_input[2][2][j/2][i/2] = (float)pr[(j*3)] /255.0; // Check i, j order
          tensor_input[2][1][j/2][i/2] = (float)pr[(j*3)+1] /255.0;
          tensor_input[2][0][j/2][i/2] = (float)pr[(j*3)+2] /255.0;
      }
      //printf("endj: %d", j);
  }

  //printf("data: %f", copy_tensor[0][0][0]);

  //printf("size: %d",sizeof (copy_tensor));

  // memcpy(&tensor_input[0][0][0][0], &copy_tensor, sizeof (copy_tensor));
  // memcpy(&tensor_input[1][0][0][0], &copy_tensor, sizeof (copy_tensor));
  // memcpy(&tensor_input[2][0][0][0], &copy_tensor, sizeof (copy_tensor));
  
  // Get the end time
  gettimeofday(&end_pre, NULL);
  
  // Calculate the elapsed time in microseconds
  long elapsed_pre = (end_pre.tv_sec - start_pre.tv_sec) * 1000000 + end_pre.tv_usec - start_pre.tv_usec;
  //printf("Elapsed time, image moving: %ld microseconds\n", elapsed_pre);

  //cvtColor(M, M, CV_YUV2GRAY_Y422);
  float tensor_output[3][1];
  //memset(tensor_output, 0, sizeof tensor_output);

  struct timeval start, end;
  
  // Get the start time
  gettimeofday(&start, NULL);
  
//   printf("in: %f", tensor_input[0][0][0][0]);
//   printf("in: %f", tensor_input[2][2][84][84]);

//   for (int i = 0; i < 3; ++i) {
//     for (int j = 0; j < 3; ++j) {
//         for (int k = 0; k < 85; ++k) {
//             for (int l = 0; l < 85; ++l) {
//                 if (tensor_input[i][j][k][l] != 0.0f) printf("Bruh %f", tensor_input[i][j][k][l]);
//             }
//         }
//     }
// }

  entry(tensor_input, tensor_output);
  //printf("daatain: %f, %f", tensor_input[0][0][0][0], tensor_input[1][0][0][0]);
  
  // printf("1: %f, %f\n", tensor_output[0][0], tensor_output[0][1]);
  
  // printf("i0: %f\n", tensor_input[1][0][0][0]);
  // printf("i1: %f\n", tensor_input[1][1][0][0]);
  // printf("i2: %f\n", tensor_input[1][2][0][0]);
  
  
  // printf("i0: %f\n", tensor_input[1][0][50][50]);
  // printf("i1: %f\n", tensor_input[1][1][50][50]);
  // printf("i2: %f\n", tensor_input[1][2][50][50]);
  
  // printf("0: %f\n", tensor_output[0][0]);
  // printf("1: %f\n", tensor_output[1][0]);
  // printf("2: %f\n", tensor_output[2][0]);
  // exit(0);
  // printf("3: %f, %f\n", tensor_output[2][0], tensor_output[2][1]);
  
  // float leftobstacle_prob = sigmoidf(tensor_output[0][0]);
  // float centerobstacle_prob = sigmoidf(tensor_output[1][0]);
  // float rightobstacle_prob = sigmoidf(tensor_output[2][0]);
  
  *l_prob = tensor_output[0][0];
  *c_prob = tensor_output[1][0];
  *r_prob = tensor_output[2][0];
  

  //printf("lo: %f, co: %f, ro: %f\n", tensor_output[0][0], tensor_output[1][0], tensor_output[2][0]);

  //printf("l: %f, c: %f, r: %f\n", leftobstacle_prob, centerobstacle_prob, rightobstacle_prob);

  

  // Get the end time
  gettimeofday(&end, NULL);
  
  // Calculate the elapsed time in microseconds
  long elapsed = (end.tv_sec - start.tv_sec) * 1000000 + end.tv_usec - start.tv_usec;
  
  // Print the elapsed time in microseconds
  //printf("Elapsed time: %ld microseconds\n", elapsed);

}





  // float tensor_input[3][3][170][170] = {0};
  
  // float copy_tensor[3][170][170];

  
  // int nRows = cropped_image.rows;
  // int nCols = cropped_image.cols;

  // int i,j;
  // uchar* p;
  // for( i = 0; i < nRows; ++i)
  // {
  //     p = cropped_image.ptr<uchar>(i);
  //     for ( j = 0; j < nCols; ++j)
  //     {
  //         copy_tensor[0][j][i] = (float)p[j] /255.0; // Check i, j order
  //         copy_tensor[1][j][i] = (float)p[j+1] /255.0;
  //         copy_tensor[2][j][i] = (float)p[j+2] /255.0;
  //     }
  // }

  // printf("data: %f", copy_tensor[0][0][0]);

  // printf("size: %d",sizeof (copy_tensor));

  // memcpy(&tensor_input[0][0][0][0], &copy_tensor, sizeof (copy_tensor));
  // memcpy(&tensor_input[1][0][0][0], &copy_tensor, sizeof (copy_tensor));
  // memcpy(&tensor_input[2][0][0][0], &copy_tensor, sizeof (copy_tensor));
  













