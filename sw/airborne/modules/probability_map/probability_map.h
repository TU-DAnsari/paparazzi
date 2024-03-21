#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "state.h"


#ifndef PROBABILITY_MAP_H
#define PROBABILITY_MAP_H

#define ROOM_WIDTH 100  //
#define ROOM_HEIGHT 100
#define NUM_POINTS 10
#define RANGE_MIN -3
#define RANGE_MAX 3

GridMap map;

typedef struct {
    int width;
    int height;
    float grid[ROOM_HEIGHT][ROOM_WIDTH]; // 2D array representing the grid map
} GridMap;

typedef struct {
    float x;
    float y;
} Point;

extern void probability_map_init();
extern void probability_map_update();
extern void printMiddleMap();
extern float roundToOneDecimal(float num);
extern float distance(Point p1, Point p2);
extern void generateRandomPoints(Point points[], int num_points);
extern Point* solveTSP(Point points[], int num_points);


#endif
