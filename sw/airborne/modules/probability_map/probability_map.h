#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "state.h"


#ifndef PROBABILITY_MAP_H
#define PROBABILITY_MAP_H

#define ROOM_WIDTH 41  //
#define ROOM_HEIGHT 41
#define CLOSE_ENOUGH_THRESHOLD 0.5
// #define NUM_POINTS 10
// #define RANGE_MIN -3
// #define RANGE_MAX 3

GridMap map;

typedef struct {
    int width;
    int height;
    int grid[ROOM_HEIGHT][ROOM_WIDTH]; // 2D array representing the grid map
} GridMap;

typedef struct {
    float x;
    float y;
} Point;

typedef struct {
    int index_x;
    int index_y;
    float g; // cost from start to current node
    float h; // heuristic (estimated cost from current node to end)
    float f; // total cost (g + h)
    struct Node* parent;
} Node;

Point points[] = {{0, 0}, {1, 1}, {2, 2}, {3, 3}}; // Array of points
int num_points = sizeof(points) / sizeof(points[0]); 
// int* path;
int* path;
int pathLength = 4;

extern void probability_map_init();
extern void probability_map_update();
extern void printMiddleMap();

extern float roundToOneDecimal(float num);
extern float distance(Point p1, Point p2);

extern void generateRandomPoints(Point points[], int num_points);

extern int coordinatesToIndexX(float x);
extern int coordinatesToIndexY(float x);
extern float indexToCoordiantesX(int x);
extern float indexToCoordiantesY(int x);

extern int countZeroPoints();
extern void chooseRandomZeroPoint(int* randomX, int* randomY);

extern bool inMap(int x,int y);
extern bool isValid(int x, int y);
extern bool isInClosedList(Node* node, Node** closedList, int closedSize);

extern Node getStartNode();
extern Node getGoalNode();

extern int* A_star();

extern int* createInitialPath(Point* points, int num_points);

extern Point getGlobalDirection();

#endif
