#include "modules/probability_map/probability_map.h"








float roundToOneDecimal(float num) {
    return roundf(num * 10) / 10;
}

void printMiddleMap(){
    int startX = (map.width - 20) / 2;
    int startY = (map.height - 20) / 2;

    for (int i = startY; i < startY + 20; i++) {
        for (int j = startX; j < startX + 20; j++) {
            printf("%.1f ", map.grid[i][j]);
        }
        printf("\n");
    }
}

void probability_map_init(){
    map.width = ROOM_WIDTH;
    map.height = ROOM_HEIGHT;

    // Fill all cells with the specified value
    for (int i = 0; i < map.height; i++) {
        for (int j = 0; j < map.width; j++) {
            map.grid[i][j] = 1.0f;
        }
    }
}


void probability_map_update(){
    float x = stateGetPositionEnu_f() -> x;
    float y = stateGetPositionEnu_f() -> y;

    int location_map_x = roundf(map.width / 2) + roundToOneDecimal(x)*10;
    int location_map_y = roundf(map.height / 2) + roundToOneDecimal(y)*10;

    if ((location_map_x >= 0 && location_map_x < 100) && (location_map_y >= 0 && location_map_y < 100)) {
        map.grid[location_map_x][location_map_y] = 0;
        // printMiddleMap(map);
    }
}

float distance(Point p1, Point p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
}

// Function to generate random points within the specified range
void generateRandomPoints(Point points[], int num_points) {
    for (int i = 0; i < num_points; i++) {
        points[i].x = (float)rand() / RAND_MAX * (RANGE_MAX - RANGE_MIN) + RANGE_MIN;
        points[i].y = (float)rand() / RAND_MAX * (RANGE_MAX - RANGE_MIN) + RANGE_MIN;
    }
}

// Function to solve TSP using brute-force approach and return the shortest path
Point* solveTSP(Point points[], int num_points) {
    // Array to store the order of visited points (permutation)
    int order[NUM_POINTS];
    for (int i = 0; i < num_points; i++) {
        order[i] = i;
    }

    float min_distance = INFINITY;
    int min_order[NUM_POINTS];

    // Try all possible permutations
    do {
        float total_distance = 0;
        for (int i = 0; i < num_points - 1; i++) {
            total_distance += distance(points[order[i]], points[order[i + 1]]);
        }
        total_distance += distance(points[order[num_points - 1]], points[order[0]]);

        // Update the minimum distance and order if a shorter path is found
        if (total_distance < min_distance) {
            min_distance = total_distance;
            for (int i = 0; i < num_points; i++) {
                min_order[i] = order[i];
            }
        }
    } while (next_permutation(order, order + num_points));

    // Create an array to store the shortest path
    Point* shortest_path = (Point*)malloc(num_points * sizeof(Point));
    for (int i = 0; i < num_points; i++) {
        shortest_path[i] = points[min_order[i]];
    }

    return shortest_path;
}




