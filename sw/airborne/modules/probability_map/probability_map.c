#include "modules/probability_map/probability_map.h"

int* path = NULL;
int pathLength = 0;
int num_points = 0;

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
            map.grid[i][j] = 1;
        }
    }
    Point points[] = {{0, 0}, {1, 1}, {2, 2}, {3, 3}};
    num_points = sizeof(points) / sizeof(points[0]);
    path = createInitialPath(points, num_points);
    if (path != NULL) {
        pathLength = 0;
        while (path[pathLength] != -1) {
            pathLength++;
        }
    } else {
        pathLength = 0; // or whatever value is appropriate if no path is found
    }
}


void probability_map_update(){
    float x = stateGetPositionEnu_f() -> x;
    float y = stateGetPositionEnu_f() -> y;

    int location_map_x = coordinatesToIndexX(x);
    int location_map_y = coordinatesToIndexY(y);

    if ((location_map_x >= 0 && location_map_x < 100) && (location_map_y >= 0 && location_map_y < 100)) {
        map.grid[location_map_x][location_map_y] = 0;
        printMiddleMap(map);
    }
}

float distance(Point p1, Point p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
}

// Function to generate random points within the specified range
// void generateRandomPoints(Point points[], int num_points) {
//     for (int i = 0; i < num_points; i++) {
//         points[i].x = (float)rand() / RAND_MAX * (RANGE_MAX - RANGE_MIN) + RANGE_MIN;
//         points[i].y = (float)rand() / RAND_MAX * (RANGE_MAX - RANGE_MIN) + RANGE_MIN;
//     }
// }


int coordinatesToIndexX(float x){
    return roundf((map.width-1) / 2 + roundToOneDecimal(x)*5);
}

int coordinatesToIndexY(float x){
    return roundf((map.height-1) / 2 - roundToOneDecimal(x)*5);
}

float indexToCoordiantesX(int x){
    return (x -(map.width - 1) / 2) / 5;
}

float indexToCoordiantesY(int x){
    /// check htis
    return -(x -(map.height-1) / 2) / 5;
}

bool inMap(int x,int y){
    return (x >= 0 && x < 100) && (y >= 0 && y < 100);
}

bool isValid(int x, int y) {
    if (inMap(x,y) && map.grid[x][y] != 2)
        return true;
    return false;
}

bool isInClosedList(Node* node, Node** closedList, int closedSize) {
    for (int i = 0; i < closedSize; i++) {
        if (closedList[i]->index_x == node->index_x && closedList[i]->index_y == node->index_y)
            return true;
    }
    return false;
}

Node getStartNode(){
    float x = stateGetPositionEnu_f() -> x;
    float y = stateGetPositionEnu_f() -> y;

    int location_map_x = coordinatesToIndexX(x);
    int location_map_y = coordinatesToIndexY(y);
    Node start;
    start.index_x = location_map_x;
    start.index_y = location_map_y;
    start.g =0;
    start.f = 0;
    start.h = 0;
    start.parent= NULL;
    return start;
}

int countZeroPoints() {
    int count = 0;
    for (int i = 0; i < map.width; ++i) {
        for (int j = 0; j < map.height; ++j) {
            if (map.grid[i][j] == 0) {
                count++;
            }
        }
    }
    return count;
}

void chooseRandomZeroPoint(int* randomX, int* randomY) {
    // Count the number of zero points
    int zeroCount = countZeroPoints();
    if (zeroCount == 0) {
        // No zero points found, return (-1, -1)
        *randomX = -1;
        *randomY = -1;
        return;
    }

    // Seed the random number generator
    srand(time(NULL));

    // Generate a random index among zero points
    int randomIndex = rand() % zeroCount;

    // Find the corresponding zero point
    int count = 0;
    for (int i = 0; i < map.width; ++i) {
        for (int j = 0; j < map.height; ++j) {
            if (map.grid[i][j] == 0) {
                if (count == randomIndex) {
                    *randomX = i;
                    *randomY = j;
                    return;
                }
                count++;
            }
        }
    }
}

Node getGoalNode(){
    int index_x;
    int index_y;
    chooseRandomZeroPoint(&index_x, &index_y);
    
    Node goal;
    goal.index_x = index_x;
    goal.index_y = index_y;
    goal.g =0;
    goal.f = 0;
    goal.h = 0;
    goal.parent= NULL;
    return goal;
}

int* createInitialPath(Point* points, int num_points) {
    // Allocate memory for the path array
    int* path = (int*)malloc((num_points + 1) * sizeof(int)); // +1 for the terminating -1

    // Convert each point's coordinates to indices
    for (int i = 0; i < num_points; i++) {
        int index = coordinatesToIndexX(points[i].x) * ROOM_HEIGHT + coordinatesToIndexY(points[i].y);
        path[i] = index;
    }

    // Terminate the path array with -1
    path[num_points] = -1;

    return path;
}


int* A_star(){
    // create surrounding nodes

    Node* openList[ROOM_WIDTH * ROOM_HEIGHT];
    Node* closedList[ROOM_WIDTH * ROOM_HEIGHT];
    int openSize = 0;
    int closedSize = 0;
    Node start = getStartNode();
    Node end = getGoalNode();

    openList[openSize++] = &start;
    // Loop until open list is empty
    while (openSize > 0) {
        // Find node with the lowest f cost in the open list
        int currentIndex = 0;
        for (int i = 1; i < openSize; i++) {
            if (openList[i]->f < openList[currentIndex]->f)
                currentIndex = i;
        }

        // Current node is the one with the lowest f cost
        Node* current = openList[currentIndex];

        // Remove current node from open list
        for (int i = currentIndex; i < openSize - 1; i++)
            openList[i] = openList[i + 1];
        openSize--;

        // Add current node to closed list
        closedList[closedSize++] = current;

        // Check if current node is the goal node
        if (current->index_x == end.index_x && current->index_y == end.index_y) {
            // Reconstruct the path
            int* path = (int*)malloc((current->g + 1) * sizeof(int)); // Allocate memory for path
            Node* temp = current;
            int pathLength = current->g + 1;
            for (int i = pathLength - 1; i >= 0; i--) {
                path[i] = temp->index_x * ROOM_HEIGHT + temp->index_y; // Convert 2D index to 1D index
                temp = temp->parent;
            }
            return path;
        }

        // Generate neighbors of current node
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0)
                    continue;

                int newX = current->index_x + dx;
                int newY = current->index_y + dy;

                // Check if the new node is valid
                if (!isValid(newX, newY))
                    continue;

                // Check if the new node is in the closed list
                if (isInClosedList(&(Node){newX, newY, 0, 0, 0, NULL}, closedList, closedSize))
                    continue;

                // Create the new node
                Node* neighbor = (Node*)malloc(sizeof(Node));
                neighbor->index_x = newX;
                neighbor->index_y = newY;
                neighbor->parent = current;
                neighbor->g = current->g + 1;
                neighbor->h = distance((Point){newX, newY}, (Point){end.index_x, end.index_y});
                neighbor->f = neighbor->g + neighbor->h;

                // Check if the new node is already in the open list
                bool inOpenList = false;
                for (int i = 0; i < openSize; i++) {
                    if (openList[i]->index_x == neighbor->index_x && openList[i]->index_y == neighbor->index_y) {
                        inOpenList = true;
                        if (neighbor->g < openList[i]->g) {
                            // Update existing node in open list with lower g cost
                            openList[i]->parent = neighbor->parent;
                            openList[i]->g = neighbor->g;
                            openList[i]->f = neighbor->f;
                        }
                        break;
                    }
                }

                // If the new node is not in the open list, add it
                if (!inOpenList){
                    openList[openSize++] = neighbor;
                } else {
                    free(neighbor);
                }
            }
        }
    }

    // If the open list is empty and goal not reached, return NULL
    return NULL;
}


Point getGlobalDirection() {
    // Check if there is an existing path
    if (path == NULL || pathLength == 0) {
        // Generate a new path
        path = A_star();
        if (path == NULL) {
            // No path found, return a default point
            return (Point){-1, -1};
        }
        // Calculate path length
        pathLength = 0;
        while (path[pathLength] != -1) {
            pathLength++;
        }
    }

    // Get the next point from the path
    int nextIndex = path[0];
    int nextX = indexToCoordiantesX(nextIndex / ROOM_HEIGHT);
    int nextY =  indexToCoordiantesY(nextIndex % ROOM_HEIGHT);
    Point nextPoint = {nextX, nextY};

    Point dronePosition = {stateGetPositionEnu_f() -> x, stateGetPositionEnu_f() -> y};
    // Check if the drone is close enough to the current point
    // You should replace this condition with your own distance check logic
    if (distance(dronePosition, nextPoint) < CLOSE_ENOUGH_THRESHOLD) {
        // Remove the current point from the path list
        for (int i = 0; i < pathLength - 1; i++) {
            path[i] = path[i + 1];
        }
        path[pathLength - 1] = -1; // Mark the last element as -1
        pathLength--; // Reduce path length

        if (pathLength == 0) {
            free(path);
            path = NULL; // Set path to NULL to indicate no active path
        }
    }

    return nextPoint;
}




