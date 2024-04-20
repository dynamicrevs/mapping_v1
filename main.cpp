#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <wiringSerial.h>
#include <limits.h>

#define MOTOR_A1 17
#define MOTOR_A2 23
#define MOTOR_B1 24
#define MOTOR_B2 25

#define MAX_SPEED 1000
#define MIN_SPEED 500

#define COMMAND_FORWARD 'F'
#define COMMAND_LEFT 'L'
#define COMMAND_RIGHT 'R'
#define COMMAND_STOP 'S'
#define COMMAND_MAP 'M'
#define COMMAND_STOP_MAP 'X'
#define COMMAND_DESTINATION 'D'

#define GRID_SIZE 10

int curSpeed = MIN_SPEED;
int serial_port;
int mapping = 0; // Flag to indicate mapping mode

// Structure to represent a cell in the grid
typedef struct {
    int x, y; // Cell coordinates
    int f, g, h; // A* algorithm values
    struct Cell *parent; // Pointer to parent cell
} Cell;

void initDriver() {
    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Can't initialise wiringPi: %s\n", strerror(errno));
        exit(1);
    }

    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);

    serial_port = serialOpen("/dev/ttyS0", 9600); // Change "/dev/ttyS0" to your serial port
    if (serial_port < 0) {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        exit(1);
    }
}

void stopMoving() {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, LOW);
}

void moveForward() {
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
}

void turnLeft() {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
}

void turnRight() {
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
}

void backOff(int direction, int duration) {
    if (direction == 0) {
        digitalWrite(MOTOR_A1, LOW);
        digitalWrite(MOTOR_A2, HIGH);
        digitalWrite(MOTOR_B1, LOW);
        digitalWrite(MOTOR_B2, HIGH);
    } else {
        digitalWrite(MOTOR_A1, HIGH);
        digitalWrite(MOTOR_A2, LOW);
        digitalWrite(MOTOR_B1, HIGH);
        digitalWrite(MOTOR_B2, LOW);
    }
    usleep(duration * 1000);
    stopMoving();
}

void sendCommand(char command) {
    serialPutchar(serial_port, command);
}

// Function to update the grid map with the robot's position
void updateGridMap(int grid[][GRID_SIZE], Cell position) {
    grid[position.x][position.y] = 1;
}

// Function to calculate the heuristic value (h) using Manhattan distance
int calculateHeuristic(Cell current, Cell destination) {
    return abs(current.x - destination.x) + abs(current.y - destination.y);
}

// Function to check if a cell is valid (within grid boundaries and not an obstacle)
int isValid(int x, int y, int grid[][GRID_SIZE]) {
    return (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && grid[x][y] != 1);
}

// Function to perform A* algorithm to find the path from start to destination
void aStar(int grid[][GRID_SIZE], Cell start, Cell destination) {
    // Array to keep track of visited cells
    int visited[GRID_SIZE][GRID_SIZE];
    memset(visited, 0, sizeof(visited));

    // Priority queue for open cells
    Cell *openSet[GRID_SIZE * GRID_SIZE];
    int openSetSize = 0;

    // Initialize start cell values
    start.g = 0;
    start.h = calculateHeuristic(start, destination);
    start.f = start.g + start.h;
    start.parent = NULL;
    openSet[openSetSize++] = &start;

    // Main loop
    while (openSetSize > 0) {
        // Find the cell with the lowest f value in the open set
        int minIndex = 0;
        for (int i = 1; i < openSetSize; i++) {
            if (openSet[i]->f < openSet[minIndex]->f) {
                minIndex = i;
            }
        }

        // Get the current cell from the open set
        Cell *current = openSet[minIndex];

        // Check if the current cell is the destination
        if (current->x == destination.x && current->y == destination.y) {
            // Path found, reconstruct the path
            while (current != NULL) {
                // Update grid map with path
                updateGridMap(grid, *current);
                current = current->parent;
            }
            return; // Exit function
        }

        // Remove current cell from open set
        openSet[minIndex] = openSet[--openSetSize];

        // Mark current cell as visited
        visited[current->x][current->y] = 1;

        // Generate neighbors of current cell
        int dx[] = {1, -1, 0, 0};
        int dy[] = {0, 0, 1, -1};
        for (int i = 0; i < 4; i++) {
            int newX = current->x + dx[i];
            int newY = current->y + dy[i];

            // Check if the neighbor is valid
            if (isValid(newX, newY, grid) && !visited[newX][newY]) {
                // Calculate tentative g value
                int tentativeG = current->g + 1;

                // Check if this path to the neighbor is better than any previous one
                int foundBetter = 0;
                for (int j = 0; j < openSetSize; j++) {
                    if (openSet[j]->x == newX && openSet[j]->y == newY && tentativeG < openSet[j]->g) {
                        foundBetter = 1;
                        break;
                    }
                }

                if (!foundBetter) {
                    // Create neighbor cell
                    Cell *neighbor = (Cell *)malloc(sizeof(Cell));
                    neighbor->x = newX;
                    neighbor->y = newY;
                    neighbor->g = tentativeG;
                    neighbor->h = calculateHeuristic(*neighbor, destination);
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = current;

                    // Add neighbor to open set
                    openSet[openSetSize++] = neighbor;
                }
            }
        }
    }
}

// Function to handle mapping mode
void handleMapping(int grid[][GRID_SIZE]) {
    // Send start mapping signal via Bluetooth
    serialPuts(serial_port, "Start mapping\n");

    // Implement mapping logic here
    // Example: Move the robot to map the environment
    // Example: Update the grid map with obstacles
    
    // Send stop mapping signal via Bluetooth
    serialPuts(serial_port, "Stop mapping\n");
}

// Function to move the robot to a desired destination
void moveToDestination(Cell destination) {
    // Grid map initialization
    int gridMap[GRID_SIZE][GRID_SIZE] = {{0}};

    // Execute A* algorithm to find the path to the destination
    aStar(gridMap, (Cell){GRID_SIZE / 2, GRID_SIZE / 2}, destination);

    // Follow the path by turning towards each successive cell
    // and updating the grid map with the robot's movement
    // Example: Move the robot to the destination using the path
}

int main() {
    initDriver();

    // Main loop
    while (1) {
        // Receive commands from Bluetooth module
        if (serialDataAvail(serial_port) > 0) {
            char receivedChar = serialGetchar(serial_port);
            // Execute command received from Bluetooth
            switch (receivedChar) {
                case COMMAND_FORWARD:
                    moveForward();
                    break;
                case COMMAND_LEFT:
                    turnLeft();
                    break;
                case COMMAND_RIGHT:
                    turnRight();
                    break;
                case COMMAND_STOP:
                    stopMoving();
                    break;
                case COMMAND_MAP:
                    // Start mapping mode
                    mapping = 1;
                    handleMapping();
                    break;
                case COMMAND_STOP_MAP:
                    // Stop mapping mode
                    mapping = 0;
                    break;
                case COMMAND_DESTINATION:
                    // Receive desired destination coordinates from Bluetooth
                    Cell destination;
                    destination.x = serialGetchar(serial_port); // Receive X coordinate
                    destination.y = serialGetchar(serial_port); // Receive Y coordinate
                    // Move the robot to the desired destination
                    moveToDestination(destination);
                    break;
                default:
                    break;
            }
        }
    }

    return 0;
}
