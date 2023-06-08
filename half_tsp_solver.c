#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <limits.h>

// Definition for stack
typedef struct stack{
    int data;
    struct stack *next;  
} stack;

typedef stack* stackPtr;

// Function for creating a new node
stackPtr newNode(int data) {
    stackPtr node = malloc(sizeof(stack));
    node->data = data;
    node->next = NULL;
    return node;
}

// Stack functions
void push(stackPtr* root, int data) {
    stackPtr node = newNode(data);
    node->next = *root;
    *root = node;
}
  
int pop(stackPtr* root) {
    stackPtr temp = *root;
    *root = (*root)->next;
    int popped = temp->data;
    free(temp);
  
    return popped;
}

int length(stackPtr root) {
    int count = 0;
    stackPtr temp = root;
    while (temp != NULL) {
        count++;
        temp = temp->next;
    }
    return count;
}

// Definition for vertex
typedef struct vertex {
    int id;
    int x;
    int y;
    int adjacentSize;
    int *adjacents;
} vertex;


#define NEW_VERTEX(v) vertex v = {0, 0, 0, 0, NULL} //  macro for creating a new vertex
typedef struct vertex* vertexPtr;


int size;
vertexPtr readInput(char* filePath, vertexPtr vertices);
int** createWeightMatrix(vertexPtr vertices);
int minKey(int keys[], int isVisited[]);
vertexPtr primMST(vertexPtr vertices, int** matrix);
int hasEdge(vertex v, int u);
vertexPtr fillEdges(vertexPtr vertices);
unsigned short *findOddVertices(vertexPtr vertices, int *oddSize);
unsigned short **greedyPrefectMatc(unsigned short* oddVertices, int oddSize, int** matrix);
void generateMultigraph(vertexPtr vertices, unsigned short** match, int matchSize);
void removeAdjacent(vertexPtr vertex, int data);
unsigned short *findEulerCircuit(vertexPtr vertices, int* eulerSize);
unsigned short *eulerToTSP(unsigned short* eulerPath, int eulerSize);
int totalDist(unsigned short* tspPath, int** matrix);
void two_optSwap(unsigned short* source, int i, int j, unsigned short* dest);
void copy(unsigned short* dest, unsigned short* source);
unsigned short* two_opt(unsigned short* path, int** matrix, int* totalDist, int maxRuntime);
void printOutput(unsigned short* tspPath, vertexPtr vertices, int dist);

int main() {
    // Read max runtime from user
    int runtime;
    printf("Enter max runtime (sec.): ");
    scanf("%d", &runtime);

    // Creating vertices and weigth matrix from the input file
    vertexPtr vertices = readInput("half-input.txt", vertices);
    int **matrix = createWeightMatrix(vertices);

    // Finding MST
    vertices = primMST(vertices, matrix);
    vertices = fillEdges(vertices);

    // Finding odd vertices
    int oddSize = 0;
    unsigned short *oddVertices = findOddVertices(vertices, &oddSize);

    // Finding perfect match
    int matchSize = oddSize/2;
    unsigned short **perfectMatch = greedyPrefectMatc(oddVertices, oddSize, matrix);
    generateMultigraph(vertices, perfectMatch, matchSize);

    // Generating Euler circuit and transforming it to TSP path
    int eulerSize = size;
    unsigned short *euler = findEulerCircuit(vertices, &eulerSize);
    unsigned short *tsp = eulerToTSP(euler, eulerSize);
    int dist = 0;

    // Optimizing the TSP path with 2-opt
    unsigned short *optimized = two_opt(tsp, matrix, &dist, runtime);
    printOutput(optimized, vertices, dist);

    return 0;
}


vertexPtr readInput(char* filePath, vertexPtr vertices) {
    FILE* input = fopen(filePath, "r");

    fscanf(input, "%d", &size);
    // Allocate memory for vertices
    vertices = malloc(sizeof(vertex) * size);

    // Read vertices and store them in the array
    for (int i = 0; i < size; i++) {
        NEW_VERTEX(v);
        fscanf(input, "%d %d %d", &v.id, &v.x, &v.y);
        vertices[i] = v;
    }
    return vertices;
}

// Function for calculating the distance between two vertices
int totalDistance(vertex v1, vertex v2) {
    return (int) (sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2)) + 0.5);
}

// Creating the adjacency matrix.
int** createWeightMatrix(vertexPtr vertices) {
    int ** matrix = malloc(sizeof(int*) * size);
    for (int i = 0; i < size; i++) {
        matrix[i] = calloc(size, sizeof(int));
    }

    // Fill the matrix with distances
    for (int i = 0; i < size; i++) {
        int *row = matrix[i];
        // Filling the upper triangle
        for (int j = 0; j < i; j++) {
            int distance = totalDistance(vertices[i], vertices[j]);
            row[j] = distance;
            matrix[j][i] = distance;
        }
    }
    return matrix;
}

// Function for finding the vertex with minimum key value
// This function is used in Prim's algorithm
int minKey(int keys[], int isVisited[]) {
    int min = INT_MAX;
    int index = -1;

    // Iterate through all vertices
    for (int i = 0; i < size; i++) {
        // If the vertex is not visited and its key is smaller than the current minimum
        if (isVisited[i] == 0 && keys[i] < min) {
            // Update the minimum and the index
            min = keys[i];
            index = i;
        }
    }
    return index;
}

// Function for finding the minimum spanning tree with Prim's algorithm
vertexPtr primMST(vertexPtr vertices, int** matrix) {
    int key[size];
    int isVisited[size];
    // Initialize the keys and the visited array
    for (int i = 0; i < size; i++) {
        key[i] = INT_MAX;
        isVisited[i] = 0;
    }

    key[0] = 0;
    // Iterate through all vertices
    for (int i = 0; i < size-1; i++) {
        int u = minKey(key, isVisited);
        isVisited[u] = 1;
        // Iterate through all adjacent vertices
        for (int v = 0; v < size; v++) {
            // If there is an edge between the vertices, the vertex is not visited and the weight is smaller than the current key
            if (matrix[u][v] && isVisited[v] == 0 && matrix[u][v] < key[v]) {
                // Update the key and the adjacent vertex
                key[v] = matrix[u][v];
                // If the vertex has no adjacent vertices, allocate memory for one
                if (vertices[v].adjacentSize == 0) {
                    vertices[v].adjacentSize++;
                    vertices[v].adjacents = malloc(sizeof(int));
                }
                 vertices[v].adjacents[0] = u;
            }
        }
    }
    return vertices;
}

// Function for checking if a vertex has an edge to another vertex
int hasEdge(vertex v, int u) {
    for (int i = 0; i < v.adjacentSize; i++) {
        if (v.adjacents[i] == u) {
            return 1;
        }
    }
    return 0;
}

// Function for filling the edges of the MST
// For example vertex v has vertex u in its adjacents list, but vertex u doesn't have vertex v in its adjacents list
vertexPtr fillEdges(vertexPtr vertices) {
    for (int i = 0; i < size; i++) {
        vertex v = vertices[i];
        for (int j = 0; j < v.adjacentSize; j++) {
            int u = v.adjacents[j];
            // If the vertex has no edge to the adjacent vertex, add one
            if (!hasEdge(vertices[u], i)) {
                int newSize = ++vertices[u].adjacentSize;
                vertices[u].adjacents = realloc(vertices[u].adjacents, sizeof(int) * newSize);
                vertices[u].adjacents[newSize-1] = i;
            }
        }
    }
    return vertices;
}

// Function for finding the odd vertices
unsigned short *findOddVertices(vertexPtr vertices, int *oddSize) {
    int initalSize = size/4;
    unsigned short *oddVertices = malloc(sizeof(unsigned short) * initalSize);
    *oddSize = 0;
    // Iterate through all vertices and checking their adjacent size. If it is odd, add it to the array
    for (int i = 0; i < size; i++) {
        if (vertices[i].adjacentSize % 2 == 1) {
            if (*oddSize == initalSize) {
                initalSize *= 2;
                oddVertices = realloc(oddVertices, sizeof(unsigned short) * initalSize);
            }
            oddVertices[*oddSize] = i;
            (*oddSize)++;
        }
    }
    return oddVertices;
}

// Function for the finding perfect matchings
unsigned short **greedyPrefectMatc(unsigned short* oddVertices, int oddSize, int** matrix) {
    unsigned short **match = malloc(sizeof(unsigned short*) * (oddSize/2));
    // Allocate memory for the matchings
    for (int i = 0; i < oddSize/2; i++) {
        match[i] = malloc(sizeof(unsigned short) * 2);
    }

    int selected[oddSize];
    // Initialize the selected array
    for (int i = 0; i < oddSize; i++) {
        selected[i] = 0;
    }

    int matchIndex = 0;
    // Iterate through all odd vertices
    for (int i = 0; i < oddSize; i++) {

        // If the vertex is not selected, find the closest vertex and add them to the matchings
        if (!selected[i]) {
            int minDist = INT_MAX;
            int index = -1;

            int v = oddVertices[i];
            // Iterate through all odd vertices
            for (int j = 0; j < oddSize; j++) {
                int u = oddVertices[j];
                // If the vertex is not selected and the distance is smaller than the current minimum, update the minimum and the index
                if (i != j && !selected[j] && matrix[v][u] < minDist) {
                    // Update the minimum and the index
                    index = j;
                    minDist = matrix[v][u];
                }
            }
            // Update the selected array and the matchings
            selected[i] = 1;
            selected[index] = 1;

            match[matchIndex][0] = v;
            match[matchIndex][1] = oddVertices[index];
            matchIndex++;
        }
    }
    return match;
}

// Function for generating the multigraph
void generateMultigraph(vertexPtr vertices, unsigned short** match, int matchSize) {
    // Iterate through all matchings
    for (int i = 0; i < matchSize; i++) {
        int v = match[i][0];
        int u = match[i][1];
        // Add the edge between the vertices
        int newSize = ++vertices[v].adjacentSize;
        vertices[v].adjacents = realloc(vertices[v].adjacents, sizeof(int) * newSize);
        vertices[v].adjacents[newSize-1] = u;
        // Add the edge between the vertices
        newSize = ++vertices[u].adjacentSize;
        vertices[u].adjacents = realloc(vertices[u].adjacents, sizeof(int) * newSize);
        vertices[u].adjacents[newSize-1] = v;
    }

}

// Function for removing the edge between two vertices
void removeAdjacent(vertexPtr vertex, int data) {
    int index = -1;
    int size = vertex->adjacentSize;
    // Iterate through all adjacent vertices
    for (int i = 0; i < size; i++) {
        // If the adjacent vertex is the one we are looking for, update the index and the adjacent size
        if (vertex->adjacents[i] == data) {
            index = i;
            vertex->adjacentSize--;
            break;
        }
    }
    for (int i = index; i < size-1; i++) {
        vertex->adjacents[i] = vertex->adjacents[i+1];
    }
}

// Function for finding the Euler circuit with Hierholzer's algorithm
unsigned short *findEulerCircuit(vertexPtr vertices, int* eulerSize) {
    // Allocate memory for the path
    unsigned short *path = malloc(sizeof(unsigned short) * size);
    stackPtr stack = NULL;
    // Push the first vertex to the stack
    int index = 0;
    int current = 0;
    push(&stack, current);
    // Iterate through all vertices
    while (length(stack) > 0) {
        // If the vertex has adjacent vertices, add the vertex to the stack and remove the edge between the vertices
        if (vertices[current].adjacentSize > 0) {
            push(&stack, current);

            int next = vertices[current].adjacents[0];
            removeAdjacent(&vertices[current], next);
            removeAdjacent(&vertices[next], current);

            current = next;
        }
        // If the vertex has no adjacent vertices, add the vertex to the path
        else {
            path[index] = current;
            index++;
            // If the path is full, reallocate the memory
            if (index == size) {
                path = realloc(path, sizeof(unsigned short) * index*2);
                *eulerSize = index*2;
            }
            current = pop(&stack);
        }
    }
    return path;
}

// Function for converting the Euler path to the TSP path
unsigned short *eulerToTSP(unsigned short* eulerPath, int eulerSize) {
    unsigned short *tspPath = malloc(sizeof(unsigned short) * (size+1));

    int index = 1;
    tspPath[0] = eulerPath[0];
    // Iterate through all vertices
    for (int i = 1; i < eulerSize; i++) {
        int current = eulerPath[i];
        int exist = 0;
        // If the vertex is not in the path, add it to the path
        for (int j = 0; j < index; j++) {
            if (tspPath[j] == current) {
                exist = 1;
                break;
            }
        }
        if (!exist) {
            tspPath[index] = current;
            index++;
        }
    }
    // Add the first vertex to the end of the path to complete the cycle
    tspPath[size] = tspPath[0];
    return tspPath;
}

// Function for calculating the total distance of the path
int totalDist(unsigned short* tspPath, int** matrix) {
    int totalDist = 0;
    // Iterate through all vertices in the path
    for (int i = 0; i < size; i++) {
        // Add the distance between the vertices to the total distance
        totalDist += matrix[tspPath[i]][tspPath[i+1]];
    }
    return totalDist;
}

// Helper function for swapping two vertices in the path
void two_optSwap(unsigned short* source, int i, int j, unsigned short* dest) {
    for (int k = 0; k < i; k++) {
        dest[k] = source[k];
    }
    for (int k = i; k < j; k++) {
        dest[k] = source[j-k+i-1];
    }
    for (int k = j; k <= size; k++) {
        dest[k] = source[k];
    }
}

// Helper function for copying the path
void copy(unsigned short* dest, unsigned short* source) {
    for (int i = 0; i <= size; i++) {
        dest[i] = source[i];
    }
}

// Function for performing the 2-opt algorithm
unsigned short* two_opt(unsigned short* path, int** matrix, int* distance, int maxRuntime) {
    // Start the timer
    clock_t start = clock();
    clock_t end = start;

    unsigned short *bestPath = malloc(sizeof(unsigned short) * (size+1));
    // Copy the path to the best path
    copy(bestPath, path);
    unsigned short *newPath = malloc(sizeof(unsigned short) * (size+1));
    int dist = totalDist(path, matrix);
    int constDist = dist;

    // Iterate until no improvement is made
    int improved = 1;
    while (improved) {
        improved = 0;

        // Try all possible swaps
        for (int i = 1; i < size-2; i++) {
            for (int j = i+1; j < size; j++) {
                // If the time limit is exceeded, return the best path
                // This is done to ensure that the program does not run for too long
                if ((end-start)/CLOCKS_PER_SEC > maxRuntime) {
                    *distance = dist;
                    return bestPath;
                }
                if (j - i == 1) continue;
                two_optSwap(path, i, j, newPath);
                // Calculating the new distance of the path
                // We are using the fact that the distance between two vertices is the same in both directions
                // So we can calculate the new distance by adding the distance between the new vertices and subtracting the distance between the old vertices
                int newDist = constDist + matrix[newPath[i-1]][newPath[i]] + matrix[newPath[j-1]][newPath[j]] - matrix[path[i-1]][path[i]] - matrix[path[j-1]][path[j]];
                // If the new distance is smaller, copy the new path to the best path
                if (newDist < dist) {
                    copy(bestPath, newPath);
                    dist = newDist;
                    improved = 1;
                }
                end = clock();
            }
            end = clock();
        }
        copy(path, bestPath);
        constDist = dist;
        end = clock();
    }
    *distance = dist;
    return bestPath;
}

// Function for printing the output
void printOutput(unsigned short *tspPath, vertexPtr vertices, int dist) {
    // Open the output file
    FILE *output = fopen("output.txt", "w");
    fprintf(output, "%d\n", dist);
    for (int i = 0; i < size; i++) {
        // Print the id of the verteces in the path
        fprintf(output, "%d\n", vertices[tspPath[i]].id);
    }
    fclose(output);
}