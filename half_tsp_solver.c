#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <limits.h>

typedef struct stack{
    int data;
    struct stack *next;  
} stack;

typedef stack* stackPtr;

stackPtr newNode(int data) {
    stackPtr node = malloc(sizeof(stack));
    node->data = data;
    node->next = NULL;
    return node;
}

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

typedef struct vertex {
    int id;
    int x;
    int y;
    int adjacentSize;
    int *adjacents;
} vertex;

#define NEW_VERTEX(v) vertex v = {0, 0, 0, 0, NULL}
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
unsigned short* two_opt(unsigned short* path, int** matrix, int* totalDist);
void printOutput(unsigned short* tspPath, vertexPtr vertices, int dist);

int main() {
    char filename[100];
    printf("Enter file name: ");
    scanf("%s", filename);

    vertexPtr vertices = readInput(filename, vertices);
    int **matrix = createWeightMatrix(vertices);

    vertices = primMST(vertices, matrix);
    vertices = fillEdges(vertices);

    int oddSize = 0;
    unsigned short *oddVertices = findOddVertices(vertices, &oddSize);

    int matchSize = oddSize/2;
    unsigned short **perfectMatch = greedyPrefectMatc(oddVertices, oddSize, matrix);
    generateMultigraph(vertices, perfectMatch, matchSize);

    int eulerSize = size;
    unsigned short *euler = findEulerCircuit(vertices, &eulerSize);
    unsigned short *tsp = eulerToTSP(euler, eulerSize);;
    int dist = 0;

    unsigned short *optimized = two_opt(tsp, matrix, &dist);
    printOutput(optimized, vertices, dist);

    return 0;
}

vertexPtr readInput(char* filePath, vertexPtr vertices) {
    FILE* input = fopen(filePath, "r");

    fscanf(input, "%d", &size);
    vertices = malloc(sizeof(vertex) * size);

    for (int i = 0; i < size; i++) {
        NEW_VERTEX(v);
        fscanf(input, "%d %d %d", &v.id, &v.x, &v.y);
        vertices[i] = v;
    }
    return vertices;
}

int totalDistance(vertex v1, vertex v2) {
    return (int) (sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2)) + 0.5);
}

int** createWeightMatrix(vertexPtr vertices) {
    int ** matrix = malloc(sizeof(int*) * size);
    for (int i = 0; i < size; i++) {
        matrix[i] = calloc(size, sizeof(int));
    }

    for (int i = 0; i < size; i++) {
        int *row = matrix[i];
        for (int j = 0; j < i; j++) {
            int distance = totalDistance(vertices[i], vertices[j]);
            row[j] = distance;
            matrix[j][i] = distance;
        }
    }
    return matrix;
}

int minKey(int keys[], int isVisited[]) {
    int min = INT_MAX;
    int index = -1;

    for (int i = 0; i < size; i++) {
        if (isVisited[i] == 0 && keys[i] < min) {
            min = keys[i];
            index = i;
        }
    }
    return index;
}

vertexPtr primMST(vertexPtr vertices, int** matrix) {
    int key[size];
    int isVisited[size];
    for (int i = 0; i < size; i++) {
        key[i] = INT_MAX;
        isVisited[i] = 0;
    }

    key[0] = 0;

    for (int i = 0; i < size-1; i++) {
        int u = minKey(key, isVisited);
        isVisited[u] = 1;

        for (int v = 0; v < size; v++) {
            if (matrix[u][v] && isVisited[v] == 0 && matrix[u][v] < key[v]) {
                key[v] = matrix[u][v];
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

int hasEdge(vertex v, int u) {
    for (int i = 0; i < v.adjacentSize; i++) {
        if (v.adjacents[i] == u) {
            return 1;
        }
    }
    return 0;
}

vertexPtr fillEdges(vertexPtr vertices) {
    for (int i = 0; i < size; i++) {
        vertex v = vertices[i];
        for (int j = 0; j < v.adjacentSize; j++) {
            int u = v.adjacents[j];
            if (!hasEdge(vertices[u], i)) {
                int newSize = ++vertices[u].adjacentSize;
                vertices[u].adjacents = realloc(vertices[u].adjacents, sizeof(int) * newSize);
                vertices[u].adjacents[newSize-1] = i;
            }
        }
    }
    return vertices;
}

unsigned short *findOddVertices(vertexPtr vertices, int *oddSize) {
    int initalSize = size/4;
    unsigned short *oddVertices = malloc(sizeof(unsigned short) * initalSize);
    *oddSize = 0;
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

unsigned short **greedyPrefectMatc(unsigned short* oddVertices, int oddSize, int** matrix) {
    unsigned short **match = malloc(sizeof(unsigned short*) * (oddSize/2));
    for (int i = 0; i < oddSize/2; i++) {
        match[i] = malloc(sizeof(unsigned short) * 2);
    }

    int selected[oddSize];
    for (int i = 0; i < oddSize; i++) {
        selected[i] = 0;
    }

    int matchIndex = 0;
    for (int i = 0; i < oddSize; i++) {
        if (!selected[i]) {
            int minDist = INT_MAX;
            int index = -1;

            int v = oddVertices[i];
            for (int j = 0; j < oddSize; j++) {
                int u = oddVertices[j];
                if (i != j && !selected[j] && matrix[v][u] < minDist) {
                    index = j;
                    minDist = matrix[v][u];
                }
            }
            selected[i] = 1;
            selected[index] = 1;

            match[matchIndex][0] = v;
            match[matchIndex][1] = oddVertices[index];
            matchIndex++;
        }
    }
    return match;
}

void generateMultigraph(vertexPtr vertices, unsigned short** match, int matchSize) {
    for (int i = 0; i < matchSize; i++) {
        int v = match[i][0];
        int u = match[i][1];

        int newSize = ++vertices[v].adjacentSize;
        vertices[v].adjacents = realloc(vertices[v].adjacents, sizeof(int) * newSize);
        vertices[v].adjacents[newSize-1] = u;

        newSize = ++vertices[u].adjacentSize;
        vertices[u].adjacents = realloc(vertices[u].adjacents, sizeof(int) * newSize);
        vertices[u].adjacents[newSize-1] = v;
    }

}

void removeAdjacent(vertexPtr vertex, int data) {
    int index = -1;
    int size = vertex->adjacentSize;
    for (int i = 0; i < size; i++) {
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

unsigned short *findEulerCircuit(vertexPtr vertices, int* eulerSize) {
    unsigned short *path = malloc(sizeof(unsigned short) * size);
    stackPtr stack = NULL;

    int index = 0;
    int current = 0;
    push(&stack, current);
    while (length(stack) > 0) {
        if (vertices[current].adjacentSize > 0) {
            push(&stack, current);

            int next = vertices[current].adjacents[0];
            removeAdjacent(&vertices[current], next);
            removeAdjacent(&vertices[next], current);

            current = next;
        }
        else {
            path[index] = current;
            index++;
            if (index == size) {
                path = realloc(path, sizeof(unsigned short) * index*2);
                *eulerSize = index*2;
            }
            current = pop(&stack);
        }
    }
    return path;
}

unsigned short *eulerToTSP(unsigned short* eulerPath, int eulerSize) {
    unsigned short *tspPath = malloc(sizeof(unsigned short) * (size+1));

    int index = 1;
    tspPath[0] = eulerPath[0];

    for (int i = 1; i < eulerSize; i++) {
        int current = eulerPath[i];
        int exist = 0;
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
    tspPath[size] = tspPath[0];
    return tspPath;
}

int totalDist(unsigned short* tspPath, int** matrix) {
    int totalDist = 0;
    for (int i = 0; i < size; i++) {
        totalDist += matrix[tspPath[i]][tspPath[i+1]];
    }
    return totalDist;
}

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

void copy(unsigned short* dest, unsigned short* source) {
    for (int i = 0; i <= size; i++) {
        dest[i] = source[i];
    }
}

unsigned short* two_opt(unsigned short* path, int** matrix, int* distance) {
    clock_t start = clock();
    clock_t end = start;

    unsigned short *bestPath = malloc(sizeof(unsigned short) * (size+1));
    unsigned short *newPath = malloc(sizeof(unsigned short) * (size+1));
    int dist = totalDist(path, matrix);
    int constDist = dist;

    int improved = 1;
    while (improved) {
        improved = 0;
        for (int i = 1; i < size-2; i++) {
            for (int j = i+1; j < size; j++) {
                if ((end-start)/CLOCKS_PER_SEC > 600) {
                    *distance = dist;
                    return bestPath;
                }
                if (j - i == 1) continue;
                two_optSwap(path, i, j, newPath);
                int newDist = constDist + matrix[newPath[i-1]][newPath[i]] + matrix[newPath[j-1]][newPath[j]] - matrix[path[i-1]][path[i]] - matrix[path[j-1]][path[j]];
                if (newDist < dist) {
                    copy(bestPath, newPath);
                    dist = newDist;
                    improved = 1;
                }
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

void printOutput(unsigned short *tspPath, vertexPtr vertices, int dist) {
    FILE *output = fopen("output.txt", "w");
    fprintf(output, "%d\n", dist);
    for (int i = 0; i < size; i++) {
        fprintf(output, "%d\n", vertices[tspPath[i]].id);
    }
    fclose(output);
}