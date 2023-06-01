import math

filename = "input1.txt"

#vertex class
#id, x, y
class Vertex:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y

    def get_id(self):
        return self.id
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def getCords(self):
        return (self.x, self.y)
    
    def __str__(self):
        return str(self.id) + " " + str(self.x) + " " + str(self.y)

def create_vertex_list():
    vertex_list = []
    with open(filename) as f:
        for line in f:
            if line[0].isdigit():
                line = line.split()
                vertex_list.append(Vertex(int(line[0]), float(line[1]), float(line[2])))
    return vertex_list

def create_adjacency_matrix(vertex_list):
    matrix = [[0 for x in range(len(vertex_list))] for y in range(len(vertex_list))]
    for i in range(len(vertex_list)):
        for j in range(len(vertex_list)):
            matrix[i][j] = distance(vertex_list[i], vertex_list[j])
    return matrix

def distance(vertex1, vertex2):
    return math.sqrt((vertex1.get_x() - vertex2.get_x())**2 + (vertex1.get_y() - vertex2.get_y())**2)

def print_matrix(matrix):
    for i in range(len(matrix)):
        print(matrix[i])

def print_vertex_list(vertex_list):
    for i in range(len(vertex_list)):
        print(vertex_list[i])

vertex_list = create_vertex_list()
matrix = create_adjacency_matrix(vertex_list)

print_matrix(matrix)
print_vertex_list(vertex_list)


