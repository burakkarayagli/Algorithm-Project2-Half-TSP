import math
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

filename = "input1.txt"

#vertex class
#id, x, y
class Vertex:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.adjacents = [-1]

    def get_id(self):
        return self.id
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y

    def get_adjacents(self):
        return self.adjacents
    
    def get_cords(self):
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
    matrix = np.array([[0 for x in range(len(vertex_list))] for y in range(len(vertex_list))])
    for i in range(len(vertex_list)):
        for j in range(len(vertex_list)):
            if i >= j:
                matrix[i][j] = distance(vertex_list[i], vertex_list[j])
                matrix[j][i] = matrix[i][j]
            else:
                break
    return matrix

def distance(vertex1, vertex2):
    return math.sqrt((vertex1.x - vertex2.x)**2 + (vertex1.y - vertex2.y)**2)

def print_matrix(matrix):
    for i in range(len(matrix)):
        print(matrix[i])

def print_vertex_list(vertex_list):
    for i in range(len(vertex_list)):
        print(vertex_list[i])
        
def plot_graph(vertex_list):
    for v in vertex_list:
        for i in range(len(v.adjacents)):
            x = np.array([v.x, vertex_list[v.adjacents[i]].x])
            y = np.array([v.y, vertex_list[v.adjacents[i]].y])    
            plt.plot(x, y, marker='o', markersize=3, mec='r', mfc='r', color='k', linewidth=1)
    plt.show()

def minKey(key, is_visited):
    min =  float('inf')
    index = -1

    for v in range(len(key)):
        if is_visited[v] == False and key[v] < min:
            min = key[v]
            index = v
    return index

def prim_mst(vertex_list, graph):
    key = np.array([float('inf') for x in range(len(vertex_list))])
    is_visited = np.array([False for x in range(len(vertex_list))])

    key[0] = 0

    for i in range(len(vertex_list)-1):
        u = minKey(key, is_visited)
        is_visited[u] = True

        for v in range(len(vertex_list)):
            if graph[u][v] != 0 and is_visited[v] == False and graph[u][v] < key[v]:
                key[v] = graph[u][v]
                vertex_list[v].adjacents[0] = u
    vertex_list[0].adjacents.clear()
    return vertex_list

def fill_edges(vertex_list):
    for i in range(len(vertex_list)):
        for j in range(len(vertex_list[i].adjacents)):
            if i not in vertex_list[vertex_list[i].adjacents[j]].adjacents:
                vertex_list[vertex_list[i].adjacents[j]].adjacents.append(i)
    return vertex_list

def find_odd_vertices(vertex_list):
    odd_vertices = []
    for i in range(len(vertex_list)):
        if len(vertex_list[i].adjacents) % 2 != 0:
            odd_vertices.append(i)
    return odd_vertices

def find_perfect_matching(odd_vertices, matrix):
    edges = []
    graph = nx.Graph()
    for i in range(len(odd_vertices)):
        for j in range(i):
            edges.append((odd_vertices[i], odd_vertices[j], matrix[odd_vertices[i]][odd_vertices[j]]))
    graph.add_weighted_edges_from(edges)

    return list(nx.min_weight_matching(graph))

def generate_multiGraph(vertex_list, perfect_match):
    for i in range(len(perfect_match)):
        vertex_list[perfect_match[i][0]].adjacents.append(perfect_match[i][1])
        vertex_list[perfect_match[i][1]].adjacents.append(perfect_match[i][0])
    return vertex_list
    
vertex_list = create_vertex_list()
matrix = create_adjacency_matrix(vertex_list)   

vertex_list = prim_mst(vertex_list, matrix)
vertex_list = fill_edges(vertex_list)

odd_vertices = find_odd_vertices(vertex_list)
perfect_match = find_perfect_matching(odd_vertices, matrix)
vertex_list = generate_multiGraph(vertex_list, perfect_match)

plot_graph(vertex_list)