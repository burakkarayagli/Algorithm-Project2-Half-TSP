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
    return (vertex1.x - vertex2.x)**2 + (vertex1.y - vertex2.y)**2

def print_matrix(matrix):
    for i in range(len(matrix)):
        print(matrix[i])

def print_vertex_list(vertex_list):
    for i in range(len(vertex_list)):
        print(vertex_list[i])
        
def plot_graph(vertex_list, odd_vertices, perfect_match):
    x = np.array([v.x for v in vertex_list])
    y = np.array([v.y for v in vertex_list])
    x_o = np.array([vertex_list[v].x for v in odd_vertices])
    y_o = np.array([vertex_list[v].y for v in odd_vertices])
    plt.scatter(x, y, s=15, c='r')
    plt.scatter(x_o, y_o, s=15, c='yellow')

    for v in vertex_list:
        for i in range(len(v.adjacents)):
            x_p = np.array([v.x, vertex_list[v.adjacents[i]].x])
            y_p = np.array([v.y, vertex_list[v.adjacents[i]].y])    
            plt.plot(x_p, y_p, color='#2f95dd', linewidth=1.4)
  
    for v in perfect_match:
        x_p = np.array([vertex_list[v[0]].x, vertex_list[v[1]].x])
        y_p = np.array([vertex_list[v[0]].y, vertex_list[v[1]].y])
        plt.plot(x_p, y_p, ':', color='k', linewidth=1.4)
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
            v = vertex_list[i].adjacents[j]
            if i not in vertex_list[v].adjacents:
                vertex_list[v].adjacents.append(i)
    return vertex_list

def find_odd_vertices(vertex_list):
    odd_vertices = []
    for i in range(len(vertex_list)):
        if len(vertex_list[i].adjacents) % 2 != 0:
            odd_vertices.append(i)
    return odd_vertices

def find_perfect_matching(odd_vertices, matrix):
    graph = nx.Graph()
    for i in range(len(odd_vertices)):
        for j in range(i):
            v = odd_vertices[i]
            u = odd_vertices[j]
            graph.add_edge(v, u, weight=-1*matrix[v][u])
    return list(nx.max_weight_matching(graph, maxcardinality=True))

def generate_multiGraph(vertex_list, perfect_match):
    for i in range(len(perfect_match)):
        v = perfect_match[i][0]
        u = perfect_match[i][1]
        vertex_list[v].adjacents.append(u)
        vertex_list[u].adjacents.append(v)
    return vertex_list
    
vertex_list = create_vertex_list()
matrix = create_adjacency_matrix(vertex_list)   

vertex_list = prim_mst(vertex_list, matrix)
vertex_list = fill_edges(vertex_list)

odd_vertices = find_odd_vertices(vertex_list)
perfect_match = find_perfect_matching(odd_vertices, matrix)

plot_graph(vertex_list, odd_vertices, perfect_match)

vertex_list = generate_multiGraph(vertex_list, perfect_match)