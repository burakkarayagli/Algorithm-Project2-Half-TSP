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

    def get_id(self):
        return self.id
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
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
            if j >= i:
                matrix[i][j] = distance(vertex_list[i], vertex_list[j])
                matrix[j][i] = matrix[i][j]
    return matrix

def distance(vertex1, vertex2):
    return math.sqrt((vertex1.get_x() - vertex2.get_x())**2 + (vertex1.get_y() - vertex2.get_y())**2)

def print_matrix(matrix):
    for i in range(len(matrix)):
        print(matrix[i])

def print_vertex_list(vertex_list):
    for i in range(len(vertex_list)):
        print(vertex_list[i])
        
def plot_graph(vertex_list, edges):
    for i in range(len(edges)):
        for j in range(len(edges[i])):
            x = np.array([vertex_list[i].get_x(), vertex_list[edges[i][j]].get_x()])
            y = np.array([vertex_list[i].get_y(), vertex_list[edges[i][j]].get_y()])    
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
    edges = [[0] for x in range(len(vertex_list))]
    key = np.array([float('inf') for x in range(len(vertex_list))])
    is_visited = np.array([False for x in range(len(vertex_list))])

    key[0] = 0
    edges[0][0] = vertex_list[0].get_id()

    for i in range(len(vertex_list)-1):
        u = minKey(key, is_visited)
        is_visited[u] = True

        for v in range(len(vertex_list)):
            if graph[u][v] != 0 and is_visited[v] == False and graph[u][v] < key[v]:
                key[v] = graph[u][v]
                edges[v][0] = u
    return edges

def find_odd_vertices(vertex_list, mst_edges):
    odd_vertices = []
    for i in range(len(vertex_list)):
        degree = 1
        for j in range(len(mst_edges)):
            if mst_edges[j][0] == i:
                degree += 1
        if degree % 2 != 0:
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

def generate_multiGraph(mst_edges, perfect_match):
    for i in range(len(perfect_match)):
        mst_edges[perfect_match[i][0]].append(perfect_match[i][1])
    return mst_edges
    
vertex_list = create_vertex_list()
matrix = create_adjacency_matrix(vertex_list)

mst_edges = prim_mst(vertex_list, matrix)
odd_vertices = find_odd_vertices(vertex_list, mst_edges)
perfect_match = find_perfect_matching(odd_vertices, matrix)
multiGraph = generate_multiGraph(mst_edges, perfect_match)

plot_graph(vertex_list, multiGraph)