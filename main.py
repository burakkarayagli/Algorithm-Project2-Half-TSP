import math
import numpy as np
import matplotlib.pyplot as plt

#vertex class
#id, x, y
class Vertex:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.adjacents = [-1]

def create_vertex_list(filename):
    vertex_list = []
    with open(filename) as f:
        for line in f:
            line = line.split()
            vertex_list.append(Vertex(int(line[0]), float(line[1]), float(line[2])))
            del line
    return np.array(vertex_list)

def create_adjacency_matrix(vertex_list):
    n = len(vertex_list)
    matrix = np.zeros((n,n), dtype=int)
    for i in range(len(vertex_list)):
        row = matrix[i]
        for j in range(i):
            dist = distance(vertex_list[i], vertex_list[j])
            row[j] = dist
            matrix[j][i] = dist
    return matrix

def distance(vertex1, vertex2):
    return round(math.sqrt((vertex1.x - vertex2.x)**2 + (vertex1.y - vertex2.y)**2))
      
def plot_graph(vertex_list, odd_vertices, perfect_match):
    plt.figure(1)

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

def plot_path(path, index):
    plt.figure(index)
    for i in range(len(path)-1):
        x = [vertex_list[path[i]].x, vertex_list[path[i+1]].x]
        y = [vertex_list[path[i]].y, vertex_list[path[i+1]].y]
        plt.plot(x, y, marker='o', markersize=3, mec='r', mfc='r', color='k', linewidth=1)

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
    return np.array(odd_vertices)

def find_perfect_matching_greedy(odd_vertices, matrix):
    perfect_match = []
    selected = np.zeros(len(odd_vertices))

    for i in range(len(odd_vertices)):
        if selected[i] == 0:
            min_dist = float('inf')
            index = -1

            v = odd_vertices[i]
            for j in range(len(odd_vertices)):
                u = odd_vertices[j]
                if i != j and selected[j] == 0 and matrix[v][u] < min_dist:
                    index = j
                    min_dist = matrix[v][u]
            selected[index] = 1
            selected[i] = 1
            perfect_match.append((v, odd_vertices[index]))
    return np.array(perfect_match)   

def generate_multiGraph(vertex_list, perfect_match):
    for i in range(len(perfect_match)):
        v = perfect_match[i][0]
        u = perfect_match[i][1]
        vertex_list[v].adjacents.append(u)
        vertex_list[u].adjacents.append(v)
    return vertex_list

def find_euler_circuit(vertex_list):
    path = []
    stack = []
    current_vertex = 0
    stack.append(current_vertex)
    while len(stack) > 0:
        if len(vertex_list[current_vertex].adjacents) > 0:
            stack.append(current_vertex)
            next_vertex = vertex_list[current_vertex].adjacents[0]
            vertex_list[current_vertex].adjacents.remove(next_vertex)
            vertex_list[next_vertex].adjacents.remove(current_vertex)
            current_vertex = next_vertex
        else:
            path.append(current_vertex)
            current_vertex = stack.pop()
    return np.array(path)
    
def euler_to_tsp(euler):
    tsp_path = []
    for i in euler:
        if i not in tsp_path:
            tsp_path.append(i)
    tsp_path.append(0)
    return np.array(tsp_path)

def christofides(vertex_list, matrix):
    vertex_list = prim_mst(vertex_list, matrix)
    vertex_list = fill_edges(vertex_list)

    odd_vertices = find_odd_vertices(vertex_list)
    perfect_match = find_perfect_matching_greedy(odd_vertices, matrix)

    plot_graph(vertex_list, odd_vertices, perfect_match)
    
    vertex_list = generate_multiGraph(vertex_list, perfect_match)
    euler_circuit = find_euler_circuit(vertex_list)
    tsp_path = euler_to_tsp(euler_circuit)
    return tsp_path

def total_dist(path, matrix):
    dist = 0
    for i in range(len(path)-1):
        dist += matrix[path[i]][path[i+1]]
    return dist

def two_opt(path, matrix, max_iter=100, mode=0):
    best_path = path
    dist = total_dist(best_path, matrix)
    const_dist = dist

    improved = True
    count = 0
    while improved and (mode == 0 or (mode == 1 and count < max_iter)):
        improved = False
        for i in range(1, len(path)-2):
            for j in range(i+1, len(path)):
                if j-i != 1:
                    new_path = np.copy(path)
                    new_path[i:j] = np.copy(path[j-1:i-1:-1])
                    new_dist = (const_dist + matrix[new_path[i-1]][new_path[i]] + matrix[new_path[j-1]][new_path[j]] 
                    - matrix[path[i-1]][path[i]] - matrix[path[j-1]][path[j]])
                    if new_dist < dist:
                        best_path = new_path
                        dist = new_dist
                        improved = True
        path = best_path
        const_dist = dist
        count += 1
    return best_path

def print_tsp(vertex_list, path):
    output_file = open("output.txt", "w")

    output_file.write(str(total_dist(path, matrix)) + "\n")
    for i in range(len(path)-1):
        output_file.write(str(vertex_list[path[i]].id) + "\n")
    output_file.close()

filename = input("Enter the name of the file: ")

vertex_list = create_vertex_list(filename)
matrix = create_adjacency_matrix(vertex_list)   

tsp_path = christofides(vertex_list, matrix)
plot_path(tsp_path, 2)

optimized_path = two_opt(tsp_path, matrix)
plot_path(optimized_path, 3)

print_tsp(vertex_list, optimized_path)
plt.show()