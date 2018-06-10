import queue
import numpy as np

def create_graph(edge_list):

    graph = {}

    for (i,j) in edge_list:
        graph[i] = {}
        graph[j] = {}

    for (i,j) in edge_list:
        graph[i].update({j: distance(i,j)})
        graph[j].update({i: distance(i,j)})

    return graph

def distance(d,s):
    return np.sqrt((d[0]-s[0])**2 + (d[1]-s[1])**2)

def get_path(graph, src, dest):
    dist = {}
    pred = {}

    unvisited = [i for i in graph.keys()]

    for i in graph.keys():
        dist[i] = 99999

    q = queue.PriorityQueue()
    dist[src] = 0

    q.put((0, src))
    unvisited.remove(src)


    while not q.empty():
        du, node = q.get()

        for i in graph[node].keys():
            dv = dist[i]

            if i in unvisited:
                if dv > (du + graph[node][i]):
                    dist[i] = du + graph[node][i]
                    q.put((dist[i], i))
                    pred[i] = node

    node = dest
    path = [node]
    while not pred[node] == src:

        node = pred[node]

        path.append(node)

    path.append(pred[node])
    path.reverse()

    return path