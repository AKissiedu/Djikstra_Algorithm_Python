# This Python Program Is An Implementation of Dijkstra's Single Shortest Path Algorithm.
# The program is for adjacency matrix representation of the graph
# I acknowledge the efforts of Jitender Punia

# Library for INT_MAX
#import sys
intmax=9999999999 
class Graph():
 
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                    for row in range(vertices)]
                    
    
 
    def printSolution(self, dist):
        print ("Vertex \tDistance from Source")
        for node in range(self.V):
            print (node, "\t", dist[node])
 
    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minDistance(self, dist, sptSet):
 
        # Initialize minimum distance for next node
        min = intmax
 
        # Search not nearest vertex not in the
        # shortest path tree
        for u in range(self.V):
            if dist[u] < min and sptSet[u] == False:
                min = dist[u]
                min_index = u
 
        return min_index
 
    # Function that implements Dijkstra's single source
    # shortest path algorithm for a graph represented
    # using adjacency matrix representation
    def dijkstra(self, src):
 
        dist = [intmax] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
 
        for cout in range(self.V):
 
            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # x is always equal to src in first iteration
            x = self.minDistance(dist, sptSet)
 
            # Put the minimum distance vertex in the
            # shortest path tree
            sptSet[x] = True
 
            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for y in range(self.V):
                if self.graph[x][y] > 0 and sptSet[y] == False and \
                dist[y] > dist[x] + self.graph[x][y]:
                        dist[y] = dist[x] + self.graph[x][y]
 
        self.printSolution(dist)
 

# This part of the code is where you specify the number of vertices of your graph 

# The parameter is the number of vertices or nodes of the graph
g = Graph(9)

# Tranform your graph in a matrix form

g.graph = [[0, 5, 0, 0, 0, 0, 0, 7, 0],
	  [5, 0, 6, 0, 0, 0, 0, 0, 12],
	  [0, 6, 0, 7, 0, 4, 0, 0, 3],
	  [0, 0, 7, 0, 10, 12, 0, 0, 0],
	  [0, 0, 0, 10, 0, 10, 0, 0, 0],
	  [0, 0, 4, 12, 10, 0, 1, 0, 0],
	  [0, 0, 0, 0, 0, 0, 1, 0, 2, 4],
          [7, 12, 0, 0, 0, 0, 0, 2, 0, 7],
          [0, 0, 3, 0, 0, 0, 4, 7, 0]]
 
g.dijkstra(0);
