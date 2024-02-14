#!/usr/bin/python3


from CS312Graph import *
import time

class NetworkRoutingSolver:
    def __init__( self):
        pass

    def initializeNetwork( self, network ):
        assert( type(network) == CS312Graph )
        self.network = network

    def getShortestPath( self, destIndex ):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        
        path_edges = []
        total_length = 0
        node = self.network.nodes[self.source]
        edges_left = 3
        while edges_left > 0:
            edge = node.neighbors[2]
            path_edges.append( (edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)) )
            total_length += edge.length
            node = edge.dest
            edges_left -= 1
        return {'cost':total_length, 'path':path_edges}

    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()
        pq = []
        if use_heap:
            pq = PriorityQueueBinaryHeap
        else:
            pq = PriorityQueueArray
        pq.make_queue()
        Dijkstra(pq)



        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)

        t2 = time.time()
        return (t2-t1)

def Dijkstra(priorityQueue):



    return


class PriorityQueueArray:
    def __init__(self):
        self.queue = {}

    def insert(self, node, dist):
        self.queue[node] = dist
        # self.queue[node,dist]

    def decrease_dist(self, node, dist):
        if node in self.queue:
            if dist < self.queue[node]:
                self.queue[node] = dist
        else:
            print("Key not found in the priority queue.")

    def delete_min(self):
        if not self.queue:
            print("Priority queue is empty.")
            return None

        min_node = min(self.queue, key=self.queue.get)
        self.queue.pop(min_node)
        return min_node

    # def make_queue(self, solver: NetworkRoutingSolver):
    #     for node in solver.network:
    #         self.queue[node] = (float('inf'))
    # def make_queue(self, nodes):
    #     for node in nodes:
    #         self.queue[node] = (float('inf'))
    def make_queue(self, nodes):
        self.queue = {key: float('inf') for key, _ in nodes}
        # self.queue = dict(nodes)

class PriorityQueueBinaryHeap:
    def __init__(self):
        self.queue = []
        self.indices = {}

    def swap(self, node):

    def bubbleUp(self, curr_index):


    def insert(self, node, dist):
        self.queue.append([node, dist])

        curr_index = len(self.queue) - 1
        while curr_index > 0:
            parent = self.queue[((curr_index+1)//2)-1]
            curr_node = self.queue[curr_index]
            if curr_node[1] > parent[1]:
                temp = parent
                parent = curr_node
                curr_node = temp
                curr_index = ((curr_index+1)//2)-1
            else:
                break

    def decrease_dist(self, node, dist):
        # if node in self.queue:
        if dist < self.queue[node]:
            self.queue[node] = dist
        curr_index = len(self.queue[node]) - 1
        while curr_index > 0:
            parent = self.queue[((curr_index+1)//2)-1]
            curr_node = self.queue[curr_index]
            if curr_node[1] > parent[1]:
                temp = parent
                parent = curr_node
                curr_node = temp
                curr_index = ((curr_index+1)//2)-1
            else:
                break

    def delete_min(self):

        # swaps top and bottom of tree and pops off min
        min_node = self.queue[0].pop()
        self.queue[0], self.queue[len(self.queue) - 1] = self.queue[len(self.queue) - 1], self.queue[0]

        # bubble down to correct place
        curr_index = 0
        L = self.queue[((curr_index + 1) * 2) - 1]
        R = self.queue[((curr_index + 1) * 2) + 1 - 1]
        while (self.queue[curr_index][1] >= L[1]) and (self.queue[curr_index][1] >= R[1]):
            curr_node = self.queue[curr_index]
            parent = self.queue[((curr_index+1)//2)-1]
            if L[1] < curr_node[1]:
                if L[1] < R[1]:
                    temp = L
                    L = curr_node
                    curr_node = temp
                    curr_index = ((curr_index+1)*2)-1
            elif R[1] < curr_node[1]:
                if L[1] > R[1]:
                    temp = R
                    R = curr_node
                    curr_node = temp
                    curr_index = ((curr_index+1)*2)+1-1
            elif ((R and L) >= curr_node) and (parent <= curr_node):
                break
            L = self.queue[((curr_index+1)*2)-1]
            R = self.queue[((curr_index+1)*2)+1-1]

        return min_node

    def make_queue(self, nodes):
        self.queue = {key: float('inf') for key, _ in nodes}




pq = PriorityQueueArray()
# pq.insert('A', 2)
# pq.insert('B', 4)
# pq.insert('C', 2)
#
# print("Initial Priority Queue:", pq.queue)
#
# pq.decrease_dist("A", 3)
# print("After decreasing value of A:", pq.queue)
#
# min_node = pq.delete_min()
# print(f"Deleted minimum value: ({min_node})")
# print("Updated Priority Queue:", pq.queue)
#
elements = [("D", 1), ("E", 8), ("F", 4)]
pq.make_queue(elements)
print("Priority Queue after make_queue:", pq.queue)