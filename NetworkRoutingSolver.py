#!/usr/bin/python3


from CS312Graph import *
import time
import math
import sys


class NetworkRoutingSolver:
    def __init__( self):
        dist = {}
        prev = {}
        pass

    def initializeNetwork( self, network ):
        assert( type(network) == CS312Graph )
        self.network = network
        self.id_to_node = {node.node_id:node for node in self.network.nodes}

    def getShortestPath( self, destIndex ):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        path_edges = []
        total_length = 0
        # node = self.network.nodes[self.source]
        # node = self.network.nodes[destIndex]
        currIndex = destIndex
        # parentIndex = currIndex
        # edges_left = len(self.prev)
        while currIndex != self.source:
            # if parentIndex in self.prev and currIndex in self.prev:
            parentIndex = self.prev[currIndex]
            # path_edges.append((self.id_to_node[parentIndex].loc, self.id_to_node[currIndex].loc, '{:.0f}'.format(math.sqrt((self.id_to_node[currIndex].loc.x()-self.id_to_node[parentIndex].loc.x())**2 + (self.id_to_node[currIndex].loc.y()-self.id_to_node[parentIndex].loc.y())**2))))
            path_edges.append((self.id_to_node[parentIndex].loc, self.id_to_node[currIndex].loc, '{:.0f}'.format(math.sqrt((self.id_to_node[currIndex].loc.x()-self.id_to_node[parentIndex].loc.x())**2 + (self.id_to_node[currIndex].loc.y()-self.id_to_node[parentIndex].loc.y())**2))))
            total_length += self.dist[currIndex]
            currIndex = parentIndex
            # total_length += ((self.id_to_node[currIndex].loc.x()-self.id_to_node[parentIndex].loc.x())**2 + (self.id_to_node[currIndex].loc.y()-self.id_to_node[parentIndex].loc.y())**2)

            # total_length += self.id_to_node[parentIndex].length

            # edge = node.neighbors[2]
            # nextNode = self.prev[destIndex]
            # path_edges.append((node.loc, nextNode.loc, '{:.0f}'.format(edge.length)))
            # total_length += edge.length
            # node = edge.dest
            # edges_left -= 1
        # return {'cost':self.dist[destIndex], 'path':path_edges}
        return {'cost':total_length, 'path':path_edges}


    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()
        pq = None
        if use_heap:
            pq = PriorityQueueBinaryHeap()
        else:
            pq = PriorityQueueArray()
        pq.make_queue(self.network.nodes)
        srcNode = self.network.nodes[srcIndex].node_id
        self.dist, self.prev = self.Dijkstra(pq, srcNode)
        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)

        t2 = time.time()
        return (t2-t1)

    def Dijkstra(self, pq, srcNode):
        prev = {}
        dist = {}
        for curr in self.network.nodes:
            dist[curr.node_id] = None
            prev[curr.node_id] = None
        pq.decrease_dist(srcNode,0)
        dist[srcNode] = 0
        while not pq.is_empty():
            currIndex, currDist = pq.delete_min()
            currNode = self.id_to_node[currIndex]
            for destNode in currNode.neighbors:
                if destNode.dest.node_id in pq.distances:
                    if pq.distances.get(destNode.dest.node_id) > (currDist + math.sqrt((currNode.loc.x()-destNode.dest.loc.x())**2 + (currNode.loc.y()-destNode.dest.loc.y())**2)):
                        prev[destNode.dest.node_id] = currIndex
                        pq.decrease_dist(destNode.dest.node_id, (currDist + math.sqrt((currNode.loc.x()-destNode.dest.loc.x())**2 + (currNode.loc.y()-destNode.dest.loc.y())**2)))
                        dist[destNode.dest.node_id] = currDist + math.sqrt((currNode.loc.x()-destNode.dest.loc.x())**2 + (currNode.loc.y()-destNode.dest.loc.y())**2)
        return dist, prev


class PriorityQueueArray:
    def __init__(self):
        self.distances = {}

    def is_empty(self):
        if len(self.distances) == 0:
            return True
        else:
            return False
        # return bool(self.distances)

    def insert(self, node, dist):
        self.distances[node] = dist
        # self.queue[node,dist]

    def decrease_dist(self, node, dist):
        if node in self.distances:
            if dist < self.distances[node]:
                self.distances[node] = dist
        # else:
        #     print("Key not found in the priority queue.")

    def delete_min(self):
        # if not self.distances:
        #     print("Priority queue is empty.")
        #     return None

        min_node = min(self.distances, key=self.distances.get)
        currDist = self.distances[min_node]
        self.distances.pop(min_node)
        return min_node, currDist

    # def make_queue(self, solver: NetworkRoutingSolver):
    #     for node in solver.network:
    #         self.queue[node] = (float('inf'))
    # def make_queue(self, nodes):
    #     for node in nodes:
    #         self.queue[node] = (float('inf'))
    def make_queue(self, nodes):
        # self.queue = {key: float('inf') for key, _ in nodes}
        # self.queue = dict(nodes)
        for node in nodes:
            # self.insert(node.node_id, float('inf'))
            self.insert(node.node_id, sys.maxsize)


class PriorityQueueBinaryHeap:
    def __init__(self):
        # store nodes in order
        self.queue = []
        # stores distances with node
        self.distances = {}
        # stores indices of pq with node
        self.indices = {}

    def is_empty(self):
        if len(self.distances) == 0:
            return True
        else:
            return False
        # return bool(self.distances)

            # def swap(self, parent, child):
    #     parentIndex = self.indices.get(parent)
    #     childIndex = self.indices.get(child)
    #     self.indices[parent] = childIndex
    #     self.indices[child] = parentIndex
    #     tempParentNode = self.queue[parentIndex]
    #     self.queue[parentIndex] = self.queue[childIndex]
    #     self.queue[childIndex] = tempParentNode

    def swap(self, node1, node2):
        node1Index = self.indices.get(node1)
        node2Index = self.indices.get(node2)
        self.indices[node1] = node2Index
        self.indices[node2] = node1Index
        tempNode = self.queue[node1Index]
        self.queue[node1Index] = self.queue[node2Index]
        self.queue[node2Index] = tempNode

    def getLChildIndex(self, index):
        return ((index + 1) * 2) - 1

    def getRChildIndex(self, index):
        return ((index + 1) * 2) + 1 - 1

    def getParentIndex(self, index):
        return ((index+1)//2)-1

    def bubbleUp(self, curr_index):
        while curr_index > 0:
            parent = self.queue[self.getParentIndex(curr_index)]
            curr_node = self.queue[curr_index]
            if self.distances[curr_node] < self.distances[parent]:
                self.swap(parent, curr_node)
                curr_index = self.getParentIndex(curr_index)
            else:
                break

    def bubbleDown(self):
        curr_index = 0
        curr_node = self.queue[curr_index]
        if len(self.queue) <= self.getLChildIndex(curr_index) or len(self.queue) <= self.getRChildIndex(curr_index):
            return
        L = self.queue[self.getLChildIndex(curr_index)]
        R = self.queue[self.getRChildIndex(curr_index)]
        while True:
            if R is not None:
                if (self.distances[L] < self.distances[curr_node]) and (self.distances[L] < self.distances[R]):
                    # if self.distances[L] < self.distances[R]:
                        self.swap(L, curr_node)
                        curr_index = self.getLChildIndex(curr_index)
                elif self.distances[R] < self.distances[curr_node] and (self.distances[L] > self.distances[R]):
                    # if self.distances[L] > self.distances[R]:
                        self.swap(R, curr_node)
                        curr_index = self.getRChildIndex(curr_index)
                elif (self.distances[curr_node] <= self.distances[L]) and (self.distances[curr_node] <= self.distances[R]):
                    break
                curr_node = self.queue[curr_index]
                if len(self.queue) <= self.getLChildIndex(curr_index):
                    break
            else:
                if self.distances[L] < self.distances[curr_node]:
                    self.swap(L, curr_node)
                    curr_index = self.getLChildIndex(curr_index)
                elif self.distances[curr_node] <= self.distances[L]:
                    break
                curr_node = self.queue[curr_index]
                if len(self.queue) <= self.getLChildIndex(curr_index):
                    break
            L = self.queue[self.getLChildIndex(curr_index)]
            if (len(self.queue) > self.getRChildIndex(curr_index)):
                R = self.queue[self.getRChildIndex(curr_index)]
            else:
                R = None


    def insert(self, node, dist):
        self.queue.append(node)
        self.distances[node] = dist
        self.indices[node] = len(self.queue) - 1
        curr_index = len(self.queue) - 1
        self.bubbleUp(curr_index)

    def decrease_dist(self, node, dist):
        # if node in self.queue:
        if dist < self.distances[node]:
            self.distances[node] = dist
        # curr_index = len(self.queue) - 1
        curr_index = self.indices[node]
        self.bubbleUp(curr_index)

    def delete_min(self):
        self.swap(self.queue[0], self.queue[len(self.queue) - 1])
        currMin = self.queue[len(self.queue) - 1]
        currDist = self.distances[currMin]
        min_node = self.queue.pop(len(self.queue) - 1)
        # currDist = self.distances[min_node]
        del self.distances[min_node]
        del self.indices[min_node]
        if len(self.queue) > 0:
            self.bubbleDown()

        return min_node, currDist

    def make_queue(self, nodes):
        # self.queue = {key: float('inf') for key, _ in nodes}
        for node in nodes:
            # self.insert(node.node_id, float('inf'))
            self.insert(node.node_id, sys.maxsize)
            # self.insert(node, sys.maxsize)


# pq = PriorityQueueArray()
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
# elements = [("D", 1), ("E", 8), ("F", 4)]
# pq.make_queue(elements)
# print("Priority Queue after make_queue:", pq.queue)

# pq = PriorityQueueBinaryHeap()
#
# # pq.make_queue(["a", "b", "c", "d"])
# pq.insert('9',9)
# pq.insert('8',8)
# pq.insert('3',3)
# pq.insert('5',5)
# pq.insert('6',6)
# pq.insert('4',4)
# pq.insert('2',2)
# pq.insert('1',1)
# pq.insert('10',10)
# pq.insert('14',14)
# pq.insert('7',7)
# pq.insert('12',12)
# pq.delete_min()
# pq.delete_min()
# pq.delete_min() #youch
# pq.delete_min()
# pq.delete_min()
# pq.delete_min()
# pq.delete_min()
# pq.delete_min()
# pq.delete_min()
# pq.delete_min()



# pq.decrease_dist('b', .5)

