#!/usr/bin/python3


from CS312Graph import *
import time
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
        path_edges = []
        total_length = 0
        currIndex = destIndex
        while currIndex != self.source:
            parentIndex = self.prev[currIndex]
            currDist = self.dist[currIndex]
            try :
                path_edges.append((self.id_to_node[parentIndex].loc, self.id_to_node[currIndex].loc, '{:.0f}'.format(currDist)))
            except:
                return {'cost':"unreachable", 'path':None}
            total_length += currDist
            currIndex = parentIndex
        return {'cost':total_length, 'path':path_edges}

    # Time: O(𝐷elete𝑀in⋅|𝑉|+𝐷ecrease𝐾ey⋅|𝐸|)
    # Space: O(|𝑉|)
    def computeShortestPaths( self, srcIndex, use_heap=False ):
        # O(1) Initializing variables
        self.source = srcIndex
        t1 = time.time()
        pq = None
        if use_heap:
            pq = PriorityQueueBinaryHeap()
        else:
            pq = PriorityQueueArray()
        # O(𝑀ake𝑄ueue) varies based on implementation (nlogn for heap, n for array)
        pq.make_queue(self.network.nodes)
        srcNode = self.network.nodes[srcIndex].node_id
        # Time: O(𝐷elete𝑀in⋅|𝑉|+𝐷ecrease𝐾ey⋅|𝐸|)
        # Space: O(|𝑉|)
        self.dist, self.prev = self.Dijkstra(pq, srcNode)
        t2 = time.time()
        return (t2-t1)

# Finds shortest path from a source node
# Time: O(𝐷elete𝑀in⋅|𝑉|+𝐷ecrease𝐾ey⋅|𝐸|) runs delete minimum for every node, and delete key for every edge
# Space: O(|𝑉|) stores all nodes
    def Dijkstra(self, pq, srcNode):
        # O(1) Intializing variables
        prev = {}
        dist = {}
        # O(|𝑉|) Assign all nodes distances to infinity and prev to nothing
        for curr in self.network.nodes:
            dist[curr.node_id] = sys.maxsize
            prev[curr.node_id] = None
        # O(𝐷ecrease𝐾ey) varies based on implementation (logn for heap, n for array)
        pq.decrease_dist(srcNode,0)
        dist[srcNode] = 0
        while not pq.is_empty():
            # O(Delete𝑀in⋅|𝑉|) removes minimum for each node based on implementation (logn for heap, n for array)
            currIndex, currDist = pq.delete_min()
            currNode = self.id_to_node[currIndex]
            # O(𝐷ecrease𝐾ey * |E|) runs decrease distance on each edge to current node
            for destNode in currNode.neighbors:
                if destNode.dest.node_id in pq.distances:
                    if pq.distances.get(destNode.dest.node_id) > (currDist + destNode.length):
                        prev[destNode.dest.node_id] = currIndex
                        pq.decrease_dist(destNode.dest.node_id, (currDist + destNode.length))
                        dist[destNode.dest.node_id] = pq.distances[destNode.dest.node_id] - currDist
        # Space: O(|𝑉|) to store all nodes in graph with distances and previous nodes
        return dist, prev

# Priority Queue using an array implementation
# Time: O(|V|) runs for every node in worst case
# Space: O(|V|) holds every node and associated distance
class PriorityQueueArray:
    def __init__(self):
        # stores node and given distance
        self.distances = {}

    # O(1) simple check to assure PQ is empty
    def is_empty(self):
        if len(self.distances) == 0:
            return True
        else:
            return False

    # O(1) inserts node at the end of array
    def insert(self, node, dist):
        self.distances[node] = dist

    # O(1) modifies one of the distances to a node
    def decrease_dist(self, node, dist):
        self.distances[node] = dist

    # O(|V|) searches through all nodes (worst case) until finds minimum and deletes it
    def delete_min(self):
        min_node = min(self.distances, key=self.distances.get)
        currDist = self.distances[min_node]
        self.distances.pop(min_node)
        return min_node, currDist

    # O(|V|) stores in PQ for every node in graph, stores infinity as its distances
    def make_queue(self, nodes):
        for node in nodes:
            self.insert(node.node_id, sys.maxsize)

# Priority Queue using a binary heap implementation
# Time: O(|V|log|V|) runs for every node in worst case
# Space: O(|V|) holds every node and associated distance and index
class PriorityQueueBinaryHeap:
    def __init__(self):
        # store nodes in order
        self.queue = []
        # stores distances with node
        self.distances = {}
        # stores indices of pq with node
        self.indices = {}

    # O(1) simple check to assure PQ is empty
    def is_empty(self):
        if len(self.distances) == 0:
            return True
        else:
            return False

    # O(1) simple swap within all given private data members of nodes
    def swap(self, node1, node2):
        node1Index = self.indices.get(node1)
        node2Index = self.indices.get(node2)
        self.indices[node1] = node2Index
        self.indices[node2] = node1Index
        tempNode = self.queue[node1Index]
        self.queue[node1Index] = self.queue[node2Index]
        self.queue[node2Index] = tempNode

    # O(1) calculates index for left child
    def getLChildIndex(self, index):
        return ((index + 1) * 2) - 1

    # O(1) calculates index for right child
    def getRChildIndex(self, index):
        return ((index + 1) * 2) + 1 - 1

    # O(1) calculates index for parent
    def getParentIndex(self, index):
        return ((index+1)//2)-1

    # O(log|V|) finds proper place based on distance in tree by bubbling up from lower in the tree
    def bubbleUp(self, curr_index):
        while curr_index > 0:
            parent = self.queue[self.getParentIndex(curr_index)]
            curr_node = self.queue[curr_index]
            if self.distances[curr_node] < self.distances[parent]:
                self.swap(parent, curr_node)
                curr_index = self.getParentIndex(curr_index)
            else:
                break

    # O(log|V|) finds proper place based on distance in tree by bubbling down from higher in the tree
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
                        self.swap(L, curr_node)
                        curr_index = self.getLChildIndex(curr_index)
                elif (self.distances[R] < self.distances[curr_node]) and (self.distances[L] > self.distances[R]):
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

    # O(log|V|) inserts a node into tree and utilizes bubbleUp function to find proper place based on distance
    def insert(self, node, dist):
        self.queue.append(node)
        self.distances[node] = dist
        self.indices[node] = len(self.queue) - 1
        curr_index = len(self.queue) - 1
        self.bubbleUp(curr_index)

    # O(log|V|) decreases key of a node within the tree and utilizes bubbleUp function to find proper place based on distance
    def decrease_dist(self, node, dist):
        if dist < self.distances[node]:
            self.distances[node] = dist
        curr_index = self.indices[node]
        self.bubbleUp(curr_index)

    # O(log|V|) deletes minimum node in a tree and utilizes bubbleDown function to find proper place based on distance
    def delete_min(self):
        self.swap(self.queue[0], self.queue[len(self.queue) - 1])
        currMin = self.queue[len(self.queue) - 1]
        currDist = self.distances[currMin]
        min_node = self.queue.pop(len(self.queue) - 1)
        del self.distances[min_node]
        del self.indices[min_node]
        if len(self.queue) > 0:
            self.bubbleDown()

        return min_node, currDist

    # O(|V|log|V|) creates PQ by inserting every node(|V|) and intializing its distance to infinity, insert utilizes bubbleUp function (O(log|V|))
    def make_queue(self, nodes):
        for node in nodes:
            self.insert(node.node_id, sys.maxsize)
