import graphviz as gv

import random
import sys

import numba
import numpy as np
import queue as Queue

class Network():

	def __init__(self, N, K, p, verbosity=0):
		# Creat a uniform latice
		self.N = N # Number of nodes in the network
		self.K = K # Per vertex degree (number of links per node)
		self.p = p # Probability of a random connection

		self.verbosity = verbosity

		graph = gv.Graph(format='svg', engine='circo')

		# Add the nodes
		for i in range(N):
			graph.node(str(i))

		# Add the links
		self.edges = []
		self.kLimit = int(K/2)
		for i in range(N):
			for j in range(self.kLimit+1):
				nodeIdx = i + j
				nodeIdx = N + nodeIdx if nodeIdx < 0 else nodeIdx if nodeIdx < N else nodeIdx - N
				if i != nodeIdx:
					# Random connection
					if random.random() < p:
						# Generate random link
						randomConnectionIndices = list(range(N))
						del randomConnectionIndices[i]
						
						# Generate random index
						nodeIdx = randomConnectionIndices[random.randint(0, N-2)]

						graph.edge(str(i), str(nodeIdx))
						self.edges.append((i, nodeIdx))
					else:
						graph.edge(str(i), str(nodeIdx))
						self.edges.append((i, nodeIdx))

					if self.verbosity > 0:
						print ("Edge added between %d and %d" % (i, nodeIdx))

		# Write the graph to file
		fileName = graph.render(filename='network_%d_%d_%.2f' % (N, K, p))
		print ("Graph written to file: %s" % fileName)

	def computeGraphStatistics(self):
		# Compute graph stats
		C = 0
		L = 0
		for i in range(self.N):
			C += self.clusteringCoefficient(i)

		numEdges = 0
		for firstNode in range(self.N):
			for secondNode in range(firstNode, self.N):
				L += self.characteristicPathLength(firstNode, secondNode)
				numEdges += 1

		# Normalize the values
		C = C / self.N
		# L = L / (numEdges * (numEdges - 1))
		L = L / float(self.N * (self.N - 1))

		if self.verbosity > -1:
			# Print graph stats
			print ("Number of nodes in graph: %d" % self.N)
			print ("Degree per vertex: %d" % self.K)
			print ("Computed clustering coefficient: %f" % C)
			print ("Computed characteristic path length: %f" % L)

		return L, C

	@numba.jit
	def getAdjacentNodes(self, nodeNum):
		# Iterate over all the nodes adjacent to the current node
		adjacentNodes = []
		for edge in self.edges:
			if (nodeNum == edge[0]):
				adjacentNodes.append(edge[1])
			elif (nodeNum == edge[1]):
				adjacentNodes.append(edge[0])

		return adjacentNodes

	# Determines the connectedness of a local region
	@numba.jit
	def clusteringCoefficient(self, nodeNum):
		adjacentNodes = self.getAdjacentNodes(nodeNum)
		numAdjacentNodes = len(adjacentNodes)
		numConnectedAdjacentNodes = 0
		for i in range(len(adjacentNodes) - 1):
			for j in range(i+1, len(adjacentNodes)):
				if ((adjacentNodes[i], adjacentNodes[j]) in self.edges) or ((adjacentNodes[j], adjacentNodes[i]) in self.edges):
					numConnectedAdjacentNodes += 1
		cc = float(numConnectedAdjacentNodes) / numAdjacentNodes

		if self.verbosity > 1:
			print ("Node: %d | Adjacent Nodes: %s" % (nodeNum, str(adjacentNodes)))
		return cc

	def insertOrUpdateItemInQueue(self, q, tupleVal):
		for itemIdx, item in enumerate(q.queue):
			if item[1] == tupleVal[1]:
				# Replace the distance value
				q.queue[itemIdx][0] = tupleVal[0]
				return

		# If not found
		q.put(tupleVal)

	# Shortest path algorithm for computing minimum distance the different nodes
	def dijstraShortestPathAlgorithm(self, sourceNode, destinationNode):
		visitedSet = [sourceNode]
		distances = {}
		for i in range(self.N):
			if i == sourceNode:
				distances[i] = 0
			else:
				distances[i] = sys.float_info.max
		q = Queue.PriorityQueue()
		q.put((0, sourceNode)) # first element in the tuple is the node distance while the second number is the node number

		while not q.empty():
			# Get the item with the lest distance
			currentNode = q.get(True)

			# Add to the visited list
			visitedSet.append(currentNode[1])
			
			# Get all adjacent nodes to the current node
			adjacentNodes = self.getAdjacentNodes(currentNode[1])

			# Iterate over all the neighbours to update their values
			for adjacentNode in adjacentNodes:
				# Update only if the node has not yet been visited
				if adjacentNode not in visitedSet:
					if currentNode[0] + 1 < distances[adjacentNode]:
						distances[adjacentNode] = currentNode[0] + 1 # Used 1 as the graph is unweighted
					
					# Add to the priority queue
					# q.put((distances[adjacentNode], adjacentNode))
					self.insertOrUpdateItemInQueue(q, [distances[adjacentNode], adjacentNode])

		if self.verbosity > 1:
			print ("Dijskstra's Algorithm completed")
			print (distances)

		return distances[destinationNode]

	# Determines the minimum number of edges to be traversed to reach the second node from the first node
	@numba.jit
	def characteristicPathLength(self, firstNode, secondNode):
		distance = self.dijstraShortestPathAlgorithm(firstNode, secondNode)
		if self.verbosity > 1:
			print ("Distance between %d and %d is %d" % (firstNode, secondNode, distance))
		return distance