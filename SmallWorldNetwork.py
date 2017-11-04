import graphviz as gv

# Creat a uniform latice
N = 20 # Number of nodes in the network
K = 4 # Per vertex degree (number of links per node)

graph = gv.Graph(format='svg', engine='circo')

# Add the nodes
for i in range(N):
	graph.node(str(i))

# Add the links
edges = []
kLimit = int(K/2)
for i in range(N):
	for j in range(kLimit+1):
		nodeIdx = i + j
		nodeIdx = N + nodeIdx if nodeIdx < 0 else nodeIdx if nodeIdx < N else nodeIdx - N
		if i != nodeIdx:
			print ("Edge added between %d and %d" % (i, nodeIdx))
			graph.edge(str(i), str(nodeIdx))
			edges.append((i, nodeIdx))

# Determines the connectedness of a local region
def clusteringCoefficient(nodeNum):
	# Iterate over all the nodes adjacent to the current node
	adjacentNodes = []
	for edge in edges:
		if (nodeNum == edge[0]):
			adjacentNodes.append(edge[1])
		elif (nodeNum == edge[1]):
			adjacentNodes.append(edge[0])

	numAdjacentNodes = len(adjacentNodes)
	numConnectedAdjacentNodes = 0
	for i in range(len(adjacentNodes) - 1):
		for j in range(i+1, len(adjacentNodes)):
			if ((adjacentNodes[i], adjacentNodes[j]) in edges) or ((adjacentNodes[j], adjacentNodes[i]) in edges):
				numConnectedAdjacentNodes += 1
	cc = float(numConnectedAdjacentNodes) / numAdjacentNodes

	print ("Node: %d | Adjacent Nodes: %s" % (nodeNum, str(adjacentNodes)))
	return cc

# Determines the minimum number of edges to be traversed to reach the second node from the first node
def characteristicPathLength(firstNode, secondNode):
	nodeDiff = abs(firstNode - secondNode)
	if nodeDiff > int(N / 2):
		nodeDiff = N - nodeDiff
	distance = 0
	if nodeDiff == 0:
		distance = 0
	elif nodeDiff < kLimit:
		distance = 1
	else:
		distance = nodeDiff - kLimit + 1

	print ("Distance between %d and %d is %d" % (firstNode, secondNode, distance))
	return distance

# Compute graph stats
C = 0
L = 0
for i in range(N):
	C += clusteringCoefficient(i)

numEdges = 0
for firstNode in range(N):
	for secondNode in range(firstNode, N):
		L += characteristicPathLength(firstNode, secondNode)
		numEdges += 1

# Normalize the values
C = C / N
# L = L / (numEdges * (numEdges - 1))
L = L / float(N * (N - 1))

filename = graph.render(filename='test')

# Print graph stats
print ("Number of nodes in graph: %d" % N)
print ("Degree per vertex: %d" % K)
print ("Computed clustering coefficient: %f" % C)
print ("Computed characteristic path length: %f" % L)