import Network
import numpy as np

import matplotlib.pyplot as plt
import matplotlib as mpl

N = 20
K = 4
pMax = 1.0
pDelta = 0.02

# Generate the graph
pVals = []
lVals = []
cVals = []
for p in list(np.arange(0, pMax, pDelta)):
	net = Network.Network(N=N, K=K, p=p)
	L, C, rho = net.computeGraphStatistics()
	pVals.append(p)
	lVals.append(L)
	cVals.append(C)

# Normalize the values using C(0) and L(0)
C_zero = cVals.pop(0)
L_zero = lVals.pop(0)
pVals.pop(0)
for i in range(len(pVals)):
	cVals[i] = cVals[i] / float(C_zero)
	lVals[i] = lVals[i] / float(L_zero)

# Add the plot
mpl.style.use('seaborn')
fig, ax = plt.subplots()
ax.set_title('Small-world Networks', color='C0')
ax.set_xlabel('Probability')
ax.set_ylabel('Value')

ax.plot(pVals, lVals, 'C0', label='L', linewidth=2.0)
ax.plot(pVals, cVals, 'C1', label='C', linewidth=2.0)
ax.legend()

plt.savefig('./small_world_network.png', dpi=300)