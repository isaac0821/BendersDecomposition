###############################################################################
#                Pareto-Optimal Cuts for Benders Decomposition                #
# Article: Accelerating Benders Decomposition: Algorithmic Enhancement and    #
#          Model Selection Criteria                                           #
# Authors: T. L. MAGNANTI, R. T. WONG (1981)                                  #
# Coding:  Lan Peng                                                           #
###############################################################################

# This script compares the performance of traditional Benders Decomposition
# and Benders Decomposition with Pareto-optimal cuts for Facility Location 
# Problem. The formulation is as follows
# 
# v = \min_{i = 1}^n \min_{j = 1}^m c_{ij} x_{ij} + \sum_{j = 1}^m d_j y_j
# s.t. \sum_{j = 1}^m x_{ij} \ge 1, \quad \forall i \in \{1, 2, \ldots, n\}
#      x_{ij} \le y_j, \quad \forall i \in \{1, 2, \ldots, n\}, j \{1, 2, \ldots, m\}
#      x_{ij} \ge 0, \quad \forall i \in \{1, 2, \ldots, n\}, j \{1, 2, \ldots, m\}
#      y_{j} \in \{0, 1\}, \quad \forall j \in \{1, 2, \ldots, m\}
#
# Notice that Gurobi is a god-like software, this script cannot beat gurobi
# even with the Pareto-optimal cuts.


import gurobipy as grb
import random

def generateInstance(
	n: "Number of CUSTOMERS",
	m: "Number of FACILITIES",
	instancePath: "Export path for instances" = "test.dat"
	) -> "Generate a facility location problem instance":

	# Cost for assigning customer i to facility j =============================
	c = {}
	for i in range(n):
		for j in range(m):
			c[i, j] = random.randint(50, 200)
	# Cost for opening facility j =============================================
	d = {}
	for j in range(m):
		d[j] = random.randint(1000, 3000)

	# Save to a local file ====================================================
	f = open(instancePath, "w")
	# Meta info
	f.write(str(m) + "\t" + str(n) + "\n")
	# Save d[j]
	for j in range(m):
		f. write(str(d[j]) + "\n")
	# Save c[i, j]
	for j in range(m):
		s = ""
		for i in range(n):
			s += str(c[i, j]) + "\t"
		f.write(s + "\n")
	f.close()

	return {
		'n': n,
		'm': m,
		'c': c,
		'd': d
	}

def readInstance(
	instancePath: "Import path for instance"
	) -> "Read an instance file from local":

	# Initialize ==============================================================
	c = {}
	d = {}

	# Read file ===============================================================
	f = open(instancePath, "r")
	meta = f.readline().split("\t")
	m = int(meta[0])
	n = int(meta[1])

	d = {}
	for i in range(m):
		d[i] = int(f.readline())

	c = {}
	for j in range(m):
		ci = f.readline().split("\t")
		for i in range(n):
			c[i, j] = int(ci[i])

	f.close()

	return {
		'n': n,
		'm': m,
		'c': c,
		'd': d
	}

def directSolveProblem(
	instance: "Facility location problem, with n, m, c, d"
	) -> "Use gurobi to the optimal ofv of the instance without Benders Decomposition":
	# Read instance ===========================================================
	n = instance['n']
	m = instance['m']
	c = instance['c']
	d = instance['d']

	# Directly use Gurobi =====================================================
	FL = grb.Model("Facility Location")
	x = {}
	for i in range(n):
		for j in range(m):
			x[i, j] = FL.addVar(vtype=grb.GRB.CONTINUOUS, obj=c[i, j])
	y = {}
	for j in range(m):
		y[j] = FL.addVar(vtype=grb.GRB.BINARY, obj=d[j])

	for i in range(n):
		FL.addConstr(grb.quicksum(x[i, j] for j in range(m)) >= 1)
	for i in range(n):
		for j in range(m):
			FL.addConstr(x[i, j] <= y[j])

	FL.update()
	FL.modelSense = grb.GRB.MINIMIZE
	FL.optimize()

	ofv = None
	masterY = []
	if (FL.status == grb.GRB.status.OPTIMAL):
		ofv = FL.getObjective().getValue()
		for j in y:
			if (y[j].x >= 0.9):
				masterY.append(j)

	return {
		'ofv': ofv,
		'y': masterY
	}


def masterProblem(
	instance: "Facility location problem, with n, m, c, d",
	paretoFlag: "True if enable Pareto-optimal cuts, False otherwise" = False,
	) -> "Use traditional Benders Decomposition to solve Facility Location Problem":
	# Read instance ===========================================================
	n = instance['n']
	m = instance['m']
	c = instance['c']
	d = instance['d']

	# Initialize model ========================================================
	master = grb.Model("Master-IP")
	x = {}
	for i in range(n):
		for j in range(m):
			x[i, j] = master.addVar(vtype=grb.GRB.CONTINUOUS, obj=c[i, j])
	y = {}
	for j in range(m):
		y[j] = master.addVar(vtype=grb.GRB.BINARY, obj=d[j])

	# Integer part no constraint ==============================================
	pass

	# Call back to add cuts ===================================================
	master._y = y
	def addCuts(model, where):
		if (where == grb.GRB.Callback.MIPSOL):
			y_sol = model.cbGetSolution(model._y)
			subY = {}
			for i in range(m):
				if (y_sol[i] >= 0.9):
					subY[i] = 1
				else:
					subY[i] = 0
			findCut = dualSubproblem(instance, subY, paretoFlag)
			if (findCut['type'] == "infeasible"):
				model.terminate()
			elif (findCut['type'] == "optimality"):
				print("Add optimality cut")
				subLam = findCut['lam']
				subPi = findCut['pi']
				model.cbLazy(grb.quicksum(c[i, j] * x[i, j] for i in range(n) for j in range(m)) 
					>= grb.quicksum(subLam[i] for i in range(n)) - grb.quicksum(subPi[i, j] * y[j] for i in range(n) for j in range(m)))
			elif (findCut['type'] == "feasibility"):
				print("Add feasibility cut")
				subLam = findCut['lam']
				subPi = findCut['pi']
				model.cbLazy(grb.quicksum(subLam[i] for i in range(n)) - grb.quicksum(subPi[i, j] * y[j] for i in range(n) for j in range(m)) <= 0)
		return

	master.Params.lazyConstraints = 1
	master.setParam('OutputFlag', 0)
	master.optimize(addCuts)

	ofv = None
	masterY = []
	if (master.status == grb.GRB.status.OPTIMAL):
		ofv = master.getObjective().getValue()
		for j in y:
			if (y[j].x >= 0.9):
				masterY.append(j)

	return {
		'ofv': ofv,
		'y': masterY
	}

def dualSubproblem(
	instance: "Facility location problem, with n, m, c, d",
	y: "Given integer solution",
	paretoFlag: "True if enable Pareto-optimal cuts, False otherwise" = False
	) -> "Calculate the dual of subproblem, which is a LP":

	# Read instance ===========================================================
	n = instance['n']
	m = instance['m']
	c = instance['c']
	d = instance['d']

	# Dual subproblem initialize ==============================================
	sub = grb.Model("Dual-LP")

	# fTy =====================================================================
	fTy = 0
	for j in range(m):
		fTy += d[j] * y[j]

	# Decision variables ======================================================
	lam = {}
	for i in range(n):
		lam[i] = sub.addVar(vtype=grb.GRB.CONTINUOUS)
	pi = {}
	for i in range(n):
		for j in range(m):
			pi[i, j] = sub.addVar(vtype=grb.GRB.CONTINUOUS)

	# Constraints =============================================================
	for i in range(n):
		for j in range(m):
			sub.addConstr(lam[i] - pi[i, j] <= c[i, j])
	sub.update()

	# Objective ===============================================================
	sub.setObjective(grb.quicksum(lam[i] - grb.quicksum(pi[i, j] * y[j] for j in range(m)) for i in range(n)) + fTy)

	# Pareto-optimal cut generation ===========================================
	def paretoOptimal():
		# Generate a core point
		y0 = {}
		for j in range(m):
			y0[j] = random.random()

		# First, define set O and set C for opened/closed facility
		O = []
		C = []
		for j in range(m):
			if (y[j] == 1):
				O.append(j)
			else:
				C.append(j)

		poLam = {}
		poPi = {}

		# Calculate for each customer i
		for i in range(n):
			# Get index where cij is minimum
			minIndex = None
			cMin = None
			for j in O:
				if (minIndex == None or c[i, j] <= cMin):
					cMin = c[i, j]
					minIndex = j
			# Calculate Li
			Li = None
			for j in O:
				if (j != minIndex and (Li == None or c[i, j] <= Li)):
					Li = c[i, j]

			if (Li == None):
				poLam[i] = cMin
				for j in O:
					poPi[i, j] = 0
				for j in C:
					poPi[i, j] = max(0, poLam[i] - c[i, j])
			else:
				# Step 1: Start with initial lambda i
				poLam[i] = cMin

				improveFlag = True
				while(improveFlag):
					improveFlag = False
					# Step 2: T, s
					T = []
					for j in C:
						if (c[i, j] <= poLam[i]):
							T.append(j)
					s = y[minIndex] - y0[minIndex]
					for j in T:
						s += y[j] - y0[j]
					if (s <= 0):
						# s <= 0, stop, poLam[i] is optimal
						pass
					elif(s > 0 and len(T) == len(C)):
						poLam[i] = Li
					else:
						ck = None
						for j in C:
							if (j not in T and (ck == None or c[i, j] <= ck)):
								ck = c[i, j]
						if (Li <= ck):
							poLam[i] = Li
						else:
							poLam[i] = ck
							improveFlag = True

				for j in range(m):
					if (j in O and j != minIndex):
						poPi[i, j] = 0
					elif (j == minIndex):
						poPi[i, j] = poLam[i] - cMin
					else:
						poPi[i, j] = max(0, poLam[i] - c[i, j])

		return {
			'lam': poLam,
			'pi': poPi
		}

	# Interpret result ========================================================
	sub.modelSense = grb.GRB.MAXIMIZE
	sub.setParam('OutputFlag', 0)
	sub.setParam("InfUnbdInfo", 1)
	sub.optimize()
	# If bounded, return optimality cut
	if (sub.status == grb.GRB.status.OPTIMAL):
		# Enable Pareto Optimality cuts
		if (paretoFlag):
			paretoCut = paretoOptimal()
			poLam = paretoCut['lam']
			poPi = paretoCut['pi']
			return {
				'type': "optimality",
				'lam': poLam,
				'pi': poPi
			}
		# Unable Pareto Optimality Cuts
		else:
			subLam = {}
			subPi = {}
			for i in range(n):
				subLam[i] = lam[i].x
			for i in range(n):
				for j in range(m):
					subPi[i, j] = pi[i, j].x
			return {
				'type': "optimality",
				'lam': subLam,
				'pi': subPi
			}
	# If unbounded, return feasibility cut
	elif (sub.status == grb.GRB.status.UNBOUNDED):
		subLam = {}
		subPi = {}
		ray = sub.UnbdRay
		for i in range(n):
			subLam[i] = ray[i]
		for i in range(n):
			for j in range(m):
				subPi[i, j] = ray[n + i * m + j]
		return {
			'type': "feasibility",
			'lam': subLam,
			'pi': subPi
		}

	elif (sub.status == grb.GRB.status.INFEASIBLE):
		return {
			'type': "infeasible",
			'lam': subLam,
			'pi': subPi
		}
