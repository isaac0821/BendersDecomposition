# BendersDecomposition
Learning Benders Decomposition by Coding


In this repo, a python script is written for implementing the Pareto-optimal cuts described in the following article

*Accelerating Benders Decomposition: Algorithmic Enhancement and Model Selection Criteria*

by **T. L. MAGNANTI**, **R. T. WONG**

for a facility location problem as follows

 v = \min_{i = 1}^n \min_{j = 1}^m c_{ij} x_{ij} + \sum_{j = 1}^m d_j y_j
 s.t. \sum_{j = 1}^m x_{ij} \ge 1, \quad \forall i \in \{1, 2, \ldots, n\}
      x_{ij} \le y_j, \quad \forall i \in \{1, 2, \ldots, n\}, j \{1, 2, \ldots, m\}
      x_{ij} \ge 0, \quad \forall i \in \{1, 2, \ldots, n\}, j \{1, 2, \ldots, m\}
      y_{j} \in \{0, 1\}, \quad \forall j \in \{1, 2, \ldots, m\}
      