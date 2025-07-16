import numpy as np
from cvxopt import matrix, solvers

class RCI_QP_Solver:
    def __init__(self, n_var):
        # Ax = b
        self.TaskEq_A = None
        self.TaskEq_b = None

        # Gx <= h
        self.TaskIneq_G = None
        self.TaskIneq_h = None

        # 0.5 x^T P x + q^T x
        self.Cost_P = None
        self.Cost_q = None

        # Variables
        self.n_var = n_var


    def AddEqTask(self, taskA : np.array , taskb : np.array):
        self.TaskEq_A = np.copy(taskA)
        self.TaskEq_b = np.copy(taskb)

    def AddIneqTask(self, taskG : np.array, taskh : np.array):
        self.TaskIneq_G = np.copy(taskG)
        self.TaskIneq_h = np.copy(taskh)

    def AddCost(self, CostP : np.array, Costq : np.array):
        self.Cost_P = np.copy(CostP)
        self.Cost_q = np.copy(Costq)

    def solveQP(self):
        P = matrix(self.Cost_P, tc='d')
        q = matrix(self.Cost_q, tc='d')

        if self.TaskIneq_G is not None and self.TaskIneq_h is not None:
            G = matrix(self.TaskIneq_G, tc='d')
            h = matrix(self.TaskIneq_h, tc='d')
        else:
            G = matrix(0.0, (0, self.n_var))
            h = matrix(0.0, (0,1))

        if self.TaskEq_A is not None and self.TaskEq_b is not None:
            A = matrix(self.TaskEq_A, tc='d')
            b = matrix(self.TaskEq_b, tc='d')
        else:
            A = matrix(0.0, (0, self.n_var))
            b = matrix(0.0, (0,1))

        sol = solvers.qp(P,q,G,h,A,b)

        x_opt = np.array(sol['x']).flatten()

        return x_opt
