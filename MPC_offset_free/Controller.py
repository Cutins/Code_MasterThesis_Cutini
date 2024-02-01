import cvxpy as cp
import numpy as np
from scipy.linalg import solve_discrete_are


class MPCController():

    def __init__(self, parameters) -> None:
        self.params = parameters
        self.Ap = self.params.Ap
        self.Bp = self.params.Bp
        self.Qo = self.params.Qo
        self.Ro = self.params.Ro

        self.Bd = self.params.Bd
        self.C = self.params.C
        self.H = self.params.H

    def build_optcon_problem(self, t_hor, dt):
        ns = 3
        ni = 1
        num_steps = int(t_hor/dt)

        # x = [(pk-pr), (vk-vr), vk]
        self.x = cp.Variable((ns, num_steps), name='X')
        self.u = cp.Variable((ni, num_steps-1), name='U') # u = uak
        self.d = cp.Variable((ni, num_steps), name="Disturbance")

        self.x0 = cp.Parameter(ns, name='x0') # Initial condition MPC
        self.d0 = cp.Parameter(ni, name='d0')
        self.aff = cp.Parameter(ns, name='Affine_term')

        self.u_max = cp.Parameter(name='u_max')
        self.u_min = cp.Parameter(name='u_min')

        self.xt = cp.Parameter(ns, name='xt')
        self.ut = cp.Parameter(ni, name='ut')

        constraints = []
        cost = 0.0

        constraints += [self.x[:,0] == self.x0]
        constraints += [self.d[:,0] == self.d0]

        for t in range(0, num_steps-1):
            # Dynamics constraint
            constraints += [self.x[:,t+1] == self.Ap @ self.x[:,t] + self.Bp @ self.u[:,t] + self.aff + self.Bd @ self.d[:,t]]
            constraints += [self.d[:,t+1] == self.d[:,t]]

            # Input constraints            
            constraints += [self.u[0,t] <= self.u_max,
                            self.u[0,t] >= self.u_min]

            # Safe-distance constraint
            # constraints += [self.x[0,t] >= self.params.safe_distance]
            
            # Cost function
            cost += cp.quad_form(self.x[:,t] - self.xt, self.Qo) + cp.quad_form(self.u[:,t] - self.ut, self.Ro)

        self.prob = cp.Problem(cp.Minimize(cost), constraints)


    def solve_optcon_problem(self, x_est, d_est, ar, u_max, u_min, h_ref):
        """ Solves optimal control problem for given initial condition """

        self.x0.value = x_est
        self.d0.value = d_est
        
        self.u_max.value = u_max
        self.u_min.value = u_min

        affine_term = np.array([0, -self.params.dt*ar, 0])
        self.aff.value = affine_term

        # Solve to find (xt,ut)
        AA = np.block([[self.Ap - np.eye(3), self.Bp], 
                       [self.C @ self.H, np.zeros_like(self.Bp)]])

        bb = np.concatenate((-self.Bd @ d_est, h_ref ))

        AA_inv = np.linalg.pinv(AA) # Moore-Penrose pseudo-inverse matrix

        xx = np.dot(AA_inv, bb)
        self.xt.value = xx[:3]
        self.ut.value = np.array([xx[3]])
        
        # print("xt for MPC", self.xt.value)
        # print("ut for MPC", self.ut.value)

        self.prob.solve(solver = cp.OSQP) #, warm_start=True, verbose=True)
        # self.prob.solve(verbose=True)
        if self.prob.status != cp.OPTIMAL:
            print("Error in solving optimization problem", self.prob.status)
        
        # print("Optimal value:", self.prob.value)

        x_opt = self.x.value
        uk_opt = self.u.value

        return x_opt, uk_opt   
    