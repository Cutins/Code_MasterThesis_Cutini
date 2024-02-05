import cvxpy as cp
from scipy.linalg import solve_discrete_are

from MPC.Parameters import *


class MPCController():

    def __init__(self, parameters) -> None:
        self.params = parameters
        self.Ap = self.params.Ap
        self.Bp = self.params.Bp
        self.Qo = self.params.Qo
        self.Ro = self.params.Ro

    def build_optcon_problem(self, t_hor, dt):
        ns = 3
        ni = 1
        num_steps = int(t_hor/dt)

        # x = [(pk-pr), (vk-vr), vk]
        self.x = cp.Variable((ns, num_steps), name='X')
        self.uk = cp.Variable((ni, num_steps-1), name='U') # u = uak

        self.x0 = cp.Parameter(ns, name='x0') # Initial condition MPC
        self.d = cp.Parameter(ns, name='Affine_term')

        self.tar = cp.Parameter(ns, name='target')

        constraints = []
        cost = 0.0

        constraints += [self.x[:,0] == self.x0]

        for t in range(0, num_steps-1):
            # Dynamics constraint
            constraints += [self.x[:,t+1] == self.params.Ap @ self.x[:,t] + self.params.Bp @ self.uk[:,t] + self.d]

            # Input constraints            
            constraints += [self.uk[0,t] <= 1.0,
                            self.uk[0,t] >= 0.0]

            # Safe-distance constraint
            # constraints += [self.x[0,t] >= self.params.safe_distance]
            
            # Cost function
            cost += cp.quad_form(self.x[:,t] - self.tar, self.Qo) + cp.quad_form(self.uk[:,t], self.Ro)

        self.prob = cp.Problem(cp.Minimize(cost), constraints)


    def solve_optcon_problem(self, x_meas, ar, x_tar):
        """ Solves optimal control problem for given initial condition """

        self.x0.value = x_meas
        
        self.tar.value = x_tar

        affine_term = np.array([0, -self.params.dt*ar, 0])

        self.d.value = affine_term

        self.prob.solve(solver = cp.OSQP) #, warm_start=True, verbose=True)
        # self.prob.solve(verbose=True)
        if self.prob.status != cp.OPTIMAL:
            print("Error in solving optimization problem", self.prob.status)
        
        # print("Optimal value:", self.prob.value)
            
        ## Generator of code in C
        # generate_code(self.prob, code_dir='C_generate_code')

        x_opt = self.x.value
        uk_opt = self.uk.value

        return x_opt, uk_opt   
    