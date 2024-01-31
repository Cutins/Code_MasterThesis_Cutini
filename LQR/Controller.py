import numpy as np
import cvxpy as cp
from scipy.linalg import solve_discrete_are


class LQRController():
    def __init__(self, parameters) -> None:
        self.params = parameters
        self.speed_matched = False
    
    def solve_LQR(self, Q, R):
        """Computes P and K matrices given A, B, Q, R"""
        
        P = solve_discrete_are(self.params.A, self.params.B, Q, R)
        K = np.linalg.solve(R + self.params.B.T @ P @ self.params.B, self.params.B.T @ P @ self.params.A)
        self.K = K
        self.P = P
        print("Riccati Matrix", P)
        print("Regulator", K)

    def track_LQR(self, Calculate_opt, u_opt, error, car_speed, input_limits):
        """Performs LQR tracking
        u = u_opt - K (error)"""
        
        if car_speed > 0.8*(car_speed - error[1]) and car_speed>6.0 and self.speed_matched == False:
            self.speed_matched = True
            print("---------------------")
            print("Catch maneuver accomplished")
            print("Position error: ", error[0])
            print("Velocity error: ", error[1])
            self.solve_LQR(Q=self.params.Q, R=self.params.R)

        if Calculate_opt == True:
            uak = - (self.K @ error)[0] + u_opt
        else:
            uak = - (self.K @ error)[0]

        # Bounds for input
        uak = np.clip(uak, input_limits[0], input_limits[1])

        return uak, self.speed_matched

    def solve_optcon_problem(self, x_init, kart_ref, dt, t_end):
        """Solves optimal control problem for given init condition"""
        ns = 2
        ni = 1
        num_steps = int(t_end/dt)

        x = cp.Variable((ns, num_steps))
        u = cp.Variable((ni, num_steps-1))

        constraints = []
        cost = 0.0

        # Initial state constraint
        constraints += [x[:,0] == x_init]

        for t in range(0, num_steps-1):
            # Dynamics constraints
            constraints += [self.xk[:,t+1] == self.A @ self.xk[:,t] + self.B @ self.uk[:,t]]

            cost += cp.quad_form(x[:,t] - kart_ref[:,t], self.params.Qo) + cp.quad_form(self.uk[:,t], self.Ro)

        prob = cp.Problem(cp.Minimize(cost), constraints)

        print("\n ------------------------------")
        print("Solving optimal control problem ...")
        
        prob.solve()
        if prob.status != cp.OPTIMAL:
            print("Error in solving optimization problem")
            print(prob.status)

        print(prob.status)
        x_opt = x.value
        u_opt = u.value

        return x_opt, u_opt