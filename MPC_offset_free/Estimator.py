import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import place_poles

class Estimator():
    
    def __init__(self, parameters) -> None:
        self.params = parameters
        self.Ap = self.params.Ap
        self.Bp = self.params.Bp
        self.Bd = self.params.Bd

        # Augmented system matrices
        self.Aaug = np.block([[self.Ap, self.Bd], 
                         [np.zeros_like(self.Bp.T), 1.0]])
        
        self.Baug = np.block([[self.Bp],
                         [0.0]])
        
        self.Caug = np.block([[np.eye(3), np.zeros_like(self.Bp)]])

    def observer(self):

        # print("A augmented", self.Aaug)
        # print("B augmented", self.Baug)
        # print("C augmented", self.Caug)

        # desired_poles = [0.22, 0.23, 0.21, 0.1]
        desired_poles = [0.7, 0.6, 0.75, 0.8]
        desired_poles = [0.5, 0.51, 0.52, 0.53]


        eigenAp = np.linalg.eigvals(self.Ap)
        # print("Eigenvalues Ap: ", eigenAp)

        eigenAaug = np.linalg.eigvals(self.Aaug)
        # print("Eigenvalues A augmented: ", eigenAaug)

        self.L = -place_poles(self.Aaug.T, self.Caug.T, desired_poles).gain_matrix.T

        Controlled = self.Aaug + self.L @ self.Caug
        eigenControlled = np.linalg.eigvals(Controlled)
        # print("Eigenvalues Controlled: ", eigenControlled)


        # t = np.linspace(0,2*np.pi, 401)
        # plt.plot(np.cos(t), np.sin(t), 'k--') # unit circle
        # plt.plot(eigenAp, np.zeros(eigenAp.shape), 'bx', label='Ap poles')
        # # plt.plot(eigenAaug, np.zeros(eigenAaug.shape), 'rx', label='Ae poles')
        # plt.plot(eigenControlled, np.zeros(eigenControlled.shape), 'mx', label='ACrtl poles')
        # plt.grid()
        # plt.axis([-1.1, 1.1, -1.1, 1.1])
        # plt.legend(loc="lower left")
        # plt.show()

        return self.L
    
    def estimate(self, x_est, d_est, ar, h_meas, u_t):

        aff_aug = np.array([[0],
                            [-self.params.dt*ar],
                            [0],
                            [0]]).squeeze()
        
        est = np.concatenate((x_est, d_est)).squeeze()
        
        est_plus = self.Aaug @ est + aff_aug + self.Baug @ u_t + self.L @ (x_est - h_meas)

        x_est_plus = est_plus[:3]
        d_est_plus = est_plus[3]

        return x_est_plus, d_est_plus