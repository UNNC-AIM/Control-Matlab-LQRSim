import cvxpy as cvx
import numpy as np
import scipy.linalg as spalg
import matplotlib.pyplot as plt

class Balance:
    def __init__(self):
        self.M = 28.52
        self.l = 0.3658
        self.Jyy = 0.654
        self.COM = np.array([1.2, 0, 0])
        self.g = 9.80665
        self.b = 0.12
        
        self.A = np.array([
            [                 0,                  1, 0,       0      ],
            [(self.M * self.g * self.l)/self.Jyy, 0, 0,       0      ],
            [                 0,                  0, 0,       1      ],
            [                 0,                  0, 0, self.b/self.M]
        ])
        self.B = np.array([
            [     0    ], 
            [1/self.Jyy], 
            [     0    ], 
            [ 1/self.M ]
        ])

    def problem_solve(self, Q, R, Omega):
        # P = cvx.Variable((4, 4), symmetric=True)
        X = cvx.Variable((4, 4), symmetric=True)
        Z = cvx.Variable((1, 1))
        Y = cvx.Variable((1, 4))
        A = self.A
        B = self.B
        # SchurComplement = cvx.bmat([
        #     [A.T @ P + P @ A - Q, P @ B],
        #     [     B.T @ P,         -R  ]
        # ])
        SchurComplement = cvx.bmat([
            [         Z,            spalg.sqrtm(R) @ Y],
            [ Y.T @ spalg.sqrtm(R),           X       ]
        ])
        target = cvx.trace(spalg.sqrtm(Q) @ X @ spalg.sqrtm(Q) + cvx.trace(Z))
        # obj = cvx.Minimize(cvx.trace(P))
        obj = cvx.Minimize(target)

        # constraints = [P >> 0]
        # constraints += [SchurComplement << 0]
        constraints = [A @ X - B @ Y + X @ A.T - Y.T @ B.T + Omega == 0]
        constraints += [SchurComplement >> 0]

        LMI_Problem = cvx.Problem(obj, constraints)
        LMI_Problem.solve(solver=cvx.CVXOPT, verbose=True, type='SDP')
        
        print("Solver Status:", LMI_Problem.status)
        print("Solver Details:", LMI_Problem.solver_stats)
        
        # self.K = np.linalg.inv(R) @ B.T @ P.value
        self.K = Y.value @ np.linalg.inv(X.value)
        
        print("Stability:\n", np.linalg.eig(A - B @ self.K).eigenvalues)


    def simulate(self, step, stop):
        dt = step  # Time step
        t_final = stop  # Final simulation time

        # Initialize simulation variables
        t = np.arange(0, t_final, dt)
        num_steps = len(t)

        x = np.zeros((4, num_steps))  # State vector
        # u = np.ones((1, num_steps))

        x[:,0] = np.array([20/180 * np.pi, 0, 0, 0])
    

        # Iteration loop
        for i in range(1, num_steps):
            x[:, i] = spalg.expm(np.dot((self.A - self.B @ self.K), t[i])) @ x[:, 0] # + sys.B @ u[:, i-1]

        # Plot the results
        plt.subplot(2, 2, 1)
        plt.plot(t, x[0], label='Angle(Rad)')
        plt.legend()

        plt.subplot(2, 2, 2)
        plt.plot(t, x[1], label='AngularRate(Rad/s)')
        plt.legend()

        plt.subplot(2, 2, 3)
        plt.plot(t, x[2], label='Pos(m)')
        plt.legend()

        plt.subplot(2, 2, 4)
        plt.plot(t, x[3], label='Vel(m/s)')
        plt.suptitle('State Space Response')

        plt.legend()

        # plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    sys = Balance()
    Q = np.diag([.01, .01, 1, .01])
    R = .01 * np.eye(1)
    omega = np.ones((4,4))

    sys.problem_solve(Q, R, omega)
    sys.simulate(.001, 20)
