import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.linalg import solve_continuous_are
import random

class FurataPendulum:

    def __init__(self, dt, m1, l1, L1, J1xx, J1yy, J1zz, m2, l2, L2, J2xx, J2yy, J2zz, b1, b2):

        self._dt = dt
        self._sim_time = 0.0
        self._reset_interval = 0.0

        """ Dynamics init"""
        # Use matrices for easy storage
        self._b1 = b1
        self._m1 = m1
        self._len1 = np.array([l1, L1])
        self._J1 = np.array([J1xx, J1yy, J1zz])

        self._b2 = b2
        self._m2 = m2
        self._len2 = np.array([l2, L2])
        self._J2 = np.array([J2xx, J2yy, J2zz])

        self._lqr_K = None

        # I WANT STATE AND POSITIONS TO BE MEMBER VARS
        self._q = np.array([0.0, 0.0, 0.0, 0.0])  # Default to 0 ICs
        self._fwd_kin() # Initialize link positions

        """ Animation Init """
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        limitor = np.max(np.array([l1, L1, l2, L2]))
        self.ax.set_xlim([-1.25*limitor, 1.25*limitor])
        self.ax.set_ylim([-1.25*limitor, 1.25*limitor])
        self.ax.set_zlim([-1.25*limitor, 1.25*limitor])

        # Lines for links
        self.link1, = self.ax.plot([], [], [], lw=3, c='b')
        self.link2, = self.ax.plot([], [], [], lw=3, c='r')

        # Animation
        self.ani = FuncAnimation(self.fig, self.update_sim, interval=dt*1000, blit=True)
    
    def _fwd_kin(self):
        """ Compute link positions geometrically given a state """
        t1 = self._q[0]
        t2 = self._q[1] # Don't need velo

        # Derived offline
        L1 = self._len1[1]
        L2 = self._len2[1]

        self._x1 = np.array([ L1*np.cos(t1), L1*np.sin(t1), 0.0 ])
        self._x2 = np.array([ L1*np.cos(t1) - L2*np.sin(t1)*np.sin(t2), L1*np.sin(t1) + L2*np.cos(t1)*np.sin(t2), -L2*np.cos(t2) ])

    
    def _compute_qd(self, q_cur, tau):
        # Have this func still take in q, so can easily pass diff values (i.e. for RK)
        g = 9.81 # [m/s^2]

        # Parse into easy vars
        t1 = q_cur[0]
        t2 = q_cur[1]
        t1d = q_cur[2]
        t2d = q_cur[3]

        damp1 = self._b1
        m1 = self._m1
        l1 = self._len1[0]
        L1 = self._len1[1]
        J1xx = self._J1[0]
        J1yy = self._J1[1]
        J1zz = self._J1[2]

        damp2 = self._b2
        m2 = self._m2
        l2 = self._len2[0]
        L2 = self._len2[1]
        J2xx = self._J2[0]
        J2yy = self._J2[1]
        J2zz = self._J2[2]

        # Calculate Accels
        T = np.array([tau, 0])

        b1 = m2*L1*l2*np.sin(t2)*(t2d**2) + t1d*t2d*np.sin(2*t2)*(m2*(l2**2) + J2yy + J2xx) + damp1*t1d
        b2 = 0.5*(t1**2)*np.sin(2*t2)*(-m2*(l2**2)-J2yy+J2xx) + g*m2*l2*np.sin(t2) + damp2*t2d
        b = np.array([b1, b2])

        N11 = J1zz + m1*(l1**2) + m2*(L1**2) + (J2yy+m2*(l2**2))*((np.sin(t2))**2) + J2xx*((np.cos(t2))**2)
        N12 = m2*L1*l2*np.cos(t2)
        N21 = m2*L1*l2*np.cos(t2)
        N22 = m2*(l2**2) + J2zz
        N = np.array([ [N11, N12],
                       [N21, N22] ])
        
        td_cur = np.array([t1d, t2d])
        # print(f"td_cur = {td_cur}")
        tdd_cur = (np.linalg.inv(N)) @ (T - b)
        # print(f"tdd_cur = {tdd_cur}")

        return np.concatenate( (td_cur, tdd_cur) )
    
    def _step(self, tau, method = "RK4"):

        # Extract 
        dt = self._dt
        q_cur = self._q

        # Func for taking a time step per the dynamics equations and desired integration method
        qd_cur = self._compute_qd(q_cur, tau)
        if method == "Euler":
            q_new = qd_cur*dt + q_cur
        elif method == "RK4":
            k1 = qd_cur
            k2 = self._compute_qd(q_cur + 0.5*dt*k1, tau)
            k3 = self._compute_qd(q_cur + 0.5*dt*k2, tau)
            k4 = self._compute_qd(q_cur + dt*k3, tau)
            q_new = q_cur + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        
        self._q = q_new # Save new q
    
    def update_sim(self, frame):
        # Step function for simulation itself

        # Torque
        if self._lqr_K is not None:
            q_equ = np.array([0.0, np.pi, 0.0, 0.0])
            tau = ((-self._lqr_K) @ (self._q - q_equ))[0]
            print(tau)
        else: # Default to 0
            tau = 0.0

        # Step simulation
        self._step(tau=tau, method="RK4")
        self._fwd_kin()

        # ----- RESET LOGIC -----
        self._sim_time += self._dt
        if self._reset_interval > 0.0:
            if self._sim_time >= self._reset_interval:
                print("RESETTING SIMULATION")
                self._sim_time = 0.0
                
                # choose new random initial conditions:
                th1 = np.random.uniform(-0.5, 0.5)
                th2 = np.random.uniform(np.pi - 0.5, np.pi + 0.5)
                th1d = np.random.uniform(-0.5, 0.5)
                th2d = np.random.uniform(-0.5, 0.5)

                self._q = np.array([th1, th2, th1d, th2d])
                self._fwd_kin()   # recompute positions
        # ------------------------

        # Update link lines
        r1 = self._x1
        # print(f"r1 = {r1}")
        r2 = self._x2
        # print(f"r2 = {r2}")

        self.link1.set_data([0, r1[0]], [0, r1[1]])
        self.link1.set_3d_properties([0, r1[2]])
        self.link2.set_data([r1[0], r2[0]], [r1[1], r2[1]])
        self.link2.set_3d_properties([r1[2], r2[2]])

         # update internal timer

        return self.link1, self.link2
    
    def simulate(self, q0, reset_time):
        self._q = q0 # Use provided initial condition
        self._reset_interval = reset_time
        plt.show()
    
    """ CONTROLLERS """
    def prep_LQR(self, Q, R):
        g = 9.81 # [m/s^2]

        # Parse into easy vars
        b1 = self._b1
        m1 = self._m1
        l1 = self._len1[0]
        L1 = self._len1[1]
        J1xx = self._J1[0]
        J1yy = self._J1[1]
        J1zz = self._J1[2]

        b2 = self._b2
        m2 = self._m2
        l2 = self._len2[0]
        L2 = self._len2[1]
        J2xx = self._J2[0]
        J2yy = self._J2[1]
        J2zz = self._J2[2]

        # Simplifications
        J1 = J1yy
        J2 = J2yy
        J1hat = J1 + m1*(l1**2)
        J2hat = J2 + m2*(l2**2)
        J0hat = J1 + m1*(l1**2) + m2*(L1**2)
        
        # Now build state space rep
        denom = (J0hat*J2hat - (m2**2)*(L1**2)*(l2**2))

        A31 = 0.0
        A32 = (g*(m2**2)*(l2**2)*L1)/denom
        A33 = (-b1*J2hat)/denom
        A34 = (-b2*m2*l2*L1)/denom
        A41 = 0.0
        A42 = (g*m2*l2*J0hat)/denom
        A43 = (-b1*m2*l2*L1)/denom
        A44 = (-b2*J0hat)/denom

        B31 = J2hat/denom
        B41 = (m2*L1*l2)/denom

        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [A31, A32, A33, A34],
                      [A41, A42, A43, A44]])
        B = np.array([[0.0], [0.0], [B31], [B41]])

        # Solve LQR
        P = solve_continuous_are(A, B, Q, R)
        self._lqr_K = np.linalg.inv(R) @ B.T @ P
        print(self._lqr_K)


if __name__=="__main__":
    # Init
    dt = 0.001 # [s]

    encMass = 0.130 # [kg]. Mass of encoder per Amazon

    m1 = 0.19 + encMass # [kg]
    b1 = 1.0*m1 # [N-m/(rad/s)]
    L1 = 0.140 # [m]
    l1 = 0.8*L1  # !!! ASSUMES how COM is shifted
    J1xx = 0.0
    J1yy = (1/12.0)*m1*(L1**2) # !!! ASSUMES this still good enough
    J1zz = J1yy
    
    m2 = 0.048 # [kg]
    b2 = 0.0198*m2 # [N-m/(rad/s)]. Measured and calc'd with ChatGPT
    L2 = 0.1425 # [m]
    l2 = L2/2
    J2xx = 0.0
    J2yy = (1/12.0)*m2*(L2**2)
    J2zz = J2yy
    simulateFurata = FurataPendulum(dt, m1, l1, L1, J1xx, J1yy, J1zz, m2, l2, L2, J2xx, J2yy, J2zz, b1, b2)
    
    t20 = random.uniform(-0.1, 0.1)
    t2d0 = random.uniform(-0.05, 0.05)
    Q = np.eye(4)
    R = np.array([[10.0]]) # Punish effort, its too high
    print("Prepping LQR")
    simulateFurata.prep_LQR(Q,R)
    simulateFurata.simulate(np.array([0.0, (np.pi + t20), t2d0, 0.0]), reset_time=1.0)
            
