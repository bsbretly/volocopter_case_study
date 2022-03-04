import numpy as np


class PDTrajectoryControl:

    # PD position controller for a octorotor to follow a timed trajectory
    def __init__(self, PDPosition_gains, octo_parameters):
        # octo_parameters = (m_b, g, (Ixx, Iyy, Izz), (A_x, A_y, A_z))
        self.Kpos_p = PDPosition_gains[0]
        self.Kpos_d = PDPosition_gains[1]

        self.psi_d = None
        self.e3 = np.array([[0.], [0.], [1.]])

        self.m = octo_parameters[0]
        self.g = octo_parameters[1]
        self.I_b = octo_parameters[2]

        self.control_type = "pd_trajectory"
        # low-level attitude control
        self.attitudeControl = None
        self.octo_parameters = octo_parameters

    def initAttitudeController(self, PDAttitude_gains, yaw_d):
        # low-level attitude control
        self.attitudeControl = AttitudeControl(PDAttitude_gains, yaw_d, self.octo_parameters)

    def computeThrustVector(self, q, qdot, pos_d, vel_d, acc_d):
        # input: state, desired state
        # output: thrust vector
        p_d = pos_d
        v_d = vel_d
        a_d = acc_d
        return self.e3 * self.m * self.g + a_d * self.m - self.Kpos_d * (qdot[:3] - v_d) - self.Kpos_p * (q[:3] - p_d)

    def computeCommand(self, q, qdot, pos_d, vel_d, acc_d):
        # input: state, desired state
        # output: thrust and torque commands: [T_d, tau_phi_d, tau_theta_d, tau_psi_d]^T

        Alpha = self.computeThrustVector(q, qdot, pos_d, vel_d, acc_d)
        attitude_command = self.attitudeControl.computeCommand(q, qdot, Alpha)

        return attitude_command

class AttitudeControl:

    def __init__(self, PDAttitude_gains, yaw_d, octo_parameters):
        # PD attitude gains: Kphi_p, Ktheta_p, Kpsi_p, Kphi_d, Ktheta_d, Kpsi_d, I_b
        # octo_parameters = (m, g, I, A)

        self.Kphi_p = PDAttitude_gains[0]
        self.Ktheta_p = PDAttitude_gains[1]
        self.Kpsi_p = PDAttitude_gains[2]
        self.Kphi_d = PDAttitude_gains[3]
        self.Ktheta_d = PDAttitude_gains[4]
        self.Kpsi_d = PDAttitude_gains[5]
        self.I_b = octo_parameters[2]
        self.yaw_d = yaw_d

        # reference gains
        self.m_r = 0.001
        self.b_r = 0.2
        self.k_r = 1.0
        # reference state
        self.eta_r = np.zeros((3, 1))
        self.eta_dot_r = np.zeros((3, 1))
        self.eta_ddot_r = np.zeros((3, 1))
        self.dt = 1. / 30.  # integration time step for reference dynamics

    def computeReferenceAttitude(self, q):
        # eta = [phi, theta, psi]^T
        # input: eta
        # input: eta_r, eta_dot_r, eta_ddot_r
        eta = q[3:6]
        # numerically integrate out reference attitude dynamics based on tuning parameters m_r, b_r, k_r
        self.eta_ddot_r = (eta - self.b_r*self.eta_dot_r - self.k_r*self.eta_r) / self.m_r
        self.eta_dot_r += self.eta_ddot_r*self.dt
        self.eta_r += self.eta_dot_r*self.dt

    def decompoaseThrustVector(self, Alpha, psi):
        # decompose thrust vector into thrust scalar and desired attitude: [T, roll_d, pitch_d, yaw_d]^T
        # input: thrust vector (Alpha) and desired yaw (psi)
        # output: thrust scalar T, roll_d, pitch_d, yaw_d
        # psi = q[5]  # yaw
        R_psi = np.array([[np.cos(psi), -np.sin(psi), 0.],
                          [np.sin(psi), np.cos(psi), 0.],
                          [0., 0., 1.]])
        T_d = np.linalg.norm(Alpha)
        Alpha_hat = (1 / T_d) * (np.transpose(R_psi).dot(Alpha))

        phi_d = -np.arcsin(Alpha_hat[1])[0]
        theta_d = np.arctan2(Alpha_hat[0], Alpha_hat[2])[0]

        return T_d, phi_d, theta_d

    def computeCommand(self, q, qdot, command):

        # desired attitude rates hard-coded to 0 for now
        phidot_d = 0.
        thetadot_d = 0.
        psidot_d = 0.
        psi_d = self.yaw_d

        #  q_octo = [x, y, z, phi, theta, psi]^T
        #  q_octo_bar = [x, y, z, phi, theta, psi, q_fly]^T
        phi = q[3, 0]
        theta = q[4, 0]
        psi = q[5, 0]
        phidot = qdot[3, 0]
        thetadot = qdot[4, 0]
        psidot = qdot[5, 0]

        Alpha = command

        T_d, phi_d, theta_d = self.decompoaseThrustVector(Alpha, psi)

        # PD attitude control law
        tau_phi_d = self.I_b[0] * (self.Kphi_d * (phidot_d - phidot) + self.Kphi_p * (phi_d - phi))
        tau_theta_d = self.I_b[1] * (self.Ktheta_d * (thetadot_d - thetadot) + self.Ktheta_p * (theta_d - theta))
        tau_psi_d = self.I_b[2] * (self.Kpsi_d * (psidot_d - psidot) + self.Kpsi_p * (psi_d - psi))

        u = np.array([T_d, tau_phi_d, tau_theta_d, tau_psi_d]).reshape(-1, 1)

        return u
