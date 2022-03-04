import numpy as np
from sympy import *
import time
from sympy.physics.quantum.matrixutils import matrix_tensor_product  # Kronecker product


class Dynamics:
    def __init__(self, m, g, I_b, A):
        self.mass = m
        self.I_b = I_b
        self.g = g
        self.A = A

def generalizedJacobian(M, q):
    result = M.col(0).jacobian(q)
    if M.shape[1] == 1:
        return result
    for i in range(1, M.shape[1]):
        result = result.col_insert(result.shape[1], M.col(i).jacobian(q))
    return result

# ref: Nguyen Van Khang, Kronecker product and a new matrix form of Lagrangian equations with multipliers for
# constrained multibody systems, Mechanics Research Communications, 2011.
def Kronecker(A, B):

    return matrix_tensor_product(A, B)

def partialSymDiffOfProduct(A, B, q):
    # symbolic implementation
    return generalizedJacobian(A, q) * (Kronecker(B, eye(q.shape[0]))) + A * generalizedJacobian(B, q)

def partialNumDiffOfProduct(A, dA_dq, B, dB_dq, q):
    # numeric implementation
    return dA_dq.dot((np.kron(B, np.eye(q.shape[0])))) + A.dot(dB_dq)


def breakOutState(q):
    phi = q[3, 0]
    theta = q[4, 0]
    psi = q[5, 0]

    return phi, theta, psi


class OctorotorDynamics(Dynamics):
    def __init__(self, m_b, g, I_b, A):
        super().__init__(m_b, g, I_b, A)
        # physical numerical properties
        self.e3 = np.array([[0.], [0.], [1.]])
        self.JTb = None
        self.JRb = None

        # symbolic variables and parameters
        g_sym, m_b_sym = symbols('g m_b', real=True)
        Ixx, Iyy, Izz = symbols('Ixx Iyy Izz', real=True)
        I_b_sym = Matrix([Ixx, Iyy, Izz])
        x, y, z, phi, theta, psi = symbols('x y z phi theta psi', real=True)
        q = Matrix([x, y, z, phi, theta, psi])

        # symbolic manipulator jacobians--------------------------------------------------------------------------------
        JTb = Matrix([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0]])

        JRb = Matrix([[0, 0, 0, 1, 0, -sin(theta)],
                      [0, 0, 0, 0, cos(theta), sin(phi)*cos(theta)],
                      [0, 0, 0, 0, -sin(phi), cos(phi)*cos(theta)]])

        # lambdify jacobians--------------------------------------------------------------------------------------------
        tic = time.perf_counter()
        self.JTb_func = np.array(JTb).astype(np.float64)
        self.JRb_func = lambdify([q.tolist()], JRb, "numpy")

        # lambdify jacobian transposes----------------------------------------------------------------------------------
        tic = time.perf_counter()
        ATb = JTb.T * m_b_sym
        ARb = JRb.T * Matrix([[I_b_sym[0], 0, 0],
                              [0, I_b_sym[1], 0],
                              [0, 0, I_b_sym[2]]])
        self.ATb_func = lambdify([m_b_sym, q.tolist()], ATb, "numpy")
        self.ARb_func = lambdify([I_b_sym, q.tolist()], ARb, "numpy")

        # lambdify jacobian derivatives---------------------------------------------------------------------------------
        tic = time.perf_counter()

        # body
        dJTb_dq = generalizedJacobian(JTb, q)
        self.dJTb_dq_func = lambdify([q.tolist()], dJTb_dq, "numpy")
        dJRb_dq = generalizedJacobian(JRb, q)
        self.dJRb_dq_func = lambdify([q.tolist()], dJRb_dq, "numpy")

        dATb_dq = generalizedJacobian(ATb, q)
        self.dATb_dq_func = lambdify([m_b_sym, q.tolist()], dATb_dq, "numpy")
        dARb_dq = generalizedJacobian(ARb, q)
        self.dARb_dq_func = lambdify([I_b_sym, q.tolist()], dARb_dq, "numpy")

        # calc symbolic V_dot matrix------------------------------------------------------------------------------------
        e3 = Matrix([[0], [0], [1]])
        Vb = m_b_sym * g_sym * e3.T * q[:3, :]

        # calc dVi_dq matrices------------------------------------------------------------------------------------------
        dVb_dq = generalizedJacobian(Vb, q).T

        # lambdify V matrices-------------------------------------------------------------------------------------------
        self.dVb_dq_func = lambdify([g_sym, m_b_sym, q.tolist()], dVb_dq, "numpy")

    def updateJacobians(self, q):
        self.JTb = self.JTb_func
        self.JRb = self.JRb_func(q)

    def computeMassMatrix(self):
        M = self.mass * (np.transpose(self.JTb).dot(self.JTb)) + np.transpose(self.JRb).dot(
            np.array([[self.I_b[0], 0, 0],
                      [0, self.I_b[1], 0],
                      [0, 0, self.I_b[2]]]).dot(self.JRb))
        return M

    def computeCoriolisMatrix(self, q, qdot):
        # body
        dJTb_dq = self.dJTb_dq_func(q)
        dJRb_dq = self.dJRb_dq_func(q)

        ATb = self.ATb_func(self.mass, q)
        dATb_dq = self.dATb_dq_func(self.mass, q)

        ARb = self.ARb_func(self.I_b, q)
        dARb_dq = self.dARb_dq_func(self.I_b, q)

        dMb_dq = partialNumDiffOfProduct(ATb, dATb_dq, self.JTb, dJTb_dq, q) + \
                 partialNumDiffOfProduct(ARb, dARb_dq, self.JRb, dJRb_dq, q)

        C = dMb_dq.dot(np.kron(np.eye(q.shape[0]), qdot)) - np.transpose(dMb_dq.dot(np.kron(qdot, np.eye(q.shape[0])))) / 2
        C[0, 0] = self.A[0]
        C[1, 1] = self.A[1]
        C[2, 2] = self.A[2]

        return C

    def computeGravityMatrix(self, q):

        dVb_dq = self.dVb_dq_func(-self.g, self.mass, q)
        G = dVb_dq

        return G

    def computeControlMatrix(self, q):
        phi, theta, psi = breakOutState(q)
        # euler rates to angular velocities transformation
        T = np.array([[1., 0., -np.sin(theta)],
                      [0., np.cos(phi), np.cos(theta) * np.sin(phi)],
                      [0., -np.sin(phi), np.cos(phi) * np.cos(theta)]])
        RW0 = np.array([[np.cos(psi) * np.cos(theta), np.sin(phi) * np.sin(theta) * np.cos(psi) - np.sin(psi) * np.cos(phi),
                         np.sin(phi) * np.sin(psi) + np.sin(theta) * np.cos(phi) * np.cos(psi)],
                        [np.sin(psi) * np.cos(theta), np.sin(phi) * np.sin(psi) * np.sin(theta) + np.cos(phi) * np.cos(psi),
                         -np.sin(phi) * np.cos(psi) + np.sin(psi) * np.sin(theta) * np.cos(phi)],
                        [-np.sin(theta), np.sin(phi) * np.cos(theta), np.cos(phi) * np.cos(theta)]])
        B = np.zeros((q.shape[0], 4))
        B[:3, 0] = RW0[:, 2]
        B[3:6, 1:4] = np.linalg.inv(T).dot(RW0)

        return B

class PointMassDynamics(Dynamics):

    def __init__(self, m, l, I):
        super().__init__(m, l, I)

    def computeMassMatrix(self, q):
        M = self.mass * np.eye(2)

        return M

    def computeCoriolisMatrix(self, q, qdot):
        C = np.zeros((2, 2))

        return C

    def computeGravityMatrix(self, q):
        G = np.zeros((2, 2))

        return G

    def computeControlMatrix(self, q):
        B = np.eye(2)

        return B
