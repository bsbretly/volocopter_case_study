import numpy as np


class TimedTrajectory:

    def __init__(self):
        self.planner_type = "timed"

    def computeDesiredState(self, t):
        raise NotImplementedError("Must override computeVelocityField")


class PolynomialTrajectory(TimedTrajectory):

    def __init__(self):
        # init states
        super().__init__()
        # dim captures 2D or 3D planning
        self.dim = None
        self.p0 = None
        self.pf = None
        self.v0 = None
        self.vf = None
        self.a0 = None
        self.af = None
        self.p = None
        self.v = None
        self.a = None

        # init polynomial coefficients
        self.alpha = None
        self.beta = None
        self.gamma = None

    def initPlanner(self, p0, pf, Tf):
        # dim captures 2D or 3D planning
        self.dim = len(p0)
        self.p0 = np.array(p0).reshape(-1, 1)
        self.pf = np.array(pf).reshape(-1, 1)
        self.v0 = np.zeros((self.dim, 1))
        self.vf = np.zeros((self.dim, 1))
        self.a0 = np.zeros((self.dim, 1))
        self.af = np.zeros((self.dim, 1))

        self.p = np.zeros((self.dim, 1))
        self.v = np.zeros((self.dim, 1))
        self.a = np.zeros((self.dim, 1))

        # init polynomial coefficients
        self.alpha = np.zeros((self.dim, 1))
        self.beta = np.zeros((self.dim, 1))
        self.gamma = np.zeros((self.dim, 1))

        # generate trajectory
        self.generateTrajectory(Tf)

    def generateTrajectory(self, Tf):
        T2 = Tf * Tf
        T3 = T2 * Tf
        T4 = T3 * Tf
        T5 = T4 * Tf

        #  generate coefficients for each axis
        for i in range(self.dim):
            # define starting position:
            delta_p = self.pf[i] - self.p0[i] - self.v0[i] * Tf - (1. / 2.) * self.a0[i] * T2
            delta_v = self.vf[i] - self.v0[i] - self.a0[i] * Tf
            delta_a = self.af[i] - self.a0[i]

            deltaVector = np.array([delta_p, delta_v, delta_a])

            # solve linear system with x = inv(A)*b where x is the coefficient vector
            A = np.array([[T3, T4, T5],
                          [3. * T2, 4. * T3, 5. * T4],
                          [6. * Tf, 12. * T2, 20. * T3]])
            x = np.linalg.inv(A).dot(deltaVector)

            self.alpha[i] = x[0]
            self.beta[i] = x[1]
            self.gamma[i] = x[2]

    def getPosition(self, t):
        for i in range(self.dim):
            self.p[i] = self.p0[i] + self.v0[i] * t + (1. / 2.) * self.a0[i] * t * t + self.alpha[i] * t * t * t + \
                        self.beta[i] * t * t * t * t + self.gamma[i] * t * t * t * t * t

        return self.p

    def getVelocity(self, t):
        for i in range(self.dim):
            self.v[i] = self.v0[i] + self.a0[i] * t + 3. * self.alpha[i] * t * t + 4. * self.beta[i] * t * t * t + \
                        5. * self.gamma[i] * t * t * t * t

        return self.v

    def getAcceleration(self, t):
        for i in range(self.dim):
            self.a[i] = self.a0[i] + 6. * self.alpha[i] * t + 12. * self.beta[i] * t * t + \
                        20. * self.gamma[i] * t * t * t

        return self.a

    def computeDesiredState(self, t):

        return self.getPosition(t), self.getVelocity(t), self.getAcceleration(t)


class WaypointGenerator:

    def __init__(self, position):
        self.planner_type = "waypoint"
        self.x_d = position[0]
        self.y_d = position[1]
        self.z_d = position[2]

    def computeDesiredState(self):
        p_d = np.array([[self.x_d], [self.y_d], [self.z_d]])
        v_d = np.zeros((3, 1))
        a_d = np.zeros((3, 1))

        return p_d, v_d, a_d
