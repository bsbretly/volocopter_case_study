import numpy as np
from scipy.integrate import odeint
from dynamics import OctorotorDynamics
from multipledispatch import dispatch


class Robot:

    def __init__(self, *args):
        self.q = self.qdot = self.vdot = None
        self.M = self.B = self.C = self.G = None
        self.controller = self.planner = None
        self.mass = args[0]
        self.g = args[1]
        self.time_array = None

    def defineController(self, controller):
        self.controller = controller

    def definePlanner(self, planner):
        self.planner = planner

    def initState(self, *args):
        q_0 = np.array([args[0]]).reshape(-1, 1)
        qdot_0 = np.array([args[1]]).reshape(-1, 1)
        self.vdot = np.zeros(q_0.shape)

        self.q = q_0
        self.qdot = qdot_0
        self.updateDynamics(self.q, self.qdot)

    def updateDynamics(self, q, q_dot):
        raise NotImplementedError("Must override updateDynamics")

    def updateTrajectory(self, t):
        # input: t
        # output: q_d, qdot_d, qddot_d
        return self.planner.computeDesiredState(t)

    def updateControl(self, *args):
        # control to desired waypoint
        if self.controller.control_type == "pd_trajectory":
            # control to waypoint: q, qdot, pos_d, vel_d, acc_d
            # control to timed trajectory: q, qdot, pos_d, vel_d, acc_d
            q = args[0]
            qdot = args[1]
            pos_d = args[2]
            vel_d = args[3]
            acc_d = args[4]

            return self.controller.computeCommand(q, qdot, pos_d, vel_d, acc_d)

        else:
            print("ERROR: please define a valid controller")
            return

    def updateState(self, sim_phases, dt):
        # inpute: simulation phases, simulation time step
        # output: final state for each simulation phase [q, q_dot]
        sim_duration = 0
        for i in range(len(sim_phases)):
            sim_duration += sim_phases[i][0]

        overall_length = len(np.arange(0, sim_duration, dt))
        final_state = np.zeros([overall_length, self.q.shape[0] + self.qdot.shape[0]])
        index = 0

        for i in range(len(sim_phases)):
            phase_time = sim_phases[i][0]
            t_a = np.arange(0, phase_time, dt)
            # plan trajectory for given sim phase
            self.planner.initPlanner(self.q[:3, 0], sim_phases[i][1], phase_time)
            state = np.concatenate((self.q[:, 0], self.qdot[:, 0]))
            output_state = odeint(self.integrator, state, t_a)
            output_length = len(output_state)

            # state update
            [q, qdot] = np.split(output_state[output_length - 1], 2)
            self.q = np.array(q).reshape(-1, 1)
            self.qdot = np.array(qdot).reshape(-1, 1)

            final_state[index:(index + output_length)] = output_state
            index = index + output_length

        return final_state

    def integrator(self, state, t):
        # input: state [q, qdot]
        # output: state rates [qdot, qddot]

        q = np.array(state[:self.q.shape[0]]).reshape(-1, 1)
        qdot = np.array(state[self.q.shape[0]:]).reshape(-1, 1)
        self.updateDynamics(q, qdot)

        # solve system of first order diff eq's with substitution: v = qdot
        v = np.array(state[self.q.shape[0]:]).reshape(-1, 1)

        if self.planner.planner_type == "timed":
            pos_d, vel_d, acc_d = self.updateTrajectory(t)  # returns q_d, qdot_d, qddot_d
            u = self.updateControl(q, v, pos_d, vel_d, acc_d)

        else:
            print("ERROR: please define a valid planner")
            return

        self.vdot = np.linalg.inv(self.M).dot(self.B.dot(u) + self.G - self.C.dot(np.array(v.reshape(-1, 1))))

        return np.concatenate((v[:, 0], self.vdot[:, 0]))


class Octorotor(Robot):

    def __init__(self, *args):
        super().__init__(args[0][0], args[0][1])
        I = args[0][2]
        A = args[0][3]
        self.octorotor_dynamics = OctorotorDynamics(self.mass, self.g, I, A)

    def updateDynamics(self, q, qdot):
        # q = [x, y, z, phi, theta, psi]^T

        self.octorotor_dynamics.updateJacobians(q)
        self.M = self.octorotor_dynamics.computeMassMatrix()
        self.C = self.octorotor_dynamics.computeCoriolisMatrix(q, qdot)
        self.G = self.octorotor_dynamics.computeGravityMatrix(q)
        self.B = self.octorotor_dynamics.computeControlMatrix(q)
