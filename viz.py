from planner import PolynomialTrajectory
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.use('TkAgg')


class PlotState:

    def __init__(self, *args, three_dee=True):
        self.q = args[0]
        self.time = args[1]
        self.RAD_TO_DEGREE = 180./np.pi
        self.p_ds = None
        self.three_dee = three_dee

    def plotPosition(self, p0, sim_phases, dt):
        if self.three_dee:
            # 3D position plot
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            ax.set_title('Position')
            ax.plot(self.q[:, 0], self.q[:, 1], self.q[:, 2], color='r', linewidth=5, label=r'$p$')
            ax.set_xlabel(r'$x\ [m]$')
            ax.set_ylabel(r'$y\ [m]$')
            ax.set_zlabel(r'$z\ [m]$')
            plt.legend()

        idx = 0
        self.p_ds = np.zeros((self.q.shape[0], 3))
        for i in range(len(sim_phases)):
            Tf = sim_phases[i][0]
            pf = sim_phases[i][1]
            t_a = np.arange(0, Tf, dt)
            phase_length = len(t_a)
            traj = PolynomialTrajectory()
            traj.initPlanner(p0, pf, Tf)
            traj.generateTrajectory(Tf)

            for j in range(len(t_a)):
                p_d = traj.getPosition(t_a[j])
                self.p_ds[idx + j, :] = p_d[0, 0], p_d[1, 0], p_d[2, 0]
            idx += phase_length
            p0 = pf

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, sharey=True)
        ax1.set_title('Position tracking')
        ax1.plot(self.time, self.q[:, 0], label=r'$x$')
        ax1.plot(self.time, self.p_ds[:, 0], '--', label=r'$x_d$')
        ax1.legend()
        ax1.grid()
        ax2.plot(self.time, self.q[:, 1], label=r'$y$')
        ax2.plot(self.time, self.p_ds[:, 1], '--', label=r'$y_d$')
        ax2.legend()
        ax2.grid()
        ax2.set_ylabel(r'$[m]$')
        ax3.plot(self.time, self.q[:, 2], label=r'$z$')
        ax3.plot(self.time, self.p_ds[:, 2], '--', label=r'$z_d$')
        ax3.legend()
        ax3.grid()
        ax3.set_xlabel(r'$time\ [s]$')
        plt.xlim(self.time[0], self.time[-1])

    def plotAttitude(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
        ax1.set_title('Attitude')
        ax1.plot(self.time, self.q[:, 3]*self.RAD_TO_DEGREE)
        ax1.set_ylabel(r'$\phi\ [deg]$')
        ax1.grid()
        ax2.plot(self.time, self.q[:, 4]*self.RAD_TO_DEGREE)
        ax2.set_ylabel(r'$\theta\ [deg]$')
        ax2.grid()
        ax3.plot(self.time, self.q[:, 5]*self.RAD_TO_DEGREE)
        ax3.set_ylabel(r'$\psi\ [deg]$')
        ax3.grid()
        ax3.set_xlabel(r'$time\ [s]$')
        plt.xlim(self.time[0], self.time[-1])
