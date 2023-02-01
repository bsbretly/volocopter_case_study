%% Case study for a Flight Control Engineer or Intern
%% Prepared by Burak Yueksel. Revised by Nicolas Staub.
%% Definitions
% Aircraft is a star shaped octorotor (8-rotors)
% All rotors are facing upwards (linearly dependent)
% North-East-Down (NED) reference frame notation is used
% Meaning in body frame +X is forward, +Y is right and +Z is downward
%
%                            +X
%                             ^
%                             |
%                             |
%                             * (1) CW
%             *               x               * (2) CCW
%         (8)   x             x             x
%      CCW        x           x           x
%                   x         x         x
%                     x       x       x
%                       x     x     x
%                         x   x   x
%                           x x x
%           *xxxxxxxxxxxxxxxxxOxxxxxxxxxxxxxxxxxx* - - > +Y
%         (7)               x x x                (3) CW
%     CW                  x   x   x
%                       x     x     x
%                     x       x       x
%                   x         x         x
%                 x           x           x
%               x             x             x
%       (6)   *               x               *
%     CCW                (5)  *              (4) CCW
%                       CW
% Mass and moment of inertia is assumed to be centralized at the center of
% the figure above, depicted with O
% 8 identical rotors are placed at the ends of the arms of the aircraft,
% depicted with *.
% Each rotor has an ID next to it, and their direction of rotation
% (Clock Wise - CW or Counter Clock Wise - CCW), when seen from above.
% Notice that each rotor spins in the opposite direction w.r.t. its
% neighboring one.
% These rotors are the actuators of the aircraft, and the motion of the
% center of gravity, also depicted with O is of interest.
% All state measurements are assumed to be coming from the center of
% gravity, and no noises or biases are considered.

% Your tasks are given below, together with some necessary parameters of
% the aircraft

% Do what you can. If you see there are missing or wrong information, this
% is not intended. But then correct it or provide it. Again, do as much as
% you can, everything counts!

% You shall provide your results implemented as .m scripts and/or 
% Simulink files (.slx).
% Only basic matlab + simulink license shall be used (no usage of other 
% tools e.g. control system toolbox or similar).
% Your submission shall be R2019b compatible.
% MATLAB/Simulink(Preferred), but if you do not have access to MATLAB,
% you may also code in Python.
%% Parameters of the VTOL (Star-Shaped Octorotor)
mass                        = 10.0; %kg
moment_of_inertia           = diag([1.0, 1.0, 2.0]); % kg_m_m
gravity                     = 9.81; %m/s
number_of_actuators         = 8;
rotor_thrust_coeff          = 0.05;   % rotor_thrust_N = rotor_thrust_coeff*rotor_rad_per_s^2
rotor_torque_coeff          = 0.005;  % rotor_torque_Nm = rotor_torque_coeff*rotor_rad_per_s^2
rotor_max_rad_per_s         = 30;     % maximum  allowed rotor spinning velocity (in both directions)
rotor_min_rad_per_s         = 0;      % minimum rotor spinning velocity (stop)

%% Task 1 (coding):
% Write the equations of motion of the aircraft (dynamics)
% in a similar form of:
% ddx = f(x,dx)+ g(x)u
% where f(x,dx) is representing the internal dynamics (gravitational
% forces, coriolis forces etc).
% g(x) is representing the
% control input matrix.
% x stands for 3 positions and 3 rotations, hence 6x1 vector.
% dx stands for the time derivative of x, hence 6x1 vector.
% ddx stands for the time derivative of dx, hence 6x1 vector. 
% Notice that the control input matrix g(x) maps the square of the rotor spinning
% velocities rotor_rad_per_s first to the rotor thrust rotor_thrust_N and rotor
% torque rotor_torque_N, and then to the body frame forces and torques.
% u represents the control input, and in this case it will be the square of
% the all rotor velocities. u is in size of 8x1.
% g(x) is in size of 6x8.
% Remember: all rotors are facing upwards, hence they are linearly
% dependent.
% Tip: this implies although system has 8 actuators, it still needs to tilt
% in order to move in the horizontal direction.

% start your code here


% end your code here

%% Task 2 (explanation/coding):
% Show your "control allocation matrix", which maps the square of the rotor
% spinning velocities to body frame forces and torques (moments).

% start your code here


% end your code here

%% Task 3 (explanation/coding):
% Show a possible inverse of your "control allocation matrix"  and 
% argument your choice.

% start your code here


% end your code here

%% Task 4 (coding):
% Create a second order attitude reference dynamics which has:
% input: roll, pitch, yaw angle commands
% output: attitude angles, attitude rates and attitude accelerations

% start your code here


% end your code here

%% Task 5 (coding):
% Write a simple attitude controller (PID) that can control attitude angles
% and rates
%% start your code here


% end your code here

%% Task 6 (coding):
% Show how you choose the gains of your attitude controller

% start your code here


% end your code here

%% Task 7 (coding):
% Simulate the following mission with the dynamics given in Task-1 and
% controllers in Task-5:
% start state: x=[0 0 -10 0 0 0], dx = zeros(6,1)
% desired commands:
% - at time 5 seconds: roll step input 0.2617 radians
% - at time 10 seconds: pitch step input 0.523 radians
% - at time 15 seconds: yaw step input - 0.785 radians
% plot:
% - desired attitude commands (roll, pitch, yaw) vs actual ones (fourth,
% fifth and sixth elements of x)
% x and dx in separated figures
% commands u, and square-root of commands u, in separated figures
% tip: use a while loop, and integrate time using time = old_time + dt_sec.
% make your own simulation loop here. Or something better: go crazy!

% start your code here


% end your code here

%% Task 8 (coding):
% Simulate one rotor failure (rotor ID 1, located at the most front along the +x axis).
% do exactly the same thing for Task 7, but this time fail one of the
% rotors (ID1 or your choice) at time 20 seconds.
% plot the same things as in Task 7.

% start your code here


% end your code here

%% Task 9 (coding):
% Simulate two rotor failure (rotor ID 1 and ID 2, located at the most front along the +x axis).
% do exactly the same thing for Task 8, but this time fail two of the
% rotors (ID1 and ID2) at time 20 seconds.
% plot the same things as in Task 8.

% start your code here


% end your code here

