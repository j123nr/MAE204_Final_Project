function [V, u, thetad, X_err, X_err_int] = FeedbackControl(config, X, X_d, X_d_next, kp, ki, dt, X_err_int)
% This function calculates the task-space feedforward and feedback control
% for the end-effector
%
% Inputs:
%   The current robot configuration : config [phi, x, y, theta1, theta2,
%                                             theta3, theta4, theta5]
%   Current actual e-e configuration: X (aka T_se)
%                  
%   Current reference e-e configuration: X_d (aka T_se,d)
%                 
%   Reference e-e configuration in next timestep: X_d_next (aka T_se,d,next)
%                  
%   PI gains: kp, ki (matrices)
%
%   Timestep: dt
%
% Outputs:
%   Commanded e-e twist: V
%
%   Commanded wheel speeds: u
%
%   commanded arm joint speeds: thetad
%
% The control law is computed as:
%   V = Ad_{(X^{-1} * Xd)} * Vd + Kp*X_err + Ki*Xerr_integral
%
% where:
%   X_err = se3ToVec(MatrixLog6(inv(X) * X_d))
%   Vd    = (1/dt)*se3ToVec(MatrixLog6(inv(Xd)*Xd_next))
%
% and [u; dot_theta] = pinv(Je) * V, with Je being the mobile manipulator
% Jacobian computed based on the current arm configuration

%% NEED TO CHANGE THIS WHEN RUNNING ON A DIFFERENT MACHINE
%addpath("C:\Users\jnrco\OneDrive - Cal Poly\Desktop\UCSD\mr");
%addpath("C:\Users\Andrew Copeland\Documents\MATLAB\MAE 204\mr");

% NOTE: Tolerance has been chosen for pinv!!
tol = 1e-2;

%% Separate arm configurations
arm_config = config(4:8);

%% Compute error twist X_err
X_err_mat = MatrixLog6(X \ X_d);
X_err = se3ToVec(X_err_mat);

%% Compute feed-forward twist V_d in current e-e frame
Vd_mat = MatrixLog6(X_d \ X_d_next);
Vd = se3ToVec(Vd_mat)/dt; % apparently need to divide by dt since Vd is the body twist of the desired trajectory at time t (or over dt for our discretized function)
% see eq 6.7 in textbook for kinda explanation - since the actual equation
% involves T_sd dot, and we aren't taking the derivative but instead doing
% an Euler step (diff(Tsd)/dt), this is how we convert this into a
% numerical method here

%% Compute error twist integral X_err_int by using Euler integration as approximation
X_err_int = X_err_int + X_err * dt;


%% Get the adjoint
Ad_X = Adjoint(X \ X_d);

%% Implement PI control law
V = Ad_X * Vd + kp * X_err + ki * X_err_int;

%% Now, we need to convert the e-e twist into joint and wheel speeds for our robot
% Blist is found in the MR capstone project description
Blist = [  0       0        0        0       0
           0      -1       -1       -1       0
           1       0        0        0       1
           0    -0.5076  -0.3526  -0.2176    0
         0.033     0        0        0       0
           0       0        0        0       0];

J_arm = JacobianBody(Blist, arm_config);

% From MR textbook for Kuka youBot
r = 0.0475; % Wheel radius [m]
l = 0.47/2; % Forward-backward wheel spacing [m]
w = 0.3/2; % Side-side wheel spacing [m]

F = (r/4) * [-1/(l+w)   1/(l+w)   1/(l+w)   -1/(l+w)
                1          1        1          1
               -1          1       -1          1];

T_b0 = [1   0   0   0.1662
        0   1   0     0
        0   0   1   0.0026
        0   0   0     1];

M_0e = [1   0   0   0.033
        0   1   0     0
        0   0   1   0.6546
        0   0   0     1];

T_0e = FKinBody(M_0e, Blist, arm_config');

F6 = [zeros(1, size(F, 2))
      zeros(1, size(F, 2))
               F
      zeros(1, size(F, 2))];

Ad_T = Adjoint(T_0e \ inv(T_b0));

J_base = Ad_T * F6;

Je = [J_base J_arm];

sped = pinv(Je,tol) * V;

u = sped(1:4);
thetad = sped(5:end);