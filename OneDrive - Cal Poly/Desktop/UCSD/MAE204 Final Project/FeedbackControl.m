function [Ve, u, thetad] = FeedbackControl(X, X_d, X_d_next, kp, ki, dt)
% This function calculates the task-space feedforward and feedback control
% for the end-effector
%
% Inputs:
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
%   Commanded e-e twist: Ve
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
%
% NOTE: Tolerance has been chosen for pinv!!
tol = 1e-4;

%% Compute error twist X_err
X_err_mat = MatrixLog6(inv(X) * X_d);
X_err = se3ToVec(X_err_mat);

%% Compute feed-forward twist V_d in current e-e frame
Vd_mat = MatrixLog6(inv(Xd) * X_d_next);
Vd = se3ToVec(Vd_mat)/dt; % apparently need to divide by dt since Vd is the body twist of the desired trajectory at time t (or over dt for our discretized function)
% see eq 6.7 in textbook for kinda explanation - since the actual equation
% involves T_sd dot, and we aren't taking the derivative but instead doing
% an Euler step (diff(Tsd)/dt), this is how we convert this into a
% numerical method here

%% Compute error twist integral X_err_int by using Euler integration as approximation
X_err_int = X_err_int + X_err * dt;

%% Get the gah damn adjoint
Ad_T = Adjoint(inv(X) * X_d);

%% Implement PI control law
V = Ad_T * Vd + kp * X_err + ki * X_err_int;

%% Now, we need to convert the e-e twist into joint and wheel speeds for our robot
Blist = [0,   0,    0,    0,    0;
         0,   1,    1,    1,    1;
         1,   0,    0,    0,    0;
         0,  -0.1, -0.2, -0.3, -0.4;
         0.1, 0,    0,    0,    0;
         0,   0.05, 0.05, 0.05, 0.05]; % This is just example - NEED TO UPDATE!!

% THIS IS FORTNITE BALLS - IDK HOW TO DO IT
% -SOMEHOW WE NEED THETA_ARM, CHASSIS_STATE
% MAYBE THIS IS WHAT T_bb' is for in lecture?
%   T_bb' = MatrixExp(Vb6)

J_arm = JacobianBody(Blist, theta_arm);
Jb_chassis = JacobianBody(Blist, chassis_state)
J_chassis = Ad_Tec * Jb_chassis; % NEED TO ADD Ad_Tec!
Je = [J_chassis, J_arm];

sped = pinv(Je,tol) * V;

u = sped(1:4);
thetad = sped(5:end);

fprintf("I literally just computed velocity and shit")




