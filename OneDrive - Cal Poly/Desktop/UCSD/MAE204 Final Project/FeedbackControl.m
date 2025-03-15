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

