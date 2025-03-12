function [V, u, dot_theta, Xerr_integral] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_integral, theta)
% FeedbackControl computes the task-space feedforward plus PI feedback control law.
%
% Inputs:
%   X             : 4x4 homogeneous transformation matrix representing the current
%                   actual end-effector configuration (T_se).
%   Xd            : 4x4 homogeneous transformation matrix representing the current
%                   reference end-effector configuration (T_se,d).
%   Xd_next       : 4x4 homogeneous transformation matrix representing the next
%                   reference end-effector configuration (T_se,d,next).
%   Kp            : 6x6 proportional gain matrix.
%   Ki            : 6x6 integral gain matrix.
%   dt            : Scalar, the time step between reference configurations.
%   Xerr_integral : 6x1 vector, the running integral of the error twist.
%   theta         : Vector of current arm joint angles (used to compute the Jacobian).
%
% Outputs:
%   V             : 6x1 commanded end-effector twist (expressed in frame {e}).
%   u             : Vector of commanded wheel speeds.
%   dot_theta     : Vector of commanded arm joint speeds.
%   Xerr_integral : Updated integral of the error twist.
%
% The control law is computed as:
%   V = Ad_{(X⁻¹Xd)} * Vd + Kp*X_err + Ki*Xerr_integral
%
% where:
%   X_err = se3ToVec( MatrixLog6(inv(X)*Xd) )
%   Vd    = (1/dt)*se3ToVec( MatrixLog6(inv(Xd)*Xd_next) )
%
% and [u; dot_theta] = pinv(Je) * V, with Je being the mobile manipulator
% Jacobian computed based on the current arm configuration.
%
% Make sure that your Modern Robotics library is in the MATLAB path.

    %% 1. Compute the error twist X_err
    T_err = inv(X) * Xd;                   % Transformation error
    se3mat_err = MatrixLog6(T_err);         % Logarithm of the transformation error
    X_err = se3ToVec(se3mat_err);             % Convert to 6-vector twist

    %% 2. Update the integral of the error
    Xerr_integral = Xerr_integral + X_err * dt;

    %% 3. Compute the feedforward twist Vd
    T_d_next = inv(Xd) * Xd_next;            % Relative transformation from Xd to Xd_next
    se3mat_d = MatrixLog6(T_d_next);         % Matrix logarithm to get se(3) representation
    Vd = se3ToVec(se3mat_d) / dt;            % Feedforward twist

    %% 4. Map Vd to the current end-effector frame using the adjoint transformation
    T_XinvXd = inv(X) * Xd;
    Ad_T = Adjoint(T_XinvXd);

    %% 5. Compute the commanded end-effector twist V
    V = Ad_T * Vd + Kp * X_err + Ki * Xerr_integral;

    %% 6. Compute the mobile manipulator Jacobian Je
    % This function should compute the Jacobian based on your robot's kinematics.
    Je = GetJacobian(theta);  % Ensure this function is implemented appropriately

    %% 7. Compute the joint speeds using the pseudoinverse of Je
    joint_speeds = pinv(Je) * V;

    %% 8. Partition the joint speeds into wheel speeds (u) and arm joint speeds (dot_theta)
    n_wheels = 4;  % Modify this value based on your robot's configuration
    u = joint_speeds(1:n_wheels);
    dot_theta = joint_speeds(n_wheels+1:end);

end
