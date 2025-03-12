function next_state = NextState(current_state, current_vel, dt, v_max)
% This function uses velocity kinematics to predict how the youBot will
% move using the Euler method
%
% Inputs:
%   Current state: 12 variables
%                  Chassis: [phi, x, y]
%                  Arm:     [theta1, theta2, theta3, theta4, theta5]
%                  Wheels:  [phi1, phi2, phi3, phi4]
%
%   Joint and wheel velocities: 9 variables
%                  Arm: [thetad1, thetad2, thetad3, thetad 4, thetad5]
%                  Wheels: [u1, u2, u3, u4]
%
%   Timestep: dt
%   Maximum joint and wheel velocities: vmax
%
% Output:
%   Next state: 12 variables
%                  Chassis: [phi, x, y]
%                  Arm:     [theta1, theta2, theta3, theta4, theta5]
%                  Wheels:  [phi1, phi2, phi3, phi4]

% The current state will be fed in as a 12-vector
    chassis_state = current_state(1:3);
    arm_state = current_state(4:8);
    wheel_state = current_state(9:12);

% Need to limit joint and wheel velocities to vmax: min(currentvel, vmax)
% Also need to preserve negative values: max(min(currentvel, vmax), -vmax)
    current_vel = max(min(current_vel, v_max), -v_max);

% The current velocity will be fed in as a 9-vector
    arm_vel = current_vel(1:5);
    wheel_vel = current_vel(6:9);

% Use Euler integration to predict arm and wheel velocities
    new_arm_state = arm_state + arm_vel * dt;
    new_wheel_state = wheel_state + wheel_vel * dt;

% From equation 13.33 in MR
    r = 0.0475; % Wheel radius [m]
    l = 0.47/2; % Forward-backward wheel spacing [m]
    w = 0.3/2;  % Side-to-side wheel spacing [m]

    F = (r/4) * [-1/(l+w)   1/(l+w)   1/(l+w)   -1/(l+w)
                    1          1        1          1
                   -1          1       -1          1];

% From the textbook, there are 2 cases based on w_bz = V_b(1)
% We need V_b = F * delta_theta, and delta_theta = theta_dot * dt
    V_b = F * wheel_vel' * dt;
    w_bz = V_b(1);
    v_bx = V_b(2);
    v_by = V_b(3);

% Practically, we have to provide a threshold at which w_bz = 0
    if abs(w_bz) < 1e-6
        delta_qb = [  0
                    v_bx
                    v_by];
    else
        delta_qb = [                       w_bz
                    (v_bx*sin(w_bz) + v_by*(cos(w_bz) - 1)) / w_bz
                    (v_by*sin(w_bz) + v_bx*(1 - cos(w_bz))) / w_bz];
    end

% Transform delta_qb in {b} to delta_q in {s}
    phi_k = chassis_state(1);
    R_c = [1       0            0
           0   cos(phi_k)  -sin(phi_k)
           0   sin(phi_k)   cos(phi_k)];
    delta_q = R_c * delta_qb;

% Updated odometry estimate of chassis configuration
    new_chassis_state = chassis_state + delta_q';

% Create the output
    next_state = [new_chassis_state new_arm_state new_wheel_state];

end