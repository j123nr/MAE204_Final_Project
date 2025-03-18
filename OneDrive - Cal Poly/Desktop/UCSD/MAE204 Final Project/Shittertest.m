clear;clc;
    % current_chassis = [0 0.1662 0];
    % current_arm = [0 0 0 -pi/2 0];
    % current_wheels = [0 0 0 0];

    current_chassis = [0 0 0];
    current_arm = [0 0 0.2 -1.6 0];
    current_wheels = [0 0 0 0];

    currentConfig = [current_chassis current_arm current_wheels]; 
robotConfig = currentConfig;

addpath("C:\Users\Andrew Copeland\Documents\MATLAB\MAE 204\mr");
T_se = getTseFromConfig(robotConfig)

function T_se = getTseFromConfig(robotConfig)
        % Extract angles
        phi   = robotConfig(1); % chassis rotation
        x     = robotConfig(2); % chassis x
        y     = robotConfig(3); % chassis y
        arm_config  = robotConfig(4:8);

        % Build T_sb:
        T_sb = [ cos(phi), -sin(phi), 0, x;
                 sin(phi),  cos(phi), 0, y;
                 0,         0,        1, 0.0963;
                 0,         0,        0, 1];
M_0e = [1 0 0 0.033;
        0 1 0     0;
        0 0 1 0.6546;
        0 0 0     1];

% Blist (from your project spec)
Blist = [ 0     0     0          0        0;
          0    -1    -1         -1        0;
          1     0     0          0        1;
          0  -0.5076 -0.3526 -0.2176      0;
          0.033 0     0          0        0;
          0     0     0          0        0];

T_b0 = [ 1  0  0  0.1662;
         0  1  0  0;
         0  0  1  0.0026;
         0  0  0  1];
        T_0e = FKinBody(M_0e, Blist, arm_config');

        % T_{s,e} = T_{s,b} * T_{b,0} * T_{0,e}
        T_se = T_sb * T_b0 * T_0e;
    end