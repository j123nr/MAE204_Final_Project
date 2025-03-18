function traj = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k, dt)
% This function generates a reference trajectory for the end effector-frame
%
% Inputs:
%   T_se_initial:   The initial configuration of the end-effector
%   T_sc_initial:   The initial configuration of the cube
%   T_sc_final:     The desired final configuration of the cube
%   T_ce_grasp:     The configuration of the end-effector relative to the
%                   cube while grasping
%   T_ce_standoff:  The standoff configuration of the end-effector above
%                   the cube, before and after grasping, relative to the cube
%   k:              The number of trajectory reference configurations per
%                   0.01 seconds
%
%
% Outputs:
%   traj:           A representation of the N configurations of the
%                   end-effector along the reference trajectory
%                   It is a cell array with rows {T_se, gripperstate}
%                   gripperstate is 0 for open, 1 for closed
%   A .csv file with the 8-segment trajectory with each line corresponding
%   to T_se, expressed as 13 variables:
%   r_11, r_12, r_13, r_21, r_22, r_23, r_31, r_32, r_33, p_x, p_y, p_z, gripper_state

% Add the path for the MR repo
%addpath("C:\Users\Andrew Copeland\Documents\Matlab\MAE 204\mr")
%addpath("C:\Users\jnrco\OneDrive - Cal Poly\Desktop\UCSD\mr");

% Duration of each segment (s)
    t1 = 10;    % Move the gripper to the standoff configuration above the cube
    t2 = 0.75;    % Move the gripper to the grasp position
    t3 = 0.75;  % Pause to close the gripper (1)
    t4 = 0.75;    % Move the gripper back to standoff configuration
    t5 = 5;    % Move the gripper to standoff configuration above the final configuration
    t6 = 0.75;    % Move the gripper to the final configuration of the object
    t7 = 0.75;  % Pause to open the gripper (1)
    t8 = 0.75;    % Move the gripper back to the standoff configuration

    % Define number of trajectories
    N1 = t1 * k / dt;
    N2 = t2 * k / dt;
    N3 = t3 * k / dt;
    N4 = t4 * k / dt;
    N5 = t5 * k / dt;
    N6 = t6 * k / dt;
    N7 = t7 * k / dt;
    N8 = t8 * k / dt;
    totalsteps = N1 + N2 + N3 + N4 + N5 + N6 + N7 + N8;

% Find desired end-effector transformation matrices in the space frame
    T_standoff_initial = T_sc_initial * T_ce_standoff;
    T_grasp_initial = T_sc_initial * T_ce_grasp;
    T_standoff_final = T_sc_final * T_ce_standoff;
    T_grasp_final = T_sc_final * T_ce_grasp;

% Initialize the traj cell array
    traj = cell(totalsteps, 2);
    stepcounter = 1;

% Create a function to append each trajectory to the next
    function add_segment(T_start, T_end, duration, gripperstate)
        N = ceil(duration * k / dt); % In case of float values, round to
                                     % nearest whole number
        % Create a trajectory segment using ScrewTrajectory with quintic
        % time scaling
        traj_segment = ScrewTrajectory(T_start, T_end, duration, N, 5);
        % Take the ouput cell array and append it to traj
        for i = 1:N
            traj{stepcounter, 1} = traj_segment{i};  % Append ith trajectory configuration
                                                     % to traj cell array

            traj{stepcounter, 2} = gripperstate; % Append current gripper state to
                                                 % traj cell array
            stepcounter = stepcounter + 1;
        end
        if isempty(traj)
            error('Error: The trajectory cell array is empty.');
        end
    end
% Create trajectory 1: moving to standoff above the cube
    add_segment(T_se_initial, T_standoff_initial, t1, 0);

% Create trajectory 2: moving to grasp the cube
    add_segment(T_standoff_initial, T_grasp_initial, t2, 0);
    
% Pause to allow the gripper to close (frome footnote 2)
    add_segment(T_grasp_initial, T_grasp_initial, t3, 1);

% Create trajectory 4: moving to standoff after grasping the cube
    add_segment(T_grasp_initial, T_standoff_initial, t4, 1);

% Create trajectory 5: moving to standoff above the desired location
    add_segment(T_standoff_initial, T_standoff_final, t5, 1);

% Create trajectory 6: moving to drop the cube
    add_segment(T_standoff_final, T_grasp_final, t6, 1);

% Pause to allow the gripper to open (from footnote 2)
    add_segment(T_grasp_final, T_grasp_final, t7, 0);
    
% Create trajectory 8: move to final standoff after releasing the cube
    add_segment(T_grasp_final, T_standoff_final, t8, 0);

% Add each entry to a csv file for input to CoppeliaSim
%   traj is an N x 2 cell array
%   traj{N, 1} is each 4 x 4 transformation matrix
%   traj(N, 2} is each gripper state

% Confirm final size of traj is totalsteps by 2
if size(traj, 1) ~= totalsteps
    error('Error: trajectory cell array wrong size');
end
 
% csventries = zeros(totalsteps, 13); % Preallocate the csv file
% 
%     for j = 1:totalsteps
%         Tmat = traj{j, 1};
%         gripperstate = traj{j, 2};
%         % Because each entry is a transformation matrix, we don't care
%         % about entries 41, 42, 43, or 44 (0, 0, 0, 1)
%         % Want the upper 3 x 3 (R) and the upper-right 3 x 1 (p)
%         csventries(j, :) = [Tmat(1,1), Tmat(1,2), Tmat(1,3), ...
%                             Tmat(2,1), Tmat(2,2), Tmat(2,3), ...
%                             Tmat(3,1), Tmat(3,2), Tmat(3,3), ...
%                             Tmat(1,4), Tmat(2,4), Tmat(3,4), gripperstate];
%     end

%filename = 'trajectory.csv';
%writematrix(csventries, filename);

end