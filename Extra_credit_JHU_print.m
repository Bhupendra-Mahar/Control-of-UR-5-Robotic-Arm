clc;
clear all;
ur5 = ur5_interface();

% Define the coordinates for each letter "J", "H", and "U"
J_points = [[0 -0.05 0];
            [0 0.025 0];
            [0.1 0 0];
            [0 -0.025 0];
            [-0.05 0 0]];
Lift=[0 0 0.05];

H_points =  [[-0.05 -0 0];
            [0 0.075 0];
            [0 0 -0.05];
            [0.1 0 0];
            [-0.05 0 0];
            [0 0.05 0];
            [0.05 0 0];
            [-0.1 0 0]];

U_points = [[0 0.05 0];
            [0 0 -0.05];
            [0.1 0 0];
            [0 0.05 0];
            [-0.1 0 0];];

ur5.swtich_to_pendant_control()
pause(2)


% Get final and start positions respectively
w = waitforbuttonpress;
c1 = ur5.get_current_joints()
pause(2);
w = waitforbuttonpress;
c2=ur5.get_current_joints()
pause(2);

gc2 =ur5FwdKin(c2.');
c2_pos = gc2*[0 0 0 1].'
gc1 =ur5FwdKin(c1.');
c1_pos = gc1*[0 0 0 1].'

% Calcuate the slope and the rotation matrix
m = atan2((c1_pos(2)-c2_pos(2)),(c1_pos(1)-c2_pos(1)))
R = ROTZ(-m)
disp('Teach the starting position')

w = waitforbuttonpress;
start_position=ur5.get_current_joints()
pause(2);

ur5.swtich_to_ros_control();
pause(2);


% get the start position
gstart = ur5FwdKin(start_position.')
q_start= find_best_theta(start_position,ur5InvKin(gstart))
R_position = tf_frame('base_link', 'SF', gstart);

ur5.move_joints(q_start, 5);
pause(5)
disp('Moved to qstart')



% Combine the points for "J", "H", and "U" to form "JHU"
points_JHU = 0.5*[
    J_points;
    [Lift];
    H_points;
    [Lift];
    U_points; 
]

% Use inverse kinematics control to Print the JHU

q_curr = q_start;
for i = 1:size(points_JHU, 1)
    g_update = [eye(3), ROTZ(m)*points_JHU(i, :).';
        0 0 0 1];
    gdesired = g_update*gstart;
    R_position = tf_frame('base_link', int2str(i), gdesired);
    q_curr = find_best_theta(q_curr,ur5InvKin(gdesired))
    ur5.move_joints(q_curr, 8);
    pause(9);
    gstart = gdesired;
    pause(1);
end

