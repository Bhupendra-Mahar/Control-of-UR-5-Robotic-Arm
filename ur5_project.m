clear
clc

%% choose control
% control = 'JT';
control = 'RR';
% control='IK';

%% start ur5 interface
ur5 = ur5_interface();

% set gain for JT and RR
K=0.5;

%% teach start and end position
ur5.swtich_to_pendant_control()
pause(2)
waitforbuttonpress;
final_position = ur5.get_current_joints()
pause(2);
waitforbuttonpress;
current_position=ur5.get_current_joints()
pause(2);

%%  safety check for the start and end points
if (~safety_check(current_position))
    disp('start not safe');
    w = waitforbuttonpress;
end
if (~safety_check(final_position))
    disp('end not safe');
    w = waitforbuttonpress;
end

gstart =ur5FwdKin(current_position.');
start_pos = gstart*[0 0 0 1].';
R_position = tf_frame('base_link', 'SP', gstart);

gend =ur5FwdKin(final_position.');
end_pos = gend*[0 0 0 1].';

R_position = tf_frame('base_link', 'FP', gend);

ur5.swtich_to_ros_control();
pause(2);

%% calculate intermediate points
[g_point1,g_point2] = get_intermediate_points(gstart,gend)

%% 3 different controls
if (control=='RR')
        ur5RRcontrol(g_point1, K, ur5);
        pause(1);
        ur5RRcontrol(g_point2, K, ur5);
        pause(1);
        ur5RRcontrol(gend, K, ur5);

elseif (control=='JT')
        ur5JTcontrol(gend, K, ur5);
        pause(1);
else % IK

time_interval = 5;

% find best q1 solution
q1 = ur5InvKin(g_point1)
q1_best = find_best_theta(current_position,q1)

% move to q1_best
ur5.move_joints(q1_best, time_interval)
pause(6)
q1_current = ur5.get_current_joints();

% find best q2 solution
q2 = ur5InvKin(g_point2)
q2_best = find_best_theta(q1_current,q2)

% move to q2_best
ur5.move_joints(q2_best, time_interval)
pause(6)
q2_current = ur5.get_current_joints();

% move the end point
ur5.move_joints(final_position, time_interval)
pause(6)
end


