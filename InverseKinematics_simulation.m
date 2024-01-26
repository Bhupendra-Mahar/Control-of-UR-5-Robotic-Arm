clear all
%%
ur5 = ur5_interface();

%% teach the start and end location
disp('Teach end point');
pause(2)
g_end = [0,-1,0,0.4;
         -1 ,0,0,0.45;
         0,0,-1,0.22;
         0,0,0,1];

q_end = find_best_theta(ur5.home,ur5InvKin(g_end));

disp('Teach start point');
pause(2);
g_start = [0,-1,0,0.25;
         -1 ,0,0,0.6;
         0,0,-1,0.22;
         0,0,0,1];

q_start = find_best_theta(ur5.home,ur5InvKin(g_start));

pause(2);

%% Calculate intermediate points
[g_point1,g_point2] = get_intermediate_points(g_start,g_end);


time_interval = 5;

q1 = ur5InvKin(g_point1)
q1_best = find_best_theta(q_start,q1)


%% Go to start point
ur5.move_joints(q_start,10);
pause(11)

q_start_current = ur5.get_current_joints()
g_start_current = ur5FwdKin(q_start_current')

Frame_start = tf_frame('base_link','SP',g_start);

%% Go to first intermediate point
ur5.move_joints(q1_best, time_interval)
pause(6)

q1_current = ur5.get_current_joints()

Frame_point1 = tf_frame('base_link','IP1',ur5FwdKin(q1_current'));

%% Go to second intermediate point
q2 = ur5InvKin(g_point2)
q2_best = find_best_theta(q1_current,q2)

ur5.move_joints(q2_best, time_interval)
pause(6)

q2_current = ur5.get_current_joints();

Frame_point2 = tf_frame('base_link','IP2',ur5FwdKin(q2_current'));

%% Go to end point
ur5.move_joints(q_end, time_interval)
pause(6)

% record the final actual position
q_end_current = ur5.get_current_joints()
g_end_current = ur5FwdKin(q_end_current')

Frame_final = tf_frame('base_link','FP',g_end_current);

%% compute errors
[g_start_err_SO3,g_start_err_R3] = SE3_error(g_start_current,g_start)
[g_end_err_SO3,g_end_err_R3] = SE3_error(g_end_current,g_end)