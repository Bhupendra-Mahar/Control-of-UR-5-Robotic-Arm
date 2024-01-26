clear
clc

control = 'RR';
% control = 'JT';
ur5 = ur5_interface();
K=0.5;

theta = [0 -pi/4 pi/2 pi/4 pi/2 0;]  ;
ur5.move_joints(theta.',10);
pause(10)


gstart =ur5FwdKin(theta);
start_pos = gstart*[0 0 0 1].'
SP = tf_frame('base_link', 'SP', gstart);

final_position=    [0.1300 -0.9732 1.9646 0.5794 1.5708 0.1300;];

gend =ur5FwdKin(final_position)
end_pos = gend*[0 0 0 1].'
FP = tf_frame('base_link', 'FP', gend);

[g_point1,g_point2] = get_intermediate_points(gstart,gend)

if (control=='RR')
        IP1 = tf_frame('base_link', 'IP1', g_point1);
        ur5RRcontrol(g_point1, K, ur5);
        pause(1);
        IP2 = tf_frame('base_link', 'IP2', g_point2);
        ur5RRcontrol(g_point2, K, ur5);
        pause(1);
        ur5RRcontrol(gend, K, ur5);
else
    ur5JTcontrol(gend, K, ur5);
    pause(1);
end


current_position=ur5.get_current_joints()
gsim =ur5FwdKin(current_position.')
[err1, err2]=SE3_error(gsim,gend)