function ifsafe = safety_check(q)
%SAFETY_CHECK Summary of this function goes here
%   Detailed explanation goes here

% judge the theta is in a reasonable range

% theta1 -180 ~ +180
% theta2 -160 ~ -20
% theta3 -140 ~ 140
% theta4 -180 ~ 0
% theta5 -135 ~ +135
% theta6 -180 ~ +180

ifsafe = true;

if ((q(1) >= pi) || (q(1) <= -pi))
disp('theta1 out of limits!');
ifsafe = false;
end
if ((q(2) >= -20/180*pi) || (q(2) <= -160/180*pi))
    disp('theta2 out of limits!');
ifsafe = false;
end
if ((q(3) >= 140/180*pi) || (q(3) <= -140/180*pi))
    disp('theta3 out of limits!');
ifsafe = false;
end
if ((q(4) >= 0) || (q(4) <= -pi))
    disp('theta4 out of limits!');
ifsafe = false;
end
if ((q(5) >= 135/180*pi) || (q(5) <= -135/180*pi))
    disp('theta5 out of limits!');
ifsafe = false;
end
if ((q(6) >= pi) || (q(6) <= -pi))
    disp('theta6 out of limits!');
ifsafe = false;
end

%% check all joints are above table
% params for length (m)
L0 = 89.2*1e-3;
L1 = 425*1e-3;
L2 = 392*1e-3;
L3 = 109.3*1e-3;
L4 = 94.75*1e-3;
L5 = 82.5*1e-3;

% ur5.home config twists
w1 = [0,0,1]';
q1 = [0,0,0]';
Xi1 = [-SKEW3(w1)*q1;w1];

w2 = [0,1,0]';
q2 = [0,0,L0]';
Xi2 = [-SKEW3(w2)*q2;w2];

w3 = [0,1,0]';
q3 = [0,0,L0+L1]';
Xi3 = [-SKEW3(w3)*q3;w3];

w4 = [0,1,0]';
q4 = [0,0,L0+L1+L2]';
Xi4 = [-SKEW3(w4)*q4;w4];

w5 = [0,0,1]';
q5 = [0,L3,L0+L1+L2]';
Xi5 = [-SKEW3(w5)*q5;w5];

w6 = [0,1,0]';
q6 = [0,L3,L0+L1+L2+L4]';
Xi6 = [-SKEW3(w6)*q6;w6];

% zero config pose
gst0 = [ROTX(-pi/2),[0,L3+L5,L0+L1+L2+L4]';
        0,0,0,1];

% PoE (ur5.home is not zero config)
H1 = expm(SKEWXi(Xi1)*q(1));
H2 = expm(SKEWXi(Xi2)*(q(2)+pi/2));
H3 = expm(SKEWXi(Xi3)*q(3));
H4 = expm(SKEWXi(Xi4)*(q(4)+pi/2));
H5 = expm(SKEWXi(Xi5)*q(5));
H6 = expm(SKEWXi(Xi6)*q(6));

% home configuration
g0_j3 = [eye(3),[0,0,L0+L1]';
        0,0,0,1];
g_j3 = H1 * H2 * g0_j3;
if (g_j3(3,4) <= 0)
    disp('joint3 is under ground!');
ifsafe = false;
end

g0_j4 = [eye(3),[0,L3,L0+L1+L2]';
        0,0,0,1];
g_j4 = H1 * H2 * H3 * g0_j4;
if (g_j4(3,4) <= 0)
    disp('joint4 is under ground!');
ifsafe = false;
end

g0_j5 = [eye(3),[0,L3,L0+L1+L2+L4]';
        0,0,0,1];
g_j5 = H1 * H2 * H3 * H4 * g0_j5;
if (g_j5(3,4) <= 0)
    disp('joint5 is under ground!');
ifsafe = false;
end

gst = H1*H2*H3*H4*H5*H6*gst0;
if (gst(3,4) <= 0)
    disp('tool0 is under ground!');
ifsafe = false;
end

end

