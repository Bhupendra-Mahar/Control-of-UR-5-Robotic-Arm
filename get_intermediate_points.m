function [g_point1,g_point2] = get_intermediate_points(g_start,g_end)
%   compute intermediate points with geometry

z = (g_start(3,4)+g_end(3,4))/2;

x_start = g_start(1,4);
y_start = g_start(2,4);

x_end = g_end(1,4);
y_end = g_end(2,4);

% circle center point
x_center = (x_start+x_end)/2;
y_center = (y_start+y_end)/2;

% xy-plane distance between start and end points
d = sqrt((x_end-x_start)^2 + (y_end - y_start)^2);

% angle of 5cm/(10cm+5cm)
theta = atan2(1,3);

% angle of the line slope from start to end
alpha = atan2(y_end-y_start,x_end-x_start);

% find that point on the circle
x_p1 = x_start - d*cos(theta)*cos(alpha-theta);
x_p2 = x_start + d*cos(theta)*cos(alpha-theta);
y_p1 = y_start - d*cos(theta)*sin(alpha-theta);
y_p2 = y_start + d*cos(theta)*sin(alpha-theta);

x = [x_p1,x_p1,x_p2,x_p2];
y = [y_p1,y_p2,y_p1,y_p2];

err2 = (x-x_center).^2 + (y-y_center).^2 - (x_start - x_center)^2 - (y_start-y_center)^2;

[~,i] = min(abs(err2));
x_p = x(i);
y_p = y(i);

% get intermediate point 1
x_point1 = x_p + 1/3*(x_start - x_p);
y_point1 = y_p + 1/3*(y_start - y_p);

% get intermediate point 2
x_point2 = x_end + 1/3*(x_start - x_p);
y_point2 = y_end + 1/3*(y_start - y_p);

g_point1 = [g_start(:,1:3),[x_point1;y_point1;z;1]];
g_point2 = [g_start(:,1:3),[x_point2;y_point2;z;1]];

end

% test get_intermediate_points()
% g_start = ur5FwdKin(rand(1,6)*pi);
% g_end = g_start;
% g_end(1:2,4) = g_end(1:2,4) + rand(2,1);
% [g_point1,g_point2] = get_intermediate_points(g_start,g_end);
% plot([g_start(1,4),g_point1(1,4),g_point2(1,4),g_end(1,4)],[g_start(2,4),g_point1(2,4),g_point2(2,4),g_end(2,4)]);xlim([-1,1]);ylim([-1,1]);