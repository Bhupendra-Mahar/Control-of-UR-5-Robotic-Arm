function [err_SO3,err_R3] = SE3_error(g,gd)
%SE3_ERROR Summary of this function goes here
%   Detailed explanation goes here

err_SO3 = sqrt(trace((g(1:3,1:3)-gd(1:3,1:3))*(g(1:3,1:3)-gd(1:3,1:3))'));
err_R3 = norm(g(1:3,4)-gd(1:3,4));

end

