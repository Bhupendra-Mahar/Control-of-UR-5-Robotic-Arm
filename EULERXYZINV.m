% cos(beta) == 0 will cause divide-zero error when calculating alpha and
% gamma.
function [angles] = EULERXYZINV(rot)
%EULERXYZINV Summary of this function goes here
%   Accept a 3 × 3 rotation matrix and returns a 3 × 1 vector [alpha,beta,gamma]' (in radians)

% check 3*3 matrix
if size(rot) ~= [3,3]
    error("Input should be 3 × 3 rotation matrix!");
end

rot_inv = rot';

beta = atan2(rot_inv(3,1),sqrt(rot_inv(3,2)^2 + rot_inv(3,3)^2));

% check cos(beta) == 0
if abs(cos(beta)) < 1e-9
    warning("cos(beta)==0, EULERXYZINV is numerically ill-defined!");
    alpha = 0;
    beta = pi/2;
    gamma = atan2(rot_inv(1,2),rot_inv(2,2));
else
    alpha = atan2(-rot_inv(3,2)/cos(beta),rot_inv(3,3)/cos(beta));
    gamma = atan2(-rot_inv(2,1)/cos(beta),rot_inv(1,1)/cos(beta));
end

angles = [alpha,beta,gamma]';
end

