function [rot] = EULERXYZ(angles)
%EULERXYZ Summary of this function goes here
%   Accept one input, a 3-vector of angles (in radians), 
% and returns the corresponding 3 Ã— 3 rotation matrix

% check 3*1 vector
if size(angles) ~= [3,1]
    error("Angles should be 3 × 1 vector!");
end

rot = ROTX(angles(1)) * ROTY(angles(2)) * ROTZ(angles(3));

end

