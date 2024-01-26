function R = ROTZ(theta)
    % Check if the input is a scalar
    if ~isscalar(theta)
        error('Input must be a scalar.');
    end
    
    % Compute the rotation matrix
    R = [cos(theta)  -sin(theta)        0;
         sin(theta)  cos(theta)  0;
         0    0   1];
end
