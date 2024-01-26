function R = ROTX(theta)
    % Check if the input is a scalar
    if ~isscalar(theta)
        error('Input must be a scalar.');
    end
    
    % Compute the rotation matrix
    R = [1      0           0;
         0  cos(theta)  -sin(theta);
         0  sin(theta)   cos(theta)];
end
