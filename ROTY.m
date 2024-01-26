function R = ROTY(theta)
    % Check if the input is a scalar
    if ~isscalar(theta)
        error('Input must be a scalar.');
    end
    
    % Compute the rotation matrix
    R = [cos(theta)     0    sin(theta);
         0          1               0 ;
         -sin(theta)  0   cos(theta)];
end