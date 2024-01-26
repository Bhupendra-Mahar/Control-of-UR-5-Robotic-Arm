function xi = getXi(g)
    % Check if the input is a 4x4 matrix
    if size(g, 1) ~= 4 || size(g, 2) ~= 4
        error('Input must be a 4x4 matrix.');
    end

    % Extract rotation matrix R and translation vector p from g
    R = g(1:3, 1:3);
    p = g(1:3, 4);

    % Compute the rotation angle theta
    theta = acos((trace(R) - 1) / 2);

    % Check for pure translation (no rotation)
    if abs(theta) == 0
        omega = [0; 0; 0];
        v = p / norm(p,2);
        theta = norm(p,2);
    else
        % Compute omega for the case of rotation
        omega = 1 / (2*sin(theta)) * VECTORIZE(R-transpose(R));

        % Compute the linear velocity part of the twist
        omega_hat = SKEW3(omega);
        % solve G*v = P for v
        G = eye(3)*theta + (1- cos(theta))*omega_hat+(theta-sin(theta))*(omega_hat^2);
        G_inv = inv(G);
        v = G_inv * p;
    end

    % Construct the twist vector
    xi = [v; omega]*theta;
end


