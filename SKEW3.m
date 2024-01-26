function S = SKEW3(x)
    
    % Extract components
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    
    % Create the skew-symmetric matrix
    S = [0, -x3, x2;
         x3, 0, -x1;
         -x2, x1, 0];
end
