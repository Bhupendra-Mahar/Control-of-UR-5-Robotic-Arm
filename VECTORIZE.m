function vec = VECTORIZE(skewMat)
    
    % check if the input matrix is a skew matrix
    if size(skewMat,1) ~= 3 || size(skewMat,2) ~= 3|| ~isequal(skewMat,-skewMat')
        error('Input must be a 3x3 skew-symmetric matrix.');
    end
     % Extract the vector from the skew-symmetric matrix
    vec = [-skewMat(2,3); skewMat(1,3); -skewMat(1,2)];
end
