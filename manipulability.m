function measure_out = manipulability(J_q,measure)

% compute the sigma_max


eigenvalues_JTJ = eig(transpose(J_q)*J_q);
singular_values = sqrt(eigenvalues_JTJ);
sigma_min =min(singular_values);
sigma_max =max(singular_values);
% measure 1:minimum single value of J
if measure=="sigmamin"
measure_out = min(singular_values);
end

% measure 2:Inverse of the condition number of 
if measure=="invcond"
    measure_out = sigma_min/sigma_max;
end

% measure 3:determinant of J
if measure=="detjac"
    measure_out = det(J_q);
end

end


