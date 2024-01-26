function q_update = find_best_theta(q0,theta)
%FIND_BEST_THETA
% q0 - 6*1 matrix, current theta
% theta - 6x8 matrix, each colum represents one possible solution 
%           of joint angles

[~,i] = min(sum(abs((theta - q0)),1));
q_update = theta(:,i);

% need to be careful near singularity 

end

