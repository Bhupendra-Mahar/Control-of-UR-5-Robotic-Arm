function finalerr = ur5RRcontrol(gdesired, K, ur5)
    dist_threshold = 0.005;  % Define distance threshold
    angle_threshold = (0.15*pi)/180;% Define angle threshold
    Tstep = 0.5;    % Define T_step
    R_position = tf_frame('base_link', 'R', gdesired);
    maxiter = 200;

    q = ur5.get_current_joints(); 

    %  Foor loop will run till the convergence or max iteration
    for iteration = 1:maxiter
        gst = ur5FwdKin(q.');
        err = inv(gdesired) * gst;
        xi = getXi(err);
        
        J = ur5BodyJacobian(q.') ;

        q = double(q - K * Tstep * pinv(J)*xi);
        
        J = ur5BodyJacobian(q.') ;
        if abs(det(J))<0.01
            fprintf('Singularity position \n');
            finalerr = -1;
            return;
        end
        
        if norm(xi(1:3)) < dist_threshold && norm(xi(4:6)) < angle_threshold
            finalerr = norm(xi(1:3))*10;  
            fprintf('Convergence achieved. Final error: %.2f cm\n', finalerr);
            return;
        end
        
       
        ur5.move_joints(q,0.5);

        pause(0.5);
    end

    finalerr = -1;
end


