function J = ur5BodyJacobian(q_in)

    L0=89.2*1e-3;
    L1=0.425;
    L2=0.392;
    L3=0.1093;
    L4=.09475;
    L5=.0825;
    w = [0 0 1;
        0 1 0;
        0 1 0;
        0 1 0;
        0 0 1;
        0 1 0;].';

    q = [0 0 0; 
        0 0 L0;
        0 0 L0+L1;
        0 0 L0+L1+L2;
        0 L3 L0+L1+L2;
        0 L3 L0+L1+L2+L4;].';

    gst_0=[ROTX(-pi/2),[0,L3+L5,L0+L1+L2+L4]';
        0,0,0,1];

    joints=6;

    % Calculate eta
    eta = sym('eta', [6,joints]);
    theta = q_in;
    for i =1:6
            eta(1:3,i) = -cross(w(:,i), q(:,i));
            eta(4:6,i) = w(:,i);
    end

    exp= sym('exp', [4,4,joints]);
    gst_theta = eye(4);

    for i=1:joints
        w_hat = [0 -w(3,i), w(2,i); w(3,i) 0 -w(1,i); -w(2,i) w(1,i) 0];
        if (i==2) || (i==4)
            R = eye(3) + sin(theta(i)+pi/2)*w_hat + (1 - cos(theta(i)+pi/2))*(w_hat^2);
        else
            R = eye(3) + sin(theta(i))*w_hat + (1 - cos(theta(i)))*(w_hat^2);
        end
        
        p = (eye(3,3) - R)* cross(eta(4:6,i),eta(1:3,i));
        exp(:,:,i) = [R p; zeros(1,3) 1];
        gst_theta = simplify(gst_theta * exp(:,:,i));
    end
    double(gst_theta*gst_0);

    J = eta;
    for i=1:6
    poe = gst_0;
    for k=joints:-1:i
        poe = simplify(exp(:,:,k)*poe);
    end
    p = poe(1:3,4);
    p_hat = [0 -p(3), p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    adjoint = [poe(1:3, 1:3) p_hat*poe(1:3, 1:3); zeros(3) poe(1:3, 1:3)];
    J(:,i) = simplify(inv(adjoint)*eta(:,i));
    end

J=double(J);



end
    