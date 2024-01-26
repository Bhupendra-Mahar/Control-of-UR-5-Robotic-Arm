function F_gst= ur5FwdKin(q_in)
    syms theta1 theta2 theta3 theta4 theta5 theta6
    L0=89.2*1e-3;
    L1=0.425;
    L2=0.392;
    L3=0.1093;
    L4=.09475;
    L5=.0825;
    W = [0 0 1;
        0 1 0;
        0 1 0;
        0 1 0;
        0 0 1;
        0 1 0;];

    Q = [0 0 0; 
        0 0 L0;
        0 0 L0+L1;
        0 0 L0+L1+L2;
        0 L3 L0+L1+L2;
        0 L3 L0+L1+L2+L4;];

    gst_0=[ROTX(-pi/2),[0,L3+L5,L0+L1+L2+L4]';
        0,0,0,1];



    theta=[theta1,theta2,theta3,theta4,theta5,theta6];
    n = size(theta,2);
    gst = eye(4);   

    for i = 1:n
        w=W(i,:).';
        q=Q(i,:).';
        w_hat=SKEW3(w);
        v=-cross(w,q);
        if (i==2) || (i==4)
            R = eye(3) + sin(theta(i)+pi/2)*w_hat + (1 - cos(theta(i)+pi/2))*(w_hat^2);
        else
            R = eye(3) + sin(theta(i))*w_hat + (1 - cos(theta(i)))*(w_hat^2);
        end
        p= (eye(3) - R) * cross(w,v);

        exp_g = [R,p;
                0,0,0,1;];
        gst = simplify(gst * exp_g);
    end

    final_gst=gst*gst_0;
    F_gst1=simplify(final_gst); 

    F_gst=double(subs(F_gst1,[theta1,theta2,theta3,theta4,theta5,theta6],q_in));

end