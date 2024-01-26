function xi_hat = SKEWXi(xi)
% Form the se(3) matrix (twist)
v =xi(1:3);
w = xi(4:6);
xi_hat = [SKEW3(w), v; 0 0 0 0];
end
