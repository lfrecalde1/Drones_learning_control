function [u] = Control_Kinematic(hd, hdp, h, k1, k2, L)
%% Get Jacobian Matrix

J = drone_jacobian(h, L);

%% Get Error
he = hd - h;

%% Control Matrices
K1 = k1*eye(4, 4);
K2 = k2*eye(4, 4);

v =  (hdp + K2*tanh(inv(K2)*K1*he));

u = inv(J)*v;
end

