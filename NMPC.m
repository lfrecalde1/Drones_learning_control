function [H0, control] = NMPC(h, v, hd, k, H0, vc, args, solver ,N, fee)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
H = [h;v];
args.p(1:8) = H;

for i = 1:N
    args.p(8*i+1:8*i+8)=[hd(:,k+i);0;0;0;0];
end
%P((n_states)+N*(n_states)+1:(n_states)+N*(n_states)+4)
args.p((8)+N*(8)+1:(8)+N*(8)+4) = fee;
args.x0 = [reshape(H0',8*(N+1),1);reshape(vc',size(vc,2)*N,1)]; % initial value of the optimization variables

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

control = reshape(full(sol.x(8*(N+1)+1:end))',4,N)';
H0 = reshape(full(sol.x(1:8*(N+1)))',8,N+1)';
end


