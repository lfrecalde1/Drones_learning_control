function [v_inertial] = wind_inertial(h, v_extern)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
psi = h(4);

%% Rotational Matrix
RotZ = [cos(psi) -sin(psi) 0, 0;...
        sin(psi) cos(psi) 0, 0;...
        0 0 1, 0;...
        0, 0, 0, 1];

%% velocity body frame

v_inertial = (RotZ)*v_extern;
end

