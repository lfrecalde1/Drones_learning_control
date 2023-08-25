function [M] = M_inertia(x, v)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
u = v(1);
w = v(4);

% INERCIAL MATRIX
M11=x(1);
M12=0;
M13=0;
M14=x(2);
M21=0;
M22=x(3);
M23=0;
M24=0;
M31=0;
M32=0;
M33=x(4);
M34=0;
M41=x(5);
M42=0;
M43=0;
M44=x(6);



M=[M11,M12,M13,M14;...
    M21,M22,M23,M24;...
    M31,M32,M33,M34;...
    M41,M42,M43,M44];
end

