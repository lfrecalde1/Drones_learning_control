function [f,solver,args] = mpc_drone(bounded, N, L, ts, chi)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;

load('parameters.mat');
%% Definicion de las restricciones en las acciones de control
ul_max = bounded(1); 
ul_min = bounded(2);

um_max = bounded(3);
um_min = bounded(4);

un_max = bounded(5);
un_min = bounded(6);

w_max = bounded(7); 
w_min = bounded(8);

%% Generacion de las variables simbolicas de los estados del sistema
x = SX.sym('x'); 
y = SX.sym('y');
z = SX.sym('z');
th = SX.sym('th');
ul = SX.sym('ul');
um = SX.sym('um');
un = SX.sym('un');
w = SX.sym('w');

%% Definicion de cuantos estados en el sistema
states = [x;y;z;th;ul;um;un;w];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
ul_ref = SX.sym('ul_ref');
um_ref = SX.sym('um_ref');
un_ref = SX.sym('un_ref');
w_ref = SX.sym('w_ref');

ul_extern = SX.sym('ul_extern');
um_extern = SX.sym('um_extern');
un_extern = SX.sym('un_extern');
w_extern = SX.sym('w_extern');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [ul_ref;um_ref;un_ref;w_ref]; 
n_control = length(controls);

extern = [ul_extern;um_extern;un_extern;w_extern]; 
n_extern = length(extern);
%% Definicion de los las constantes dl sistema
a = L(1);
b = L(2);

%% Defincion del sistema pero usando espacios de estados todo el sistema de ecuaciones
J = [cos(th), -sin(th), 0, -(a*sin(th)+b*cos(th));...
     sin(th), cos(th), 0,  (a*cos(th)-b*sin(th));...
     0, 0, 1, 0;...
     0, 0, 0, 1]; 

% INERCIAL MATRIX
M11=chi(1);
M12=0;
M13=0;
M14=chi(2);
M21=0;
M22=chi(3);
M23=0;
M24=0;
M31=0;
M32=0;
M33=chi(4);
M34=0;
M41=chi(5);
M42=0;
M43=0;
M44=chi(6);



M=[M11,M12,M13,M14;...
    M21,M22,M23,M24;...
    M31,M32,M33,M34;...
    M41,M42,M43,M44];

%% CENTRIOLIS MATRIX
C11=chi(7);
C12=chi(8)+chi(9)*w;
C13=chi(10);
C14=chi(11);
C21=chi(12)+chi(13)*w;
C22=chi(14);
C23=chi(15);
C24=chi(16)+chi(17)*w;
C31=chi(18);
C32=chi(19);
C33=chi(20);
C34=chi(21);
C41=chi(22);
C42=chi(23)+chi(24)*w;
C43=chi(25);
C44=chi(26);

C=[C11,C12,C13,C14;...
    C21,C22,C23,C24;...
    C31,C32,C33,C34;...
    C41,C42,C43,C44];

%% GRAVITATIONAL MATRIX
G11=0;
G21=0;
G31=chi(27);
G41=0;

G=[G11;G21;G31;G41]; 


A = [zeros(4,4),J;...
     zeros(4,4),-inv(M)*C];
 
B = [zeros(4,4);...
    inv(M)];

C_aux = [zeros(4,1);...
    -inv(M)*G];

F = [zeros(4,1);...
     -inv(M)*extern];

rhs=(A*states+B*controls + C_aux + F);

%% Definicion de kas funciones del sistema
f = Function('f',{states,controls,extern},{rhs}); 
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states) + 4);
%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
g = [];  % restricciones de estados del problema  de optimizacion

%%EMPY VECTOR ERRORS
he = [];

%% EMPY VECTOR CONTROL VALUES
u = [];

%% INITIAL CONDITION ERROR
st  = X(:,1); % initial state

g = [g;X(:,1)-P(1:8)]; % initial condition constraints


%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    
    st = X(:,k);  con = U(:,k);
    %X(4,k) = full(wrapToPi(X(4,k)));

    %% Funcion costo a minimizar 
    he = [he;X(1:4,k)-P(8*k+1:8*k+4)];
    u = [u;con];
%     obj = obj+(X(1:4,k)-P(8*k+1:8*k+4))'*Q*(X(1:4,k)-P(8*k+1:8*k+4)) + con'*R*con;
    fee = P((n_states)+N*(n_states)+1:(n_states)+N*(n_states)+4);
    %% Actualizacion del sistema usando Euler runge kutta
    st_next = X(:,k+1);
    k1 = f(st, con, fee');   % new 
    k2 = f(st + ts/2*k1, con, fee'); % new
    k3 = f(st + ts/2*k2, con, fee'); % new
    k4 = f(st + ts*k3, con, fee'); % new
    st_next_RK4 = st +ts/6*(k1 +2*k2 +2*k3 +k4); % new 
    
    %% Restricciones del sistema se =basan en el modelo del sistema
    g = [g;st_next-st_next_RK4]; 
end

% Cost final 
Q = 1*eye(size(he,1));
R = 0.009*eye(size(u,1));


% FINAL COST
obj = he'*Q*he+u'*R*u;

%% Control values constrains
% for k =1:N-1
%     g = [g; U(1,k)-U(1,k+1) - 0.05];
%     g = [g; U(1,k+1)-U(1,k) - 0.05]; 
%     
%     g = [g; U(2,k)-U(2,k+1) - 0.05];
%     g = [g; U(2,k+1)-U(2,k) - 0.05];
%     
%     g = [g; U(3,k)-U(3,k+1) - 0.05];
%     g = [g; U(3,k+1)-U(3,k) - 0.05];  
%     
%     g = [g; U(4,k)-U(4,k+1) - 0.02];
%     g = [g; U(4,k+1)-U(4,k) - 0.02];  
% end
% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,8*(N+1),1);reshape(U,4*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:8*(N+1)) = 0;  %-1e-20  %Equality constraints
args.ubg(1:8*(N+1)) = 0;  %1e-20   %Equality constraints
% 
% args.lbg(8*(N+1)+1:8*(N+1)+ 8*(N-1)) = -inf;  
% args.ubg(8*(N+1)+1:8*(N+1)+ 8*(N-1)) = 0;  

args.lbx(1:8:8*(N+1),1) = -inf; %state x lower bound
args.ubx(1:8:8*(N+1),1) = inf;  %state x upper bound

args.lbx(2:8:8*(N+1),1) = -inf; %state y lower bound
args.ubx(2:8:8*(N+1),1) = inf;  %state y upper bound

args.lbx(3:8:8*(N+1),1) = -inf; %state z lower bound
args.ubx(3:8:8*(N+1),1) = inf;  %state z upper bound

args.lbx(4:8:8*(N+1),1) = -inf; %state theta lower bound
args.ubx(4:8:8*(N+1),1) = inf;  %state theta upper bound

args.lbx(5:8:8*(N+1),1) = -inf; %state x lower bound
args.ubx(5:8:8*(N+1),1) = inf;  %state x upper bound

args.lbx(6:8:8*(N+1),1) = -inf; %state y lower bound
args.ubx(6:8:8*(N+1),1) = inf;  %state y upper bound

args.lbx(7:8:8*(N+1),1) = -inf; %state z lower bound
args.ubx(7:8:8*(N+1),1) = inf;  %state z upper bound

args.lbx(8:8:8*(N+1),1) = -inf; %state theta lower bound
args.ubx(8:8:8*(N+1),1) = inf;  %state theta upper bound


%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(8*(N+1)+1:4:8*(N+1)+4*N,1) = ul_min;  %
args.ubx(8*(N+1)+1:4:8*(N+1)+4*N,1) = ul_max;  %

args.lbx(8*(N+1)+2:4:8*(N+1)+4*N,1) = um_min;  %
args.ubx(8*(N+1)+2:4:8*(N+1)+4*N,1) = um_max;  % 

args.lbx(8*(N+1)+3:4:8*(N+1)+4*N,1) = un_min;  %
args.ubx(8*(N+1)+3:4:8*(N+1)+4*N,1) = un_max;  %

args.lbx(8*(N+1)+4:4:8*(N+1)+4*N,1) = w_min;  %
args.ubx(8*(N+1)+4:4:8*(N+1)+4*N,1) = w_max;  %

end