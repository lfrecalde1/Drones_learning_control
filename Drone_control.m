%% CLEAN VARIABLES
clc,clear all,close all;

%% DEFINITION OF TIME VARIABLES
ts = 0.1;
t_f = 100;
to = 0;
t = (to:ts:t_f);

%% CONSTANTS VALUES OF THE ROBOT 1
a1 = 0.0; 
b1 = 0.0;
c1 = 0.0;
L1 = [a1, b1, c1];

%% INITIAL CONDITIONS 1
x1 = -0;
y1 = 0.0;
z1 = 3;
yaw1 = 0*(pi/180);

%% DIRECT KINEMATICS 1
x1 = x1 +a1*cos(yaw1) - b1*sin(yaw1);
y1 = y1 +a1*sin(yaw1) + b1*cos(yaw1);
z1 = z1 + c1;

h1 = [x1;...
     y1;...
     z1;...
     yaw1];
 
%% INITIAL GENERALIZE VELOCITIES 1
v1 = [0;...
     0;...
     0;...
     0];
 

 
H = [h1;v1];

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[xd, yd, zd, psid, xdp, ydp, zdp, psidp] = Trayectorias(1,t);

%% GENERALIZED DESIRED SIGNALS
hd1 = [xd; yd; zd; psid];
hdp1 = [xdp;ydp;zdp;psidp];

%% LOAD DYAMIC PARAMETERS DRONE
load("parameters.mat");

%% Control Vector
u_c = zeros(4, length(t));

%% Kinematic controller gains
k1 = 1;
k2 = 1;

%% Dynamic Compensation gains
k3 = 1;
k4 = 1;

% External torque of the system
v_extern = zeros(4, length(t));

% Aux time variable 
t_aux = (t >= 20) & (t < 40);
v_extern(1, t_aux) = 0.5;
v_extern(2, t_aux) = 0.2;
v_extern(3, t_aux) = 0.8;
v_extern(4, t_aux) = 0.0;


t_aux_1 = (t >= 50) & (t < 70);
v_extern(1, t_aux_1) = 0.8;
v_extern(2, t_aux_1) = -0.2;
v_extern(3, t_aux_1) = -0.1;
v_extern(4, t_aux_1) = 0.0;
%% Learning control Parameters
gamma = -0;

% Learning system
% Transfer funtion definitions
P = tf([0 1 (k3-gamma)], [0 0 1]);
A = tf([1], [0 1 k3]);
S = tf([0 0 1], [0 1 0]);

P_operator = P*A;
% Operator contraction
aux = S*P*A;

% Operator Function
aux_d = c2d(aux, ts);
P_operator =c2d(P_operator, ts);
[num1d, den1d] = tfdata(aux_d,'v');

%% Variables Learning control
B = den1d(2:end);
A = num1d(2:end);

% memories values

l1_memories = zeros(length(B), 1);
e1_memories = zeros(length(A), 1);

l2_memories = zeros(length(B), 1);
e2_memories = zeros(length(A), 1);

l3_memories = zeros(length(B), 1);
e3_memories = zeros(length(A), 1);

l4_memories = zeros(length(B), 1);
e4_memories = zeros(length(A), 1);

%% MPC controller definition
%% Prediction Horizon
N = 50; 

%% Boundaries of the control actions
bounded = [3.0; -3.0; 3.0; -3.0; 3.0; -3.0; 3.5; -3.5];
%% Definicion del vectro de control inicial del sistema
vc = zeros(N,4);
H0 = repmat(H,1,N+1)';

%% OPTIMIZATION SOLVER
[f, solver, args] = mpc_drone(bounded, N, L1, ts);
 
for k = 1:1:length(t)-N
    tic; 
    %% GENERAL VECTOR OF ERROR SYSTEM
    he1(:, k) = hd1(1:4,k)-h1(1:4,k);
    

    %% OBTAIN CONTROL VALUES OF THE VECTOR
    u_c(:, k) = Control_Kinematic(hd1(:, k), hdp1(:, k), h1(:, k), k1, k2, L1);
    if k>1
        u_c_p(:, k) = (u_c(:, k)- u_c(:, k-1))/ts;
    else
        u_c_p(:, k) = [0;0;0;0];
    end
    
    %% Learning control section
    if t(k) >= t_aux
        [learning(:, k), l1_memories, l2_memories, l3_memories, l4_memories, e1_memories, e2_memories, e3_memories, e4_memories] = learning_control(u_c(:, k) , v1(:, k), A, B, l1_memories, l2_memories, l3_memories, l4_memories, e1_memories, e2_memories, e3_memories, e4_memories);
    
    %% Dynamic Compensation
    %[u_dynamic(:, k)] = dynamic_compensation(u_c(:, k), u_c_p(:, k), v1(:, k), chi, k3, k4, M_inertia(chi, v1(:, k))*learning(:, k));
    %u_learning(:, k) =  M_inertia(chi, v1(:,k))*learning(:, k) + u_dynamic(:, k);
    
    %% Estimation System
    Estimation(: , k) = M_inertia(chi, v1(:, k))*learning(:, k);
    
        
    else
        learning(:, k) = zeros(4,1); 
    
    %% Dynamic Compensation
    %[u_dynamic(:, k)] = dynamic_compensation(u_c(:, k), u_c_p(:, k), v1(:, k), chi, k3, k4, M_inertia(chi, v1(:, k))*learning(:, k));
    %u_learning(:, k) =  M_inertia(chi, v1(:,k))*learning(:, k) + u_dynamic(:, k);
    
    %% Estimation System
        Estimation(: , k) = M_inertia(chi, v1(:, k))*learning(:, k);
        
    end
    
    
    %% MPC Controller
    [H0, control] = NMPC(h1(:,k), v1(:,k), hd1(:,:), k, H0, vc, args, solver, N, Estimation(:, k));
        
    optimal_control(:, k) = control(1, :)';
    
    %% GET VALUES OF DRONE
    [v_body(:, k)] = wind_body(h1(:, k), v_extern(:, k));
    %% Drone 1
    v1(:,k+1) = system_dynamic(chi, v1(:,k), control(1, :)', v_body(:,k), ts);
    [h1(:,k+1)] = system_drone(h1(:,k), v1(:,k), ts, L1);
    
    %% SAMPLE TIME
    t_sample(k) = toc;
    toc;
    
    %% UPDATE OPTIMIZATION
    vc = [control(2:end,:);control(end,:)];
    H0 = [H0(2:end,:);H0(end,:)];
end

close all; paso=1; 
figure

myVideo = VideoWriter('myVideoFile_2'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
Drone_Parameters(0.02);
G1=Drone_Plot_3D(h1(1,1),h1(2,1),h1(3,1),0,0,h1(4,1));hold on
plot3(h1(1,1),h1(2,1),h1(3,1),'--','Color',[56,171,217]/255,'linewidth',1.3);hold on,grid on
plot3(xd(1),yd(1),zd(1),'Color',[32,185,29]/255,'linewidth',1.3);

view([0 90])
for k = 1:30:length(t)-N
    drawnow
    delete(G1);
    
    
    G1=Drone_Plot_3D(h1(1,k),h1(2,k),h1(3,k),0,0,h1(4,k));hold on
  
    plot3(xd(1:k),yd(1:k),zd(1:k),'Color',[32,185,29]/255,'linewidth',1.3);
    plot3(h1(1,1:k),h1(2,1:k),h1(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.3);
    
    
    legend({'$\eta^{1}$','$\eta^{1}_{ref}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robots}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(he1)),he1(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(he1)),he1(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(he1)),he1(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(he1)),he1(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid('minor')
grid on;
legend({'$\tilde{\eta}^{1}_{x}$','$\tilde{\eta}^{1}_{y}$','$\tilde{\eta}^{1}_{z}$','$\tilde{\eta}^{1}_{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);


% figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 10 4]);
% plot(t(1:length(u_c)),u_c(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on
% plot(t(1:length(u_c)),u_c(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on
% plot(t(1:length(u_c)),u_c(3,:),'Color',[26,115,160]/255,'linewidth',1); hold on
% plot(t(1:length(u_c)),u_c(4,:),'Color',[83,57,217]/255,'linewidth',1); hold on
% grid('minor')
% grid on;
% legend({'$\mu^{1}_{lc}$','$\mu^{1}_{mc}$','$\mu^{1}_{nc}$','$\omega^{1}_{c}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend('boxoff')
% ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
% xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
% 


% figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 10 4]);
% plot(t(1:length(u_dynamic)),u_dynamic(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on
% plot(t(1:length(u_dynamic)),u_dynamic(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on
% plot(t(1:length(u_dynamic)),u_dynamic(3,:),'Color',[26,115,160]/255,'linewidth',1); hold on
% plot(t(1:length(u_dynamic)),u_dynamic(4,:),'Color',[83,57,217]/255,'linewidth',1); hold on
% grid('minor')
% grid on;
% legend({'$\mu^{1}_{lref}$','$\mu^{1}_{mref}$','$\mu^{1}_{nref}$','$\omega^{1}_{ref}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend('boxoff')
% ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
% xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(Estimation)),Estimation(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(Estimation)),Estimation(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(Estimation)),Estimation(3,:),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(Estimation)),Estimation(4,:),'Color',[83,57,217]/255,'linewidth',1); hold on

plot(t(1:length(v_body)),v_body(1,:),'--','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(v_body)),v_body(2,:),'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(v_body)),v_body(3,:),'--','Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(v_body)),v_body(4,:),'--','Color',[83,57,217]/255,'linewidth',1); hold on

grid('minor')
grid on;
legend({'$w_{xe}$','$w_{ye}$','$w_{ze}$','$w_{\psi e}$','$w_{x}$','$w_{y}$','$w_{z}$','$w_{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


% figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 10 4]);
% plot(t(1:length(u_learning)),u_learning(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on
% plot(t(1:length(u_learning)),u_learning(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on
% plot(t(1:length(u_learning)),u_learning(3,:),'Color',[26,115,160]/255,'linewidth',1); hold on
% plot(t(1:length(u_learning)),u_learning(4,:),'Color',[83,57,217]/255,'linewidth',1); hold on
% grid('minor')
% grid on;
% legend({'$\mu^{1}_{ll}$','$\mu^{1}_{ml}$','$\mu^{1}_{nl}$','$\omega^{1}_{l}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend('boxoff')
% ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
% xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(optimal_control)),optimal_control(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(optimal_control)),optimal_control(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(optimal_control)),optimal_control(3,:),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(optimal_control)),optimal_control(4,:),'Color',[83,57,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu^{1}_{lo}$','$\mu^{1}_{mo}$','$\mu^{1}_{no}$','$\omega^{1}_{o}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
