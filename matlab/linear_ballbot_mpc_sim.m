%% Prepare workspace
clear all; clc; close all;

%% Load Model
modelName = 'ballbot2D';
load(strcat('syms_model_',modelName,'.mat'));

% Model Parameters values
[params, unpacked_params] = get_ballbot2D_model_params();
load(unpacked_params);

% Ballbot 2D Model
% Define symbolic variables
syms phi theta dphi dtheta ddphi ddtheta tau real % State variables

% State vector
q = [theta; phi];
dq = [dtheta; dphi];
X = [q;dq];
u = tau;

% Linearize System
Alin = jacobian(dX,X);
Blin = jacobian(dX,u);

% subs parameters
theta = 0; phi = 0;
dtheta = 0; dphi = 0;

Anum = double(subs(Alin));
Bnum = double(subs(Blin));

%%
Nu = size(Bnum,2); % Number of control inputs (appended gravity term)
Nx = size(Bnum,1); % Number of states
N = 10; % Time horizons to consider
Nq = (N+1)*Nx + N*Nu; % Toal number of decision variables
dt = 0.1; % Time step
m = 5; % Mass of drone
g=9.81;
k_cmd=1;
tau = 0.01;

% Weights on state deviation and control input
Qx = diag([100 100 1 100]);
Qn = 10*Qx;
Ru = diag([1]);

% Bounds on states and controls
xmin = [-inf;-inf;-inf;-inf];
xmax = [inf; inf; inf;inf];
umin = [-20];
umax = [20];

stateBounds = [xmin xmax];
controlBounds = [umin umax];

% Reference trajectory
theta_ref = sin(0.5*(0:N));
phi_ref = zeros(1,N+1);

dtheta_ref = zeros(1,N+1);
dphi_ref = zeros(1,N+1);
x0 = [theta_ref(1);phi_ref(1);dtheta_ref(1);dphi_ref(1)];
refTraj = [theta_ref; phi_ref; dtheta_ref; dphi_ref];

% Setup MPC object
mpc = LinearMPC(Anum,Bnum,Qx,Qn,Ru,stateBounds,controlBounds,N);
mpc.updateHorizonLength(N);
mpc.setupCostFunction(refTraj);
[H,f,A,b,Aeq,beq,lb,ub] = mpc.getQuadprogMatrices(x0,refTraj);


[Qout,fval] = quadprog(H,f,A,b,Aeq,beq,lb,ub);

% Extract results
xend = Nx*(N+1);
theta_out = Qout(1:Nx:xend);
phi_out = Qout(2:Nx:xend);
dtheta_out = Qout(3:Nx:xend);
dphi_out = Qout(4:Nx:xend);

X_out1 = [theta_out, phi_out,dtheta_out, dphi_out];
t_out = 0:dt:dt*50;
ustart = xend+1;
u1out = Qout(ustart+0:Nu:end);

%% MPC Main Sim loop
% MPC Loop variables
time = 0;
sim_time = 30;
x_curr = x0;

X = x0;
X_hist = [X'];
i = 1;

x_hist = [];
x_ref_hist = [];
u_hist = [];
t_hist = [];
t_horizon_hist = [];
theta_ref_hist = [];

options = optimoptions('quadprog','Display','off');
mpc_iter = 1;
fig_anim = figure(100);
while(time < sim_time)
    tic;
    
    % Set initial and goal reference
    x0 = x_curr; 
    
    t_horizon = time:dt:time+dt*(N);
    x_ref = 0.5*sin(0.5*t_horizon); % Linear position
    theta_ref = x_ref/params.r;
    phi_ref = zeros(1,N+1);

    dtheta_ref = zeros(1,N+1);
    dphi_ref = zeros(1,N+1);
    refTraj = [theta_ref; phi_ref; dtheta_ref; dphi_ref];
    
    % Set intial guess
    
    % Update dynamics
    %theta = X(1); phi = X(2);
    %dtheta = X(3); dphi = X(4);
    %Anum = double(subs(Alin));
    %Bnum = double(subs(Blin));
    %mpc = LinearMPC(Anum,Bnum,Qx,Qn,Ru,stateBounds,controlBounds,N);
    %mpc.updateHorizonLength(N);

    % Solve optimization
    [H,f,A,b,Aeq,beq,lb,ub] = mpc.getQuadprogMatrices(x0,refTraj);
    [Qout,fval] = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
    u_opt = Qout(ustart+0:Nu:end);
    u_opt1 = u_opt(1);
    
    % Unpack solution
    theta_out = Qout(1:Nx:xend);
    phi_out = Qout(2:Nx:xend);
    dtheta_out = Qout(3:Nx:xend);
    dphi_out = Qout(4:Nx:xend);
    
    x_out = [theta_out,phi_out,dtheta_out,dphi_out];
    
    % Apply control and integrate dynamics
    x_curr = Anum*x_curr + Bnum*u_opt1;
    
    % Forward simulate
    %u = u_opt1;
    %[t_out,X_out] = ode45(@(t,x)ballbot2D_dyn_mpc_wrap(t,x,params),[time-dt time],[X;u]);
    %X = [X_out(end,1); -X_out(end,2); X_out(end,3); -X_out(end,4)];
    
    % log values
    %X_hist(i,:) = [X_out(end,1); X_out(end,2); X_out(end,3); X_out(end,4)]';
    %T_hist(i) = time;
        
    x_hist = [x_hist,x_curr];
    x_ref_hist = [x_ref_hist,refTraj(:,1)];
    u_hist = [u_hist,u_opt];
    theta_ref_hist(:,:,mpc_iter) = theta_ref;
    t_horizon_hist = [t_horizon_hist;t_horizon];
    t_hist = [t_hist; time];
    t_loop(mpc_iter) = toc;
    mpc_iter = mpc_iter + 1;
    
    % Animate
    %draw_mpc_iteration(fig_anim,t_horizon,x0,refTraj,x_out,t_hist,x_hist);
    %draw_mpc_anim(fig_anim,t_horizon,x0,refTraj,x_out,t_hist,x_hist);
    
    time = time + dt;
    i=i+1;
end


%% Plot Results
figure
for i = 1:size(theta_ref_hist,3)
   plot(t_horizon_hist(i,1),theta_ref_hist(1,1,i),'g*','DisplayName','Reference')
   hold on;
   plot(t_horizon_hist(i,2:end),theta_ref_hist(1,2:end,i),'b-','DisplayName','Reference')
end
ylabel('Theta [rad]')
xlabel('Steps')
title('Ballbot ball trajectory')

%----- MPC iteration time history
figure
plot(t_loop)
xlabel('Iteration')
ylabel('time [sec]');
title('MPC iteration time history')

%----- State Evolution
figure
subplot(2,2,1)
plot(t_hist,x_ref_hist(1,:),'b-','DisplayName','Reference')
hold on
plot(t_hist, x_hist(1,:), 'r-','DisplayName','Actual')
grid on
ylabel('Ball angle theta [rad]')
xlabel('Time [sec]')

subplot(2,2,2)
plot(t_hist,x_ref_hist(2,:),'b-','DisplayName','Reference')
hold on
plot(t_hist, x_hist(2,:), 'r-','DisplayName','Actual')
grid on
ylabel('Body angle phi [rad]')
xlabel('Time [sec]')

subplot(2,2,3)
plot(t_hist,x_ref_hist(3,:),'b-','DisplayName','Reference')
hold on
plot(t_hist, x_hist(3,:), 'r-','DisplayName','Actual')
grid on
ylabel('Ball angular vel. dtheta [rad/sec]')
xlabel('Time [sec]')

subplot(2,2,4)
plot(t_hist,x_ref_hist(4,:),'b-','DisplayName','Reference')
hold on
plot(t_hist, x_hist(4,:), 'r-','DisplayName','Actual')
grid on
ylabel('Body angular vel. dphi [rad/sec]')
xlabel('Time [sec]')

%----- Control Evolution
figure
plot(t_hist,u_hist(1,:))


%% Run Animation
Anim.speed = 1;
Anim.vidName = 'MPC-QP-2d_ballbot';
Anim.avi = 1;
Anim.plotFunc = @draw_bb;
animate(t_hist(:,1),x_hist',Anim);
%animate(T_hist,X_hist,Anim);




