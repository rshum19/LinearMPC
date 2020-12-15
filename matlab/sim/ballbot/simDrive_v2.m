% clear workspace
clear all; close all; clc;
addpath('../..')
addpath('../../qpOASES/interfaces/matlab')
addpath('../../osqp-matlab')

N = 20;
dt = 0.01;
dt_attitude = 0.001; % Attitude controller update rate

% System parameters

% Weights on state deviation and control input
Qx = diag([1000 1 1 100]);
Qn = 10*Qx;
Ru = 1;

% Bounds on states and controls
xmin = [-inf;   -5*pi/180;     -inf;  -inf];
xmax = [inf;    5*pi/180;  inf;    inf];
umin = -20;
umax = 20;
 
stateBounds = [xmin xmax];
controlBounds = [umin umax];

% Linearized dynamics
A = [0, 0, 1, 0;...
      0, 0, 0, 1;...
      0, -31.4603, 0, 0;...
      0, 16.333, 0,0];
  
B = [0;
      0;
      13.4993;
      -2.8567];
  

% Ad = expm(A*dt);
% Bd = expm(A*dt - eye(size(A)))*pinv(A)*B;
Ad = A;
Bd = B;

% load ballbot params
%[params,~] = get_ballbot2D_model_params();
params = [];

% Setup MPC object
mpc = LinearMPC(Ad,Bd,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','osqp');

% Reference Trajectory Generation
refTraj = generateReference('sinusoidal',dt);
%refTraj = generateReferenceTime('sinusoidal',0,dt,N);
N_traj = size(refTraj,2);

% Initial state
qCur = refTraj(1:4,1);
qCur = [0; deg2rad(0); 0; 0];

qCache = qCur;
optCache = [];
uCache = [];
tf = 0;
tCache = tf;
mpcRefCache = [];

% Simulate
sim_time = 30; % [sec]
step = 1;
while(step <  N_traj)
    tic
    
    if(tf < 20)
        mpcRef = zeros(4,N+1);
        mpcRef(1,:) = 0.05*(tf:dt:(tf+dt*N));
    else
       
    end
%     
%     % Sample reference trajectory
%     mpcRef_full = generateReferenceTime('straight',tf,dt,N+1);
%     mpcRef = mpcRef_full(1:4,:); 

    % Collect MPC Control (roll,pitch,thrust commands, all in world frame)
    tic
    [Qout,fval] = mpc.solve(qCur,mpcRef);
    toc
    [uOpt,optTraj] = mpc.getOutput(Qout); % Collect first control, optimzied state traj 
        

    %u = -uOpt;
    u = -uOpt;
    % Simulate with ode45
    t0 = (step-1)*dt;
    tf = t0+dt;
    [~,qNext] = ode45(@(t,q) ballbotDynamicsOL(t,q,u,params),t0:dt_attitude:tf,qCur);
    %qCur = A*qCur + B*u;
    qCur = qNext(end,:)';
    
    % Store outputs and update step
    qCache =[qCache, qCur];
    optCache = [optCache, optTraj(:,1)];
    uCache  = [uCache, u];
    tCache = [tCache,tf];
    mpcRefCache = [mpcRefCache, mpcRef(:,1)];
    step = step + 1;
    fprintf("Time: %d ", t0);
end
 
%plotTrajectory(qCache,optCache,uCache,refTraj,dt,false)
plotResultsOL(qCache,optCache,uCache,mpcRefCache,tCache,dt,false)

figure
subplot(3,1,1)
plot(tCache, refTraj(2,:))

subplot(3,1,2)
plot(tCache(1:end-1), optCache(3,:))

subplot(3,1,3)
plot(tCache(1:end-1), qCache(2,:))

