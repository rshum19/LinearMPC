close all; clc;
addpath('../..')
addpath('../../qpOASES/interfaces/matlab')
addpath('../../osqp-matlab')

N = 10;
dt = 0.01;
dt_attitude = 0.01; % Attitude controller update rate

% System parameters

% Weights on state deviation and control input
Qx = diag([100 1 1 1]);
Qn = 10*Qx;
Ru = diag([1]);

% Bounds on states and controls
xmin = [-inf;   -10;     -inf;  -inf];
xmax = [inf;    10;  inf;    inf];
umin = [-inf];
umax = [inf];
% 
% xmin = [-inf;   -inf;     -inf;  -inf];
% xmax = [inf;    inf;  inf;   inf];
% umin = [-inf;   -inf;     -inf;  -inf];
% umax = [inf;    inf;  inf;   inf];

stateBounds = [xmin xmax];
controlBounds = [umin umax];

% Linearized dynamics
Ad = [0, 0, 1, 0;...
      0, 0, 0, 1;...
      0, -171.8039, 0, 0;...
      0, 24.3626, 0,0];
  
Bd = [0;
      0;
      5.0686;
      -0.4913];

% load ballbot params
[params] = get_ballbot2D_model_params(1);
%[params] = get_shmoo_model_params(2);
%load(params);

% Setup MPC object
mpc = LinearMPC(Ad,Bd,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','osqp');


% Reference Trajectory Generation
refTraj = generateReference('sinusoidal',dt);
%refTraj = generateReferenceTime('sinusoidal',0,dt,N);
N_traj = size(refTraj,2);

qCur = refTraj(1:4,1);
qCur = [0; 0*pi/180; 0; 0];
%qCur = zeros(4,1);

qCache = [];
optCache = [];
uCache = [];
tCache = [];
mpcRefCache = [];

% Simulate
sim_time = 30; % [sec]
step = 1;
tf = 0;
while(step < N_traj)
%while(tf < sim_time)
    tic

    % Get ref trajectory for next N steps
%     if (step + N < N_traj)
%         mpcRef = refTraj(1:4,step:step+N);
%     else % If we reach the end of the trajectory, hover at final state
%         mpcRef = refTraj(1:4,step:end);
%         
%         lastState = mpcRef(1:4,end);
%         lastState(2) = 0; % No velocity, no orientation
%         lastState(4) = 0;
%         
%         mpcRef = [mpcRef, repmat(lastState,1,N+1-size(mpcRef,2))];
%     end

%     refTraj = generateReferenceTime('sinusoidal',tf,dt,N+1);
    %mpcRef = refTraj(1:4,:);
    mpcRef = zeros(4,N+1);
    mpcRef(1,:) = sin(0.5*(tf:dt:(tf+dt*N)));
    %mpcRef(:,1) = qCur;
    
    % Collect MPC Control (roll,pitch,thrust commands, all in world frame)
    tic
    [Qout,fval] = mpc.solve(qCur,mpcRef);
    toc
    [uOpt,optTraj] = mpc.getOutput(Qout); % Collect first control, optimzied state traj 
    
    u = -uOpt(:,1);
    u(2) = 2*u(2);
    %u2 = -uOpt(:,2);
    % Simulate with ode45
    t0 = (step-1)*dt;
    tf = t0+dt;
    [~,qNext] = ode45(@(t,q) ballbotDynamicsOL(t,q,u,params),t0:dt_attitude:tf,qCur);
    qCur = qNext(end,:)';
    
    % Store outputs and update step
    qCache =[qCache, qCur];
    optCache = [optCache, optTraj];
    uCache  = [uCache, u];
    tCache = [tCache,t0];
    mpcRefCache = [mpcRefCache, mpcRef(:,1)];
    step = step + 1;
    fprintf("Time: %d ", t0);
end

%plotTrajectory(qCache,optCache,uCache,refTraj,dt,false)
plotResults(qCache,optCache,uCache,mpcRefCache,tCache,dt,false)


