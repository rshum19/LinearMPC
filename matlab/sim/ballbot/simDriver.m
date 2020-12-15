% clear workspace
clear all; close all; clc;
addpath('../..')
addpath('../../qpOASES/interfaces/matlab')
addpath('../../osqp-matlab')

N = 10;
dt = 0.01;
dt_attitude = 0.01; % Attitude controller update rate

% System parameters

% Weights on state deviation and control input
Qx = diag([100 10 10 1]);
Qn = 10*Qx;
Ru = diag([1 1 1 1]);

% Bounds on states and controls
% xmin = [-inf;   -5*pi/180;     -inf;  -inf];
% xmax = [inf;    5*pi/180;  inf;    inf];
% umin = [-10; -3*pi/180; -100*pi/180; -100*pi/180];
% umax = [10; 3*pi/180; 100*pi/180; 100*pi/180];

xmin = [-inf; -inf; -inf;  -inf];
xmax = [inf;    inf;  inf;    inf];
umin = [-inf; -inf; -inf; -inf];
umax = [inf; inf; inf; inf];
 
stateBounds = [xmin xmax];
controlBounds = [umin umax];

% Linearized dynamics
A = [0, 0, 1, 0;...
      0, 0, 0, 1;...
      0, -171.8039, 0, 0;...
      0, 24.3626, 0,0];
  
B = [0;
      0;
      5.0686;
      -0.4913];
  
Klqr =  -[-1.0000 -173.1954   -2.0268  -48.6683];

Acld = A - B*Klqr;
Bcld = B*Klqr;

% load ballbot params
[params,~] = get_ballbot2D_model_params();

% Setup MPC object
mpc = LinearMPC(Acld,Bcld,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','osqp');

% Reference Trajectory Generation
motion_name = 'straight';
%refTraj = generateReferenceTime('sinusoidal',0,dt,N);
N_traj = 200;

% Initial state
qCur = [0; 0*pi/180; 0; 0];

qCache = [];
optCache = [];
uCache = [];
tCache = [];
mpcRefCache = [];

% Simulate
sim_time = 30; % [sec]
step = 1;
tf = 0;
while(step <  4000)
    tic
%     
%     if(tf < 30)
%         mpcRef = zeros(4,N+1);
%         mpcRef(1,:) = 1*(tf:dt:(tf+dt*N));
%         mpcRef(3,:) = 1*ones(1,N+1);
%     else
%        mpcRef(3,:) = zeros(1,N+1);
%     end
    
%     % Sample reference trajectory
    mpcRef_full = generateReferenceTime(motion_name,tf,dt,N+1);
    mpcRef = mpcRef_full(1:4,:); 

    % Collect MPC Control (roll,pitch,thrust commands, all in world frame)
    tic
    [Qout,fval] = mpc.solve(qCur,mpcRef);
    toc
    [uOpt,optTraj] = mpc.getOutput(Qout); % Collect first control, optimzied state traj 
        

    u = 0.2*[0;uOpt(2);uOpt(3);0];

    % Simulate with ode45
    t0 = (step-1)*dt;
    tf = t0+dt;
    [~,qNext] = ode45(@(t,q) ballbotDynamics(t,q,u,params),t0:dt_attitude:tf,qCur);
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

plotResults(qCache,optCache,uCache,mpcRefCache,tCache,dt,false)


