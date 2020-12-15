function [] = plotResults(qCache,optCache,uCache,refTraj,tCache,dt,record)
%PLOTTRAJECTORY 
% qCache Cell array of states
% optCache cell array of optimized trajectory segments

N = size(qCache,2);
r =0.105838037;
% Control command history
figure(1)
subplot(2,2,1)
plot(tCache,uCache(1,:)*r)
ylabel('Pos [m]');

subplot(2,2,3)
plot(tCache,rad2deg(uCache(2,:)))
ylabel('Phi [deg]');

subplot(2,2,2)
plot(tCache,uCache(3,:)*r)
ylabel('Vel [m/s]');

subplot(2,2,4)
plot(tCache,rad2deg(uCache(4,:)))
ylabel('dPhi [deg/s]');
title('Control Trajectory');

% Actual trajectory
figure(2)
subplot(2,2,1)
plot(tCache,qCache(1,:)*r)
ylabel('Pos [m]');

subplot(2,2,3)
plot(tCache,rad2deg(qCache(2,:)))
ylabel('Phi [deg]');

subplot(2,2,2)
plot(tCache,qCache(3,:)*r)
ylabel('Vel [m/s]');

subplot(2,2,4)
plot(tCache,rad2deg(qCache(3,:)))
ylabel('dPhi [deg/s]');
title('Actual Trajectory');

% Reference trajectory
figure(3)
subplot(2,2,1)
plot(tCache,refTraj(1,:)*r)
ylabel('Pos [m]');

subplot(2,2,3)
plot(tCache,rad2deg(refTraj(2,:)))
ylabel('Phi [deg]');

subplot(2,2,2)
plot(tCache,refTraj(3,:)*r)
ylabel('Vel [m/s]');

subplot(2,2,4)
plot(tCache,rad2deg(refTraj(4,:)))
ylabel('dPhi [deg/s]');
title('Reference Trajectory');


end