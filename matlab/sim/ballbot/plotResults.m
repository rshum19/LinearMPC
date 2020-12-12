function [] = plotResults(qCache,optCache,uCache,refTraj,tCache,dt,record)
%PLOTTRAJECTORY 
% qCache Cell array of states
% optCache cell array of optimized trajectory segments

N = size(qCache,2);

% Control command history
figure(1)
subplot(2,2,1)
plot(tCache,uCache(1,:))
ylabel('Theta [rad]');

subplot(2,2,3)
plot(tCache,rad2deg(uCache(2,:)))
ylabel('Phi [deg]');

subplot(2,2,2)
plot(tCache,uCache(3,:))
ylabel('dTheta [rad/s]');

subplot(2,2,4)
plot(tCache,rad2deg(uCache(4,:)))
ylabel('dPhi [deg/s]');
title('Control Trajectory');

% Actual trajectory
figure(2)
subplot(2,2,1)
plot(tCache,qCache(1,:))
ylabel('Theta [rad]');

subplot(2,2,3)
plot(tCache,rad2deg(qCache(2,:)))
ylabel('Phi [deg]');

subplot(2,2,2)
plot(tCache,qCache(3,:))
ylabel('dTheta [rad/s]');

subplot(2,2,4)
plot(tCache,rad2deg(qCache(3,:)))
ylabel('dPhi [deg/s]');
title('Actual Trajectory');

% Reference trajectory
figure(3)
subplot(2,2,1)
plot(refTraj(1,:))
ylabel('Theta [rad]');

subplot(2,2,3)
plot(rad2deg(refTraj(2,:)))
ylabel('Phi [deg]');

subplot(2,2,2)
plot(refTraj(3,:))
ylabel('dTheta [rad/s]');

subplot(2,2,4)
plot(rad2deg(refTraj(4,:)))
ylabel('dPhi [deg/s]');
title('Reference Trajectory');


end