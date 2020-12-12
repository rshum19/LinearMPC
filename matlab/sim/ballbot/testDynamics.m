
t0 = 0.0;
dt_attitude = 0.01;
tf = 30.0;
[params] = get_ballbot2D_model_params(1);
%[params] = get_shmoo_model_params(2);

u = zeros(4,1);
qCur = [0;1*pi/180;0;0;];
qd.ref_traj = zeros(4,1);

[tHist, xHist] = ode45(@(t,q) ballbotDynamics(t,q,qd,params),t0:dt_attitude:tf,qCur);

figure(3)
subplot(2,2,1)
plot(tHist,xHist(:,1)*params.r)
ylabel('xPos [m]');

subplot(2,2,3)
plot(tHist,rad2deg(xHist(:,2)))
ylabel('Phi [deg]');

subplot(2,2,2)
plot(tHist,xHist(:,3)*params.r)
ylabel('xVel[m/s]');

subplot(2,2,4)
plot(tHist,rad2deg(xHist(:,4)))
ylabel('dPhi [deg/s]');