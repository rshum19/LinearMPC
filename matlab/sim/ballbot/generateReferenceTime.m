function refTraj = generateReferenceTime(name,t,dt, N_traj)

% Ball radius
r = 0.105838037; % ballbot ball radius [m]
%r = 0.0762; % shmoobot ball radius [m]

if strcmp(name,'sinusoidal')
    %N_traj = 200;
    tend = t + dt*(N_traj-1);
    x_ref = sin(0.05*(t:dt:tend));
    theta_ref = x_ref/r;
    phi_ref = zeros(1,N_traj);

    dtheta_ref = [0, diff(theta_ref)/dt];
    dphi_ref = [0, diff(phi_ref)/dt];

    refTraj = [theta_ref; phi_ref; dtheta_ref; dphi_ref; x_ref];    
elseif strcmp(name,'straight')
    xref = [(0:0.1:5), (5:-0.1:0)];
    yref = zeros(size(xref));
    zref = zeros(size(xref));
    rollref = zeros(size(zref));
    pitchref = zeros(size(zref));

    dxref = [0, diff(xref)/dt];
    dyref = [0, diff(yref)/dt];
    dzref = [0, diff(zref)/dt]; 

    refTraj = [xref;yref;zref;dxref;dyref;dzref;rollref;pitchref];
end