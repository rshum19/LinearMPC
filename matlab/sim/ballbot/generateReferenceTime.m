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
    tend = 5;
    xVel = 0.01;
    if(t < tend)
        x_ref = xVel*(t:dt:(t+dt*(N_traj-1)));
        theta_ref = x_ref/r;
        phi_ref = zeros(1,N_traj);
        
        dtheta_ref = [0, xVel/r*ones(1,(N_traj-1))];
        dphi_ref = [0, diff(phi_ref)/dt];    
    else
        x_ref = xVel*tend*ones(1,N_traj);
        theta_ref = x_ref/r;
        phi_ref = zeros(1,N_traj);
        
        dtheta_ref = 0*[0, diff(theta_ref)/dt];
        dphi_ref = [0, diff(phi_ref)/dt];   
    end

    refTraj = [theta_ref; phi_ref; dtheta_ref; dphi_ref; x_ref];    
elseif strcmp(name,'square')
    tramp = 5.0;
    Amp = 0.3;
    if(t < tramp)
        theta_ref =zeros(1,N_traj);
        phi_ref =zeros(1,N_traj);
        dtheta_ref =zeros(1,N_traj);
        dphi_ref =zeros(1,N_traj);
        x_ref = zeros(1,N_traj);
    else
        x_ref = Amp*ones(1,N_traj);
        theta_ref = ones(1,N_traj);
        phi_ref =zeros(1,N_traj);
        dtheta_ref =zeros(1,N_traj);
        dphi_ref =zeros(1,N_traj);
    end

    refTraj = [theta_ref; phi_ref; dtheta_ref; dphi_ref; x_ref];    

elseif strcmp(name,'zero')
    
    refTraj = zeros(5,N_traj);

end