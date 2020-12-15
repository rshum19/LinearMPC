function dx = ballbotDynamicsOL(t,x,u,model_params)
%ballbotDynamics Ballbot dynamics function, includes low level balancing
%controller


% Nonlinear dynamics
model_nl_dyn = @(t,x,u)ballbot2D_dyn_wrap(t,x,u,model_params);

% Linearized dynamics
Ad = [0, 0, 1, 0;...
      0, 0, 0, 1;...
      0, -171.8039, 0, 0;...
      0, 24.3626, 0,0];
  
Bd = [0;
      0;
      5.0686;
      -0.4913];

%dx = model_nl_dyn(t,x,u);
dx = Ad*x + Bd*u;

% check for fall
if (abs(x(2)) >= pi/2)
  dx = zeros(size(x));
end
end

