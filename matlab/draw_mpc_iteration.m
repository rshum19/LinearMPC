function draw_mpc_iteration(fig_handle,time,x0,xref,xsol,thist,xhist)%,usol,xlog,ulog)


figure(fig_handle)

%----- Extract horizon
N = size(xsol,1);

%----- draw reference
plot(time,xref(1,:),'b+','MarkerSize',10,'LineWidth',2);
hold on;

%----- draw opt solution
plot(time,xsol(:,1),'r','MarkerSize',10,'LineWidth',2);

%----- draw trajectory
plot(thist,xhist(1,:),'g','MarkerSize',10,'LineWidth',2);

% format figure
title(sprintf(' t=%6.4f', time(1)));
grid on;
%axis equal;
axis([0,35,-5,5])
%pause(0.2);
hold off;

end







