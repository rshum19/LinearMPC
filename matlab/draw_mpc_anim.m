function draw_mpc_anim(fig_handle,time,x0,xref,xsol,thist,xhist)%,usol,xlog,ulog)


figure(fig_handle)

%----- Extract horizon
N = size(xsol,1);

%----- draw ballbot
options.color_alpha = 0.1;

for i = N:-1:1
    xcurr = xsol(i,:);
    if(i == 1)
        draw_bb(time,xcurr);
    else
        draw_bb(time,xcurr,options);
    end
    hold on;
end


% format figure
title(sprintf(' t=%6.4f', time(1)));
grid on;
hold off;

end







