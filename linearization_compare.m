% linearize the equations of motion around trim conditions
param_chap5;

t = 0:0.1:10;
u = ones(length(t),4)*diag([0.01,0.00, 0.00,0.0])';
sys = ss(A,B,eye(12,12),0);
[y,t,x_bar] = lsim(sys,u,t,zeros(12,1));

simOut = sim('mavsim_chap5_no_plot','SaveState','on','StateSaveName','xout');
x_star = get(simOut,'xout');

set_param('mavsim_chap5_no_plot/u trim 1','Value','u_trim(1)+0.01');
simOut_2 = sim('mavsim_chap5_no_plot','SaveState','on','StateSaveName','xout');
x = get(simOut_2,'xout');

set_param('mavsim_chap5_no_plot/u trim 1','Value','u_trim(1)');


%%
figure
plot(t,x(:,3), t, x_bar(:,3)+ x_star(:,3));
figure
plot(t,x_bar(:,3))