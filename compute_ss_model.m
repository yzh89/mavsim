function [A_lon,B_lon,A_lat,B_lat, A, B] = compute_ss_model(filename,x_trim,u_trim)
% x_trim is the trimmed state,
% u_trim is the trimmed input
  
[A,B,C,D]=linmod(filename,x_trim,u_trim);
% t = 0:0.1:10;
% u = ones(length(t),4)*diag([0.0,0.001, 0.001,0.0])';
% sys = ss(A,B,eye(12,12),0);
% [y,t,x] = lsim(sys,u,t,zeros(12,1));
% plot3(x(:,1), x(:,2), x(:,3))

P_lat = zeros(5,12);
P_lat(1,5)=1;
P_lat(2,10)=1;
P_lat(3,12)=1;
P_lat(4,7)=1;
P_lat(5,9)=1;

A_lat=P_lat*A*P_lat';
B_lat=P_lat*B;


P_lon = zeros(5,12);
P_lon(1,4)=1;
P_lon(2,6)=1;
P_lon(3,11)=1;
P_lon(4,8)=1;
P_lon(5,3)=1;

A_lon=P_lon*A*P_lon';
B_lon=P_lon*B;