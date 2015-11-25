function [x_trim,u_trim] = compute_trim(filename, Va, gamma, R, P)
% Va is the desired airspeed (m/s)
% gamma is the desired flight path angle (radians)
% R is the desired radius (m) - use (+) for right handed orbit, 
%                                   (-) for left handed orbit


% setup x0 and ix etc
x0=[
    P.pn0;...  % initial North position
    P.pe0;...  % initial East position
    P.pd0;...  % initial Down position (negative altitude)
    P.u0;... % initial velocity along body x-axis
    P.v0;...  % initial velocity along body y-axis
    P.w0;...  % initial velocity along body z-axis
    P.phi0;...  % initial roll angle
    P.theta0;...  % initial pitch angle
    P.psi0;...  % initial yaw angle
    P.p0;...  % initial body frame roll rate
    P.q0;...  % initial body frame pitch rate
    P.r0;...  % initial body frame yaw rate
    ];

ix=[];

u0=[0;... %delta_e
    0;... %delta_a
    0;... %delta_r
    1;... %delta_t
    ];

iu=[];

y0=[Va;... %Va
    0;... %alpha
    0;... %beta
    ];

iy=[1,3];

%hold derivative of hdot and psidot accordingly
dx0=[0;...              %1 - pndot
     0;...              %2 - pedot
     -Va*sin(gamma);... %3 - pddot
     0;...              %4 - udot
     0;...              %5 - vdot
     0;...              %6 - wdot
     0;...              %7 - phidot
     0;...              %8 - thetadot
     0;...              %9 - psidot
     0;...              %10 - pdot
     0;...              %11 - qdot
     0;...              %12 - rdot
     ];
if R~=Inf
    dx0(9)=Va*cos(gamma)/R;
end
%speicify which derivative to hold constant
idx = [3;4;5;6;7;8;9;10;11;12];

% compute trim conditions
[x_trim,u_trim,y_trim,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);

% check to make sure that the linearization worked (should be small)
norm(dx_trim(3:end)-dx0(3:end))

