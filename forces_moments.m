% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
        
    % compute wind data in NED
    
    w_n = w_ns+ [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)]*[u_wg, v_wg, w_wg]';
    w_e = w_es+ [cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)]*[u_wg, v_wg, w_wg]';
    w_d = w_ds+ [-sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]*[u_wg, v_wg, w_wg]';    
    
    % compute air data
    Va = 0;
    alpha = 0;
    beta = 0;
    
    Cx = - P.C_D_alpha*cos(alpha)
    
    % compute external forces and torques on aircraft
    Force(1) =  -P.mass*g*sin(theta)+1/2*P.rho*Va^2*P.S_wing*;
    Force(2) =  0;
    Force(3) =  0;
    
    Torque(1) = 0;
    Torque(2) = 0;   
    Torque(3) = 0;
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



