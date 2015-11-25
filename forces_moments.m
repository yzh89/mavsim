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
    
    Rb_v = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
            cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
            -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]';    
    
    w_n = w_ns+ [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)]*[u_wg, v_wg, w_wg]';
    w_e = w_es+ [cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)]*[u_wg, v_wg, w_wg]';
    w_d = w_ds+ [-sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]*[u_wg, v_wg, w_wg]';    
    
    % compute air data
    vb_a = [u, v, w]'- Rb_v*[w_ns, w_es, w_ds]' - [u_wg, v_wg, w_wg]';
    
    Va = norm(vb_a);
    alpha = atan2(vb_a(3),vb_a(1));
    beta = asin(vb_a(2)/Va);
    
    Cx = - (P.C_D_alpha*alpha + P.C_D_0)* cos(alpha) + (P.C_L_alpha*alpha + P.C_L_0)*sin(alpha);
    Cxq = - P.C_D_q*cos(alpha) +P.C_L_q*sin(alpha);
    Cxd = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
    Cz = -(P.C_D_alpha*alpha + P.C_D_0)*sin(alpha) - (P.C_L_alpha*alpha + P.C_L_0)*cos(alpha);
    Czq = - P.C_D_q*sin(alpha) -P.C_L_q*cos(alpha);
    Czd = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);
    % compute external forces and torques on aircraft
    Force(1) =  -P.mass*P.gravity*sin(theta)+1/2*P.rho*Va^2*P.S_wing*(Cx + Cxq*P.c/2/Va*q + Cxd*delta_e) + 1/2*P.rho*P.S_prop*P.C_prop*((P.k_motor*delta_t)^2-Va^2);
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi)+1/2*P.rho*Va^2*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b/2/Va*p + P.C_Y_r*P.b/2/Va*r + P.C_Y_delta_a* delta_a + P.C_Y_delta_r*delta_r);
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi)+1/2*P.rho*Va^2*P.S_wing*(Cz + Czq*P.c/2/Va*q + Czd*delta_e);
    
    Torque(1) = 1/2*P.rho*Va^2*P.S_wing*P.b*(P.C_ell_0+P.C_ell_beta*beta + P.C_ell_p*P.b/2/Va*p + P.C_ell_r*P.b/2/Va*r + P.C_ell_delta_a * delta_a + P.C_ell_delta_r*delta_r) - P.k_T_P*P.k_Omega^2*delta_t^2;
    Torque(2) = 1/2*P.rho*Va^2*P.S_wing*P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c/2/Va*q + P.C_m_delta_e*delta_e);   
    Torque(3) = 1/2*P.rho*Va^2*P.S_wing*P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b/2/Va*p + P.C_n_r*P.b/2/Va*r + P.C_n_delta_a*delta_a + P.C_n_delta_r* delta_r);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



