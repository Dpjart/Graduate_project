function [R, Tth] = rov_kinematics(phi, th, psi)
%ROV_KINEMATICS  Body-to-NED rotation and Euler-rate transform (ZYX).
%   [R, Tth] = rov_kinematics(phi, theta, psi)
%   R   : 3x3, p_dot_NED = R * [u v w]'
%   Tth : 3x3, [phi_dot theta_dot psi_dot]' = Tth * [p q r]'
%   Tth is singular at theta = +-90 deg (not reached by a BG-stable ROV).

cphi = cos(phi); sphi = sin(phi);
cth  = cos(th);  sth  = sin(th);  tth = tan(th);
cpsi = cos(psi); spsi = sin(psi);

R = [ cpsi*cth,  -spsi*cphi + cpsi*sth*sphi,   spsi*sphi + cpsi*cphi*sth ;
      spsi*cth,   cpsi*cphi + sphi*sth*spsi,  -cpsi*sphi + sth*spsi*cphi ;
     -sth,        cth*sphi,                    cth*cphi ];

Tth = [ 1,  sphi*tth,   cphi*tth ;
        0,  cphi,      -sphi ;
        0,  sphi/cth,   cphi/cth ];
end
