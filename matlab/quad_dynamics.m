function xdot = quad_dynamics(x, Tcmd, p, dist)
%QUAD_DYNAMICS  Nonlinear 6-DOF quadcopter + first-order motor model.
%   State x (16x1):
%     x(1:3)   position NED [m]          (altitude h = -x(3))
%     x(4:6)   velocity NED [m/s]
%     x(7:9)   Euler angles [phi theta psi] [rad]  (roll, pitch, yaw)
%     x(10:12) body rates [p q r] [rad/s]
%     x(13:16) motor thrusts T1..T4 [N]
%   Tcmd (4x1) commanded motor thrusts [N] (held constant over the call).
%   p     parameter struct (true plant, see params.m).
%   dist  6x1 disturbance [Fn Fe Fd (N, inertial)  tx ty tz (N m, body)].

phi = x(7); th = x(8); psi = x(9);
pqr = x(10:12);
T   = x(13:16);

% Rotation body -> inertial (ZYX)
cph = cos(phi); sph = sin(phi); cth = cos(th); sth = sin(th); cps = cos(psi); sps = sin(psi);
R = [cth*cps, sph*sth*cps - cph*sps, cph*sth*cps + sph*sps;
     cth*sps, sph*sth*sps + cph*cps, cph*sth*sps - sph*cps;
     -sth,    sph*cth,               cph*cth];

% Forces and torques from the four motors
Ttot   = sum(T);
tau_phi   = p.d*( T(2) + T(3) - T(1) - T(4) );     % + right side down
tau_theta = p.d*( T(1) + T(2) - T(3) - T(4) );     % + nose up
tau_psi   = p.kq*( T(1) + T(3) - T(2) - T(4) );    % + clockwise (CW props 2,4 react CCW)
tau = [tau_phi; tau_theta; tau_psi] + dist(4:6);

% Translational dynamics (NED, thrust along -z body)
vel = x(4:6);
acc = [0; 0; p.g] + R*[0; 0; -Ttot]/p.m - (p.cd/p.m)*vel + dist(1:3)/p.m;

% Euler kinematics
eul_dot = [ pqr(1) + (pqr(2)*sph + pqr(3)*cph)*tan(th);
            pqr(2)*cph - pqr(3)*sph;
            (pqr(2)*sph + pqr(3)*cph)/cth ];

% Rotational dynamics
pqr_dot = p.J \ (tau - cross(pqr, p.J*pqr));

% Motor dynamics
T_dot = (Tcmd - T)/p.tau_m;

xdot = [vel; acc; eul_dot; pqr_dot; T_dot];
end
