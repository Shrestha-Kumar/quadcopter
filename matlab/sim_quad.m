function out = sim_quad(sc, p, ctrl)
%SIM_QUAD  Closed-loop simulation of the quadcopter with the designed controller.
%   out = sim_quad(sc, p, ctrl)
%   sc (scenario) fields (all optional, defaults in brackets):
%     t_end [10]            simulation length [s]
%     roll_ref, pitch_ref, yaw_ref   @(t) reference angle [rad]   [0]
%     alt_ref               @(t) altitude reference [m]           [0]
%     dist                  @(t) 6x1 disturbance [Fn Fe Fd tx ty tz] [zeros]
%     noise                 true/false sensor noise               [false]
%     mScale, JScale, kTScale, tauMScale   true-plant multipliers  [1]
%     sat                   true/false actuator saturation        [true]
%     delay                 controller sample delay (samples)     [0]
%   The controller always uses the NOMINAL parameters p; the true plant uses the scaled ones,
%   which is how parameter uncertainty is investigated.
%   Physics: RK4 at 1 ms, controller at p.Ts (200 Hz, zero-order hold).

def = struct('t_end',10,'roll_ref',@(t)0,'pitch_ref',@(t)0,'yaw_ref',@(t)0,'alt_ref',@(t)0, ...
             'dist',@(t)zeros(6,1),'noise',false,'mScale',1,'JScale',1,'kTScale',1, ...
             'tauMScale',1,'sat',true,'delay',0,'seed',1);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(sc,f{i}), sc.(f{i}) = def.(f{i}); end, end
rng(sc.seed);

% True plant (perturbed)
pt = p;
pt.m = p.m*sc.mScale;
pt.J = p.J*sc.JScale; pt.Ixx = pt.J(1,1); pt.Iyy = pt.J(2,2); pt.Izz = pt.J(3,3);
pt.tau_m = p.tau_m*sc.tauMScale;

dt = 1e-3;
Nctrl = round(p.Ts/dt);
N = round(sc.t_end/dt);

x = zeros(16,1);
x(13:16) = p.hover_T/4*ones(4,1);           % start in hover
Tcmd = x(13:16);
Tmin = 0; Tmaxm = p.Tmax;

% Controller states
Iint = zeros(3,1); prev_meas = zeros(3,1); dfilt = zeros(3,1);
Ialt = 0; prev_alt = 0; dalt = 0; Ilqr = zeros(3,1);
buf = repmat(struct('Tcmd',Tcmd),max(sc.delay,0)+1,1);

% Logs
L.t = (0:N-1)'*dt;
L.eul = zeros(N,3); L.pqr = zeros(N,3); L.pos = zeros(N,3);
L.ref = zeros(N,4); L.T = zeros(N,4); L.Tcmd = zeros(N,4); L.tau = zeros(N,3);
tau_cmd = zeros(3,1);

for k = 1:N
    t = (k-1)*dt;
    % ---------------- controller update ----------------
    if mod(k-1, Nctrl) == 0
        eul_m = x(7:9); pqr_m = x(10:12); alt_m = -x(3);
        if sc.noise
            eul_m = eul_m + p.sig_ang*randn(3,1);
            pqr_m = pqr_m + p.sig_gyro*randn(3,1);
            alt_m = alt_m + p.sig_alt*randn;
        end
        ref = [sc.roll_ref(t); sc.pitch_ref(t); sc.yaw_ref(t)];
        e_ang = ref - eul_m;
        e_ang(3) = atan2(sin(e_ang(3)), cos(e_ang(3)));

        switch ctrl.type
            case 'PID'
                rate_sp = ctrl.Kang(:).*e_ang;
                rate_sp(1:2) = max(min(rate_sp(1:2), ctrl.rate_max), -ctrl.rate_max);
                rate_sp(3)   = max(min(rate_sp(3), ctrl.yawrate_max), -ctrl.yawrate_max);
                e_r = rate_sp - pqr_m;
                Iint = Iint + ctrl.Ki(:).*e_r*p.Ts;
                Iint = max(min(Iint, ctrl.Ilim(:)), -ctrl.Ilim(:));
                d_raw = -(pqr_m - prev_meas)/p.Ts;
                dfilt = dfilt + ctrl.d_alpha*(d_raw - dfilt);
                prev_meas = pqr_m;
                tau_cmd = ctrl.Kp(:).*e_r + Iint + ctrl.Kd(:).*dfilt;
            case 'LQR'
                Ilqr = Ilqr - 0 + (-e_ang)*p.Ts;             % integral of (angle - ref)
                for a = 1:3
                    xs = [Ilqr(a); -e_ang(a); pqr_m(a)];
                    tau_cmd(a) = -ctrl.Klqr(a,:)*xs;
                end
        end
        tau_cmd = max(min(tau_cmd, ctrl.tau_max(:)), -ctrl.tau_max(:));

        % Altitude PID with hover feed-forward and tilt compensation
        alt_ref = sc.alt_ref(t);
        e_alt = alt_ref - alt_m;
        if k == 1, prev_alt = alt_m; end
        dalt = dalt + 0.05*((-(alt_m - prev_alt)/p.Ts) - dalt);
        prev_alt = alt_m;
        u_pd = ctrl.alt.kp*e_alt + ctrl.alt.kd*dalt;
        % Conditional integration (anti-windup): only integrate when error is small or I helps reduce it
        if abs(e_alt) < 0.60 || (Ialt * e_alt < 0)
            Ialt = max(min(Ialt + ctrl.alt.ki*e_alt*p.Ts, ctrl.alt.ilim), -ctrl.alt.ilim);
        end
        u_alt = u_pd + Ialt;
        tilt = max(cos(eul_m(1))*cos(eul_m(2)), 0.5);
        Tt = (p.hover_T + u_alt)/tilt;

        % Allocation (torques -> motor thrusts), see quad_dynamics for the sign convention
        tp = tau_cmd(1); tt = tau_cmd(2); ty = tau_cmd(3);
        Tc = [ Tt/4 - tp/(4*p.d) + tt/(4*p.d) + ty/(4*p.kq);
               Tt/4 + tp/(4*p.d) + tt/(4*p.d) - ty/(4*p.kq);
               Tt/4 + tp/(4*p.d) - tt/(4*p.d) + ty/(4*p.kq);
               Tt/4 - tp/(4*p.d) - tt/(4*p.d) - ty/(4*p.kq) ];
        if sc.sat, Tc = max(min(Tc, Tmaxm), p.Tidle); end

        % optional sample delay
        buf = [struct('Tcmd',Tc); buf(1:end-1)];
        Tcmd = buf(end).Tcmd;
    end

    % ---------------- plant (RK4) ----------------
    Tcmd_real = Tcmd*sc.kTScale;           % thrust-constant error: produced thrust != commanded
    d = sc.dist(t);
    k1 = quad_dynamics(x,        Tcmd_real, pt, d);
    k2 = quad_dynamics(x+dt/2*k1, Tcmd_real, pt, d);
    k3 = quad_dynamics(x+dt/2*k2, Tcmd_real, pt, d);
    k4 = quad_dynamics(x+dt*k3,   Tcmd_real, pt, d);
    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    % ---------------- log ----------------
    L.eul(k,:) = x(7:9)'; L.pqr(k,:) = x(10:12)'; L.pos(k,:) = x(1:3)';
    L.ref(k,:) = [sc.roll_ref(t) sc.pitch_ref(t) sc.yaw_ref(t) sc.alt_ref(t)];
    L.T(k,:) = x(13:16)'; L.Tcmd(k,:) = Tcmd'; L.tau(k,:) = tau_cmd';
end
out = L;
out.alt = -L.pos(:,3);
out.ctrl = ctrl.type;
end
