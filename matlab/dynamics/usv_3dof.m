function state_next = usv_3dof(state, tau, dt, disturbance)
% USV_3DOF  Fossen 3-DOF surge-sway-yaw dynamics for catamaran USV
%
%   state = [x_m, y_m, psi_rad, u_ms, v_ms, r_rads]'
%           x_m    : East position (metres, local ENU)
%           y_m    : North position (metres, local ENU)
%           psi_rad: Heading (rad, 0=North, CCW positive)
%           u_ms   : Surge velocity (m/s, body frame)
%           v_ms   : Sway velocity (m/s, body frame)
%           r_rads : Yaw rate (rad/s)
%
%   tau  = [Fx; Fy; Mz]  Forces/moment in body frame (N, N, N·m)
%   dt   = timestep (s)
%   disturbance = [Fx_d; Fy_d] external force (waves/current, N)
%
%   Returns state_next (same format)
%
% Reference: Fossen, T.I. "Handbook of Marine Craft Hydrodynamics and
%            Motion Control", Wiley 2011.

% ── Physical Parameters ─────────────────────────────────────────────────
m   = 3.5;       % Total mass [kg]  (bot + electronics + battery)
Iz  = 0.18;      % Yaw moment of inertia [kg·m²] (catamaran beam = 0.5m)
xg  = 0.0;       % CG offset from geometric centre [m]

% Added mass (frequency-independent approximation)
Xud = -0.2 * m;  % Added mass surge
Yvd = -0.8 * m;  % Added mass sway
Nrd = -0.02 * Iz;% Added yaw inertia

% Linear damping
Xu  = -4.5;      % Surge damping [N/(m/s)]
Yv  = -8.0;      % Sway damping [N/(m/s)]
Nr  = -0.8;      % Yaw damping [N·m/(rad/s)]

% Nonlinear (quadratic) damping
Xu2 = -1.2;      % [N/(m/s)²]
Yv2 = -2.5;
Nr2 = -0.15;

% ── State Extraction ────────────────────────────────────────────────────
x   = state(1);
y   = state(2);
psi = state(3);
u   = state(4);  % surge
v   = state(5);  % sway
r   = state(6);  % yaw rate

% ── Inertia Matrix (including added mass) ───────────────────────────────
M_rb = [m,     0,    m*xg ;
        0,     m,    0    ;
        m*xg,  0,    Iz   ];

M_A  = [-Xud,  0,    0   ;
         0,   -Yvd,  0   ;
         0,    0,   -Nrd  ];

M    = M_rb + M_A;     % Total inertia matrix

% ── Coriolis & Centripetal ───────────────────────────────────────────────
% Rigid-body Coriolis
C_rb = [ 0,    0,   -m*(xg*r + v) ;
         0,    0,    m*u           ;
         m*(xg*r + v), -m*u, 0    ];

% Hydrodynamic Coriolis (added mass)
C_A  = [ 0,     0,    Yvd*v ;
         0,     0,   -Xud*u ;
        -Yvd*v, Xud*u, 0   ];

C    = C_rb + C_A;

% ── Damping Matrix ───────────────────────────────────────────────────────
D    = diag([-Xu - Xu2*abs(u), ...
             -Yv - Yv2*abs(v), ...
             -Nr - Nr2*abs(r)]);

% ── External Disturbance (water current + waves) ────────────────────────
if nargin < 4 || isempty(disturbance)
    disturbance = [0; 0];
end
% Rotate current from world to body frame
tau_ext = [ disturbance(1)*cos(psi) + disturbance(2)*sin(psi); ...
           -disturbance(1)*sin(psi) + disturbance(2)*cos(psi); ...
            0];

% ── Equations of Motion: M·ν̇ = τ - C·ν - D·ν + τ_ext ─────────────────
nu   = [u; v; r];
nu_dot = M \ (tau - C*nu - D*nu + tau_ext);

% ── Kinematic Equations (body → world) ──────────────────────────────────
J = [ cos(psi), -sin(psi), 0 ;
      sin(psi),  cos(psi), 0 ;
      0,         0,        1 ];

eta     = [x; y; psi];
eta_dot = J * nu;

% ── Euler Integration ────────────────────────────────────────────────────
eta_next = eta + eta_dot * dt;
nu_next  = nu  + nu_dot  * dt;

% ── Wrap heading to [-pi, pi] ────────────────────────────────────────────
eta_next(3) = atan2(sin(eta_next(3)), cos(eta_next(3)));

state_next = [eta_next; nu_next];
end
