% dynamics.m
function dxdt = dynamics(t, x, m, g, Ixx, Iyy, Izz, alpha_angle, rho, Iw, r_w, s, a, T0, x_0, Nx, Ny, Nz, N_phi, N_theta, N_psi, ...
    k1, k2, k3, k4, k5, k6, k7, k8, k9, k10, k11, k12, k13, k14, threshold, max_tether_length, waypoints, n, k_spring, d_damping, m_mass)

persistent current_waypoint
if isempty(current_waypoint)
    current_waypoint = 1;
end

% Extract state variables
x1 = x(1);  % x position
x2 = x(2);  % x velocity
x3 = x(3);  % y position
x4 = x(4);  % y velocity
x5 = x(5);  % z position
x6 = x(6);  % z velocity
x7 = x(7);  % phi (roll)
x8 = x(8);  % phi dot (roll rate)
x9 = x(9);  % theta (pitch)
x10 = x(10); % theta dot (pitch rate)
x11 = x(11); % psi (yaw)
x12 = x(12); % psi dot (yaw rate)
x13 = x(13); % Tether Length
x14 = x(14); % Tether length rate of change

% State variables for cable
x_cable = x(15:end);

% Saturation function for sine
S_phi = sin(x7); if abs(S_phi) < threshold, S_phi = threshold; end
S_theta = sin(x9); if abs(S_theta) < threshold, S_theta = threshold; end
S_psi = sin(x11); if abs(S_psi) < threshold, S_psi = threshold; end

C_phi = cos(x7);
C_theta = cos(x9);
C_psi = cos(x11);

% Reference trajectory
% Update current waypoint if the quadcopter is close to the desired position
if norm([x1 - waypoints(current_waypoint, 1), x3 - waypoints(current_waypoint, 2), x5 - waypoints(current_waypoint, 3)]) < 0.1
    current_waypoint = current_waypoint + 1;
end
if current_waypoint > size(waypoints, 1)
    current_waypoint = size(waypoints, 1); % Stay at the last waypoint
end

% Desired positions
x1_ref = waypoints(current_waypoint, 1);
x3_ref = waypoints(current_waypoint, 2);
x5_ref = waypoints(current_waypoint, 3);
x7_ref = 0; 
x9_ref = pi/6; 
x11_ref = 0;

% Catenary parameter
beta = atan(sinh((x1 - x_0) / a));

% Calculate tension force on body
Th = T0 + rho * s * x5 * g;
Tx = Th * cos(alpha_angle) * sin(beta);
Ty = -Th * cos(alpha_angle) * cos(beta);
Tz = -Th * sin(alpha_angle);

% Calculate desired tether length based on current state
x13_ref = sqrt(2 * a * sinh(x1 / (2 * a)) + x5^2);

% Error calculations
e1 = x1 - x1_ref;
e2 = x2;
e3 = x3 - x3_ref;
e4 = x4;
e5 = x5 - x5_ref;
e6 = x6;
e7 = x7 - x7_ref;
e8 = x8;
e9 = x9 - x9_ref;
e10 = x10;
e11 = x11 - x11_ref;
e12 = x12;
e13 = x13 - x13_ref;
e14 = x14;

% Transformations
z1 = e2 - (-k1 * e1);
z2 = e4 - (-k3 * e3);
z3 = e6 - (-k5 * e5);
z4 = e8 - (-k7 * e7);
z5 = e10 - (-k9 * e9);
z6 = e12 - (-k11 * e11);
z7 = e14 - (-k13 * e13);

% Control inputs
Uf1 = (-m * x12 * x4 + m * x10 * x6 + m * g * sin(x9) - Ty + Nx - m * k2 * z1 - m * e1 - m * k1) / (C_psi * C_phi * S_theta + S_psi * S_phi);
Uf2 = (-m * x12 * x2 + m * x8 * x6 + m * g * C_theta * S_phi - Tx + Ny - m * e3 - m * k4 * z2 - m * k3) / (C_phi * S_psi * S_theta - C_psi * S_phi);
Uf3 = (-m * x10 * x12 + m * x8 * x4 - m * g * C_theta * C_phi - Tz + Nz - m * e5 - m * k6 * z3 - m * k5) / (C_theta * C_phi);
U_phi = Ixx * (-k8 * z4 - x7 - k7) + x10 * x12 * Iyy - x10 * x12 * Izz + N_phi;
U_theta = Iyy * (-k10 * z5 - x9 - k9) - x8 * x12 * Ixx + x8 * x12 * Izz + N_theta; 
U_psi = Izz * (-k12 * z6 - x11 - k11) - x8 * x10 * Ixx + x8 * x10 * Iyy + N_psi; 
U_win = (-Iw * e13 - Iw * k14 * z7 - Iw * k13 - r_w * T0 - r_w^2 * x5 * s * rho * g) / r_w;

% Initialize derivatives
dxdt = zeros(14 + 2*n, 1);

% Position derivatives
dxdt(1) = x2;
dxdt(3) = x4;
dxdt(5) = x6;

% Velocity derivatives
dxdt(2) = (Uf1 * (C_psi * C_phi * S_theta + S_psi * S_phi) + m * x12 * x4 - m * x10 * x6 - m * g * S_theta + Ty - Nx) / m;
dxdt(4) = (Uf2 * (C_phi * S_psi * S_theta - C_psi * S_phi) + m * x12 * x2 - m * x8 * x6 - m * g * C_theta * S_theta + Tx - Ny) / m;
dxdt(6) = (Uf3 * C_theta * C_phi + m * x10 * x2 - m * x8 * x4 + m * g * C_theta * C_phi + Tz - Nz) / m;

% Angular position derivatives
dxdt(7) = x8;
dxdt(9) = x10;
dxdt(11) = x12;

% Angular velocity derivatives
dxdt(8) = (U_phi - x10 * x12 * Iyy + x10 * x12 * Izz - N_phi) / Ixx;
dxdt(10) = (U_theta + x8 * x12 * Ixx - x8 * x12 * Izz - N_theta) / Iyy;
dxdt(12) = (U_psi + x8 * x10 * Ixx - x8 * x10 * Iyy - N_psi) / Izz;

% Winder dynamics 
dxdt(13) = x14;
dxdt(14) = (U_win * r_w / Iw) + (T0 / Iw) * r_w + (r_w^2 / Iw) * x5 * s * rho * g;

% Cable dynamics
dxdt(15:end) = cableDynamics(t, x_cable, n, k_spring, d_damping, m_mass);

end
