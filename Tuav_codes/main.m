% main.m
clear all;
close all;
clc;

%% Parameters for the Quadcopter and Cable
% Quadcopter parameters
m = 2.84; % kg, drone mass
g = 9.81; % m/s^2, gravity
Ixx = 0.5192; 
Iyy = 0.4929; 
Izz = 0.0947; % kg/m^2, moments of inertia
L = 0.405; % m, distance from propeller to com

% Winder parameters
Iw = 0.01; % Moment of inertia of winder
r_w = 0.1; % radius of winder 

% Tether parameters
rho = 0.034; % kg/m^2, density of tether
s = 1.1e-4; % m^2, cross-sectional area of tether
T0 = 5; % N (initial tension)
a = T0 / (rho * g); % Catenary parameter
x_0 = 0; 
alpha_angle = atan(sinh(x_0 / a)); % Tension force angle

% Wind disturbance 
Nx = 0.01; Ny = 0.01; Nz = 0.01; N_phi = 0; N_theta = 0; N_psi = 0;

% Gains (controller parameters)
k1  = 25; k2  = 25;
k3  = 25; k4 = 15;
k5  = 45; k6 = 55;
k7  = 85; k8 = 85; 
k9  = 85; k10 = 95;
k11 = 105; k12 = 95;
k13 = 100; k14 = 100;

% Threshold to avoid division by zero
threshold = 1e-10;

% Maximum tether length constraint
max_tether_length = 30; 

% Cable parameters
n = 10; % Number of masses
k_spring = 100; % Spring constant
d_damping = 0.01; % Damping coefficient
m_mass = 2; % Mass of each segment

% Initial conditions for the cable
x0_cable = zeros(2*n, 1); 
x0_cable(n) = 1; % Small displacement
x0_cable(n + n/2) = 0.1; % Small initial velocity

%% Initial Conditions for the Quadcopter
initial_x = 0;
initial_z = 0;
initial_l = sqrt(2 * a * sinh(initial_x / (2 * a)) + initial_z^2);

x0_quadcopter = [initial_x; 0; 0; 0; initial_z; 0; 0; 0; 0; 0; 0; 0; initial_l ; 0];
x0 = [x0_quadcopter; x0_cable]; 

% Define the waypoints
waypoints = [1, 2, 4; 4, 5, 8; 6, 10, 15; 2, 10, 10];

% Time span for the simulation
tspan = [0 2];

%% Solve the ODE
[t, sol] = ode45(@(t, x) dynamics(t, x, m, g, Ixx, Iyy, Izz, alpha_angle, rho, Iw, r_w, s, a, T0, x_0, Nx, Ny, Nz, N_phi, N_theta, N_psi, ...
    k1, k2, k3, k4, k5, k6, k7, k8, k9, k10, k11, k12, k13, k14, threshold, max_tether_length, waypoints, n, k_spring, d_damping, m_mass), ...
    tspan, x0);

%% Extract Simulation Results
x_pos = sol(:, 1);
y_pos = sol(:, 3);
z_pos = sol(:, 5);
L_tether = sol(:, 13);
cable_positions = sol(:, 15:2:end);
phi_angle = sol(:, 7);
theta_angle = sol(:, 9);
psi_angle = sol(:, 11);

%% Animation Parameters
trail = [];
l = 2; % Length scale for the drone representation

% Initialize the current waypoint
current_waypoint = 1;

%% Initialize VideoWriter (Optional)
% Uncomment the following lines if you wish to save the animation as a video
% videoFilename = 'quadcopter_animation.mp4';
% videoWriter = VideoWriter(videoFilename, 'MPEG-4');
% videoWriter.FrameRate = 30;
% open(videoWriter);

%% Animation Loop
figure;
hold on;
grid on;
xlabel('X Position','FontSize',14);
ylabel('Y Position', 'FontSize',14);
zlabel('Z Position','FontSize',14);
title('Quadcopter Position and Tether over Time', 'FontSize',14);
xlim([min(x_pos)-1, max(x_pos)+1]);
ylim([min(y_pos)-1, max(y_pos)+1]);
zlim([0, max(z_pos)+1]);  % Start z from 0 to always keep ground visible
view(3); % Ensure the plot is in 3D view
set(gcf, 'Color', 'w');
set(gca, 'Color', 'w');
set(gca, 'FontSize', 14);

% Plot desired waypoints with blue asterisks
desired_handle = plot3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'b*', 'MarkerSize', 10, 'LineWidth', 2);

% Plot start point with a red dot
start_handle = plot3(x_pos(1), y_pos(1), z_pos(1), 'r.', 'MarkerSize', 10, 'LineWidth', 2);

drone_handles = [];

for i = 1:length(t)
    % Calculate catenary shape for the tether
    x_tether = linspace(0, x_pos(i), n);
    y_tether = linspace(0, y_pos(i), n);
    z_tether = a * cosh((linspace(0, sqrt(x_pos(i)^2 + y_pos(i)^2), n) / a)) - a;
    % Scale the catenary to match the drone's current height
    scale_factor = z_pos(i) / max(z_tether);
    z_tether = z_tether * scale_factor;

    % Add wiggling effect from cable dynamics
    z_tether = z_tether + cable_positions(i, :);

    % Ensure cable is connected to the origin
    x_tether = [0, x_tether];
    y_tether = [0, y_tether];
    z_tether = [0, z_tether];

    % Plot tether
    tether_handle = plot3(x_tether, y_tether, z_tether, 'k-', 'LineWidth', 1.5); % Increase LineWidth for a thicker cable

    % Plot quadcopter
    if ~isempty(drone_handles)
        delete(drone_handles);
    end
    drone_handles = draw_drone([x_pos(i), y_pos(i), z_pos(i)], [phi_angle(i), theta_angle(i), psi_angle(i)], l);

    % Update trail
    trail = [trail; x_pos(i), y_pos(i), z_pos(i)];
    trail_handle = plot3(trail(:,1), trail(:,2), trail(:,3), 'Color', [1, 0.5, 0.5], 'LineWidth', 0.5);

    % Capture the current frame (Optional for video)
    % frame = getframe(gcf);
    % writeVideo(videoWriter, frame);

    pause(0.03);

    if i < length(t)
        delete(tether_handle);
    end

    % Update current waypoint if close enough
    if norm([x_pos(i) - waypoints(current_waypoint, 1), y_pos(i) - waypoints(current_waypoint, 2), z_pos(i) - waypoints(current_waypoint, 3)]) < 0.1
        current_waypoint = current_waypoint + 1;
        if current_waypoint > size(waypoints, 1)
            current_waypoint = size(waypoints, 1); % Stay at the last waypoint
        end
    end
end

% Add legend
legend([trail_handle, desired_handle, start_handle], {'UAV Trail', 'Desired Locations', 'Start Point'}, 'FontSize', 14);

% Close the video file (Optional)
% close(videoWriter);
