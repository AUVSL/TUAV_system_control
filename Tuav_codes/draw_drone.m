% draw_drone.m
function drone_handles = draw_drone(position, angle, l)
axle_x = [-l/2 0 0; l/2 0 0];
axle_y = [0 -l/2 0; 0 l/2 0];

r = 0.1 * l; % Radius of propellers
ang = linspace(0, 2*pi);
x_circle = r * cos(ang);
y_circle = r * sin(ang);
z_circle = zeros(1, length(ang));
propeller = [x_circle', y_circle', z_circle'];

[p1, ~] = size(propeller);
[p2, ~] = size(axle_x);

% Position and angle
x = position(1);
y = position(2);
z = position(3);
phi = angle(1);
theta = angle(2);
psi = angle(3);

R = get_rotation(phi, theta, psi);

for i = 1:p2
    r_body = axle_x(i, :)';
    r_world = R * r_body;
    new_axle_x(i, :) = r_world';
end
new_axle_x = [x y z] + new_axle_x;

for i = 1:p2
    r_body = axle_y(i, :)';
    r_world = R * r_body;
    new_axle_y(i, :) = r_world';
end
new_axle_y = [x y z] + new_axle_y;

for i = 1:p1
    r_body = propeller(i, :)';
    r_world = R * r_body;
    new_propeller(i, :) = r_world';
end

new_propeller1 = new_axle_x(1, :) + new_propeller;
new_propeller3 = new_axle_x(2, :) + new_propeller;
new_propeller2 = new_axle_y(1, :) + new_propeller;
new_propeller4 = new_axle_y(2, :) + new_propeller;

h1 = line(new_axle_x(:, 1), new_axle_x(:, 2), new_axle_x(:, 3), 'LineWidth', 2); hold on;
h2 = line(new_axle_y(:, 1), new_axle_y(:, 2), new_axle_y(:, 3), 'LineWidth', 2);
h3 = patch(new_propeller1(:, 1), new_propeller1(:, 2), new_propeller1(:, 3), 'r');
h4 = patch(new_propeller2(:, 1), new_propeller2(:, 2), new_propeller2(:, 3), 'g');
h5 = patch(new_propeller3(:, 1), new_propeller3(:, 2), new_propeller3(:, 3), 'b');
h6 = patch(new_propeller4(:, 1), new_propeller4(:, 2), new_propeller4(:, 3), 'c');

drone_handles = [h1, h2, h3, h4, h5, h6];
end
