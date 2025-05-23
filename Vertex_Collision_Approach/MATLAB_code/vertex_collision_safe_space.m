clc; clear; close all;

% Rectangle parameters
l = 1.2;               % Length
w = 0.6;               % Width
alpha = -0.9;          % Center X
beta = 0.9;            % Center Y
phi = 0.820305;        % Inclination (radians)

% Compute midpoints of widths
M_x = alpha - l*cos(phi)/2;
N_x = alpha + l*cos(phi)/2;
M_y = beta - l*sin(phi)/2;
N_y = beta + l*sin(phi)/2;

% Compute vertices
obstacle.vertices = [
    M_x - w*sin(phi)/2, M_y + w*cos(phi)/2;  %v1
    N_x - w*sin(phi)/2, N_y + w*cos(phi)/2;  %v2
    M_x + w*sin(phi)/2, M_y - w*cos(phi)/2;  %v3
    N_x + w*sin(phi)/2, N_y - w*cos(phi)/2   %v4
];

vertices = obstacle.vertices;
save('obstacle_vertices.mat', 'vertices');

obstacle.faces = [1 2 4; 1 4 3];

% Plot
figure;
patch('Faces', obstacle.faces, 'Vertices', obstacle.vertices, ...
      'FaceColor', [255 165 0]/255, 'EdgeColor', [255 165 0]/255);
hold on;

% Circle
r = 3;
theta = linspace(0, 2*pi, 300);
plot(r*cos(theta), r*sin(theta), 'k', 'LineWidth', 1.5);
axis equal off; xlim([-5 5]); ylim([-5 5]);

% ---------- Modified: Identify collision vertices via max angular span ----------
angles_rad = mod(atan2(obstacle.vertices(:,2), obstacle.vertices(:,1)), 2*pi);
max_diff = 0;
idx_min = 0; idx_max = 0;
for i = 1:3
    for j = i+1:4
        diff = abs(angles_rad(i) - angles_rad(j));
        diff = min(diff, 2*pi - diff);  % shorter arc
        if diff > max_diff
            max_diff = diff;
            idx_min = i;
            idx_max = j;
        end
    end
end

% Ensure smallest sector is chosen
a1 = angles_rad(idx_min);
a2 = angles_rad(idx_max);
if mod(a2 - a1, 2*pi) > mod(a1 - a2, 2*pi)
    temp = idx_min;
    idx_min = idx_max;
    idx_max = temp;
end

theta_min_rad = angles_rad(idx_min);
theta_max_rad = angles_rad(idx_max);

% Plot sector boundary rays
v_min = obstacle.vertices(idx_min,:) / norm(obstacle.vertices(idx_min,:));
v_max = obstacle.vertices(idx_max,:) / norm(obstacle.vertices(idx_max,:));
endpoint_min = v_min * r;
endpoint_max = v_max * r;
plot([0 endpoint_min(1)], [0 endpoint_min(2)], 'r--', 'LineWidth', 1.5);
plot([0 endpoint_max(1)], [0 endpoint_max(2)], 'r--', 'LineWidth', 1.5);

% Shade sector
theta_sector = linspace(theta_min_rad, theta_max_rad, 100);
x_sector = r * cos(theta_sector);
y_sector = r * sin(theta_sector);
patch([0 x_sector 0], [0 y_sector 0], [1 0.8 0], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Store vertices to check
vertices_to_check = [idx_min, idx_max];
colors = ['b', 'm'];
endpoints = [];

% Check if a point is inside the sector
is_in_sector = @(x, y) (sqrt(x^2 + y^2) <= r) && ...
    (mod(atan2(y, x) - theta_min_rad, 2*pi) <= mod(theta_max_rad - theta_min_rad, 2*pi));

% Loop over the two key vertices
for k = 1:2
    idx = vertices_to_check(k);
    vx = obstacle.vertices(idx,1);
    vy = obstacle.vertices(idx,2);

    A = vx; B = vy;
    C = (vx^2 + vy^2) / (2*r);
    R = sqrt(A^2 + B^2);
    delta = atan2(B, A);

    found = false;

    if abs(C/R) <= 1
        sol = [delta + acos(C/R), delta - acos(C/R)];
        sol = mod(sol, 2*pi);

        for a = sol
            px = r * cos(a);
            py = r * sin(a);

            % Sample points on the segment
            p1 = [vx, vy];
            p2 = [px, py];
            t_vals = [0.1, 0.25, 0.5, 0.75, 0.9];
            sample_pts = (1 - t_vals') * p1 + t_vals' * p2;

            % Check if all points lie outside the sector
            safe = true;
            for i = 1:size(sample_pts,1)
                if is_in_sector(sample_pts(i,1), sample_pts(i,2))
                    safe = false;
                    break;
                end
            end

            if safe
                plot([vx px], [vy py], [colors(k) '-'], 'LineWidth', 1.5);
                plot(px, py, [colors(k) 'o'], 'MarkerFaceColor', colors(k));
                plot([0 px], [0 py], [colors(k) '-'], 'LineWidth', 1.5); % colored radial line
                plot(px + r*cos(theta), py + r*sin(theta), ':k', 'LineWidth', 1.2); % dotted black circle

                endpoints(k,:) = [px py];

                angle_rad = atan2(py, px);
                
                if k == 1
                    fprintf('For blue   : px = %.4f, py = %.4f, angle = %.4f deg\n', px, py, angle_rad *180/pi);
                else
                    fprintf('For magenta: px = %.4f, py = %.4f, angle = %.4f deg\n', px, py, angle_rad*180/pi);
                end

                found = true;
                break;
            end
        end
    end

    if ~found
        disp(['No valid ray found for vertex ', num2str(idx)]);
    end
end

% Fill green complementary sector
if size(endpoints,1) == 2
    a1 = mod(atan2(endpoints(1,2), endpoints(1,1)), 2*pi);
    a2 = mod(atan2(endpoints(2,2), endpoints(2,1)), 2*pi);
    if a2 < a1, theta_fill = linspace(a2, a1, 300);
    else, theta_fill = linspace(a2, a1 + 2*pi, 300); end
    x_fill = r * cos(theta_fill);
    y_fill = r * sin(theta_fill);
    patch([0 x_fill 0], [0 y_fill 0], [0 1 0], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end

hold off;
