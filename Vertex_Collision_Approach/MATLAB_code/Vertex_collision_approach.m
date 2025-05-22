% Obstacle parameters
l = 1.2;
w = 0.6;
alpha = -0.9;
beta = 0.9;
phi = 0.820305;

% Midpoints of 2 parallel widths
M_x = alpha - l*cos(phi)/2;
N_x = alpha + l*cos(phi)/2;
M_y = beta - l*sin(phi)/2;
N_y = beta + l*sin(phi)/2;

% Rectangle vertices (rotated)
obstacle.vertices = [
    M_x - w*sin(phi)/2, M_y + w*cos(phi)/2;  % vertex 1
    N_x - w*sin(phi)/2, N_y + w*cos(phi)/2;  % vertex 2
    M_x + w*sin(phi)/2, M_y - w*cos(phi)/2;  % vertex 3
    N_x + w*sin(phi)/2, N_y - w*cos(phi)/2   % vertex 4
];

% Triangle-based rectangle
obstacle.faces = [1 2 4; 1 4 3];

figure;

% Plot rectangle
patch('Faces', obstacle.faces, 'Vertices', obstacle.vertices, ...
      'FaceColor', [255 165 0]/255, 'EdgeColor', [255 165 0]/255);

hold on;

% Plot circle of radius 3 centered at (0,0)
theta = linspace(0, 2*pi, 300);
r = 3;
x_circle = r * cos(theta);
y_circle = r * sin(theta);
plot(x_circle, y_circle, 'k', 'LineWidth', 1.5);  % black circle

axis equal
axis off
xlim([-5 5])
ylim([-5 5])

% ---- Compute angles in radians from origin to each vertex ----
angles_rad = zeros(1, 4);

for i = 1:4
    x_i = obstacle.vertices(i, 1);
    y_i = obstacle.vertices(i, 2);
    angle_standard = atan2(y_i, x_i);
    
    if alpha < 0
        % Measure angle from positive X-axis
        angles_rad(i) = mod(angle_standard, 2*pi);
    else
        % Measure angle from negative Y-axis (anticlockwise)
        angle_shifted = angle_standard + pi/2;
        angles_rad(i) = mod(angle_shifted, 2*pi);
    end
end

% Find min and max angle indices
[theta_min_rad, idx_min] = min(angles_rad);
[theta_max_rad, idx_max] = max(angles_rad);

%PLOTTING FUNCTIONS

% Find unit vectors in directions of those vertices
v_min = obstacle.vertices(idx_min, :) / norm(obstacle.vertices(idx_min, :));
v_max = obstacle.vertices(idx_max, :) / norm(obstacle.vertices(idx_max, :));

% Scale unit vectors to circle radius r = 3
endpoint_min = v_min * r;
endpoint_max = v_max * r;

% Plot radius lines from origin to circle boundary along vertex directions
plot([0 endpoint_min(1)], [0 endpoint_min(2)], 'r--', 'LineWidth', 1.5);
plot([0 endpoint_max(1)], [0 endpoint_max(2)], 'r--', 'LineWidth', 1.5);

% Convert sector bounds back to standard polar angles (from +X axis)
if alpha < 0
    % No adjustment needed
    theta_plot_min = theta_min_rad;
    theta_plot_max = theta_max_rad;
else
    % Angles measured from -Y axis; shift back by -π/2
    theta_plot_min = mod(theta_min_rad - pi/2, 2*pi);
    theta_plot_max = mod(theta_max_rad - pi/2, 2*pi);

    % Ensure correct angle span
    if theta_plot_max < theta_plot_min
        theta_plot_max = theta_plot_max + 2*pi;
    end
end

% Plot the sector enclosed between those radii
num_points_sector = 100;
theta_sector = linspace(theta_plot_min, theta_plot_max, num_points_sector);
x_sector = r * cos(theta_sector);
y_sector = r * sin(theta_sector);
sector_x = [0, x_sector, 0];
sector_y = [0, y_sector, 0];
patch(sector_x, sector_y, [1 0.8 0], 'FaceAlpha', 0.3, 'EdgeColor', 'none');


% ----------- Annotate angles on the figure -----------

% --- Convert angles back to standard polar reference (from +X axis) ---
if alpha < 0
    theta_min_plot = theta_min_rad;
    theta_max_plot = theta_max_rad;
else
    % Angles were measured from -Y axis, shift back to +X axis
    theta_min_plot = mod(theta_min_rad - pi/2, 2*pi);
    theta_max_plot = mod(theta_max_rad - pi/2, 2*pi);

    % Ensure positive angular span
    if theta_max_plot < theta_min_plot
        theta_max_plot = theta_max_plot + 2*pi;
    end
end

% --- Convert to degrees ---
theta_min_deg = rad2deg(theta_min_plot);
theta_max_deg = rad2deg(theta_max_plot);
angle_span_rad = theta_max_plot - theta_min_plot;
angle_span_deg = rad2deg(angle_span_rad);

%  Wrap angles for annotation (ensure in [0, 360) for display only)
theta_min_deg = mod(theta_min_deg, 360);
theta_max_deg = mod(theta_max_deg, 360);


% --- Text labels for theta_min and theta_max ---
text(endpoint_min(1)*1.1, endpoint_min(2)*1.1, ...
     sprintf('\\theta_{min} = %.1f^\\circ', theta_min_deg), ...
     'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'left');

text(endpoint_max(1)*1.1, endpoint_max(2)*1.1, ...
     sprintf('\\theta_{max} = %.1f^\\circ', theta_max_deg), ...
     'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'left');

% --- Label for angle span at center ---
text(0.5, 0.3, ...
     sprintf('\\Delta\\theta = %.1f^\\circ', angle_span_deg), ...
     'Color', 'k', 'FontSize', 11, 'FontWeight', 'bold');

hold off;

% ------- Print angle values to console -------
fprintf('Theta min (from +X): %.2f degrees\n', theta_min_deg);
fprintf('Theta max (from +X): %.2f degrees\n', theta_max_deg);
fprintf('Angle span: %.2f degrees\n', angle_span_deg);

