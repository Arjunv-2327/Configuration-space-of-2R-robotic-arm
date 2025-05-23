% Rectangle parameters
l = 1.2;               % Length
w = 0.6;               % Width
alpha = -0.9;          % Center X
beta = 0.9;            % Center Y
phi = 0.820305;        % Inclination (radians)

% Midpoints of 2 parallel widths
M_x = alpha - l*cos(phi)/2;
N_x = alpha + l*cos(phi)/2;
M_y = beta - l*sin(phi)/2;
N_y = beta + l*sin(phi)/2;

% Rectangle vertices (rotated)
obstacle.vertices = [
    M_x - w*sin(phi)/2, M_y + w*cos(phi)/2;
    N_x - w*sin(phi)/2, N_y + w*cos(phi)/2;
    M_x + w*sin(phi)/2, M_y - w*cos(phi)/2;
    N_x + w*sin(phi)/2, N_y - w*cos(phi)/2
];

% Triangle-based rectangle
obstacle.faces = [1 2 4; 1 4 3];

% ---------- Plot Circle and Rectangle ----------
figure;
hold on;
r = 3;
theta = linspace(0, 2*pi, 300);
x_circle = r * cos(theta);
y_circle = r * sin(theta);
plot(x_circle, y_circle, 'k', 'LineWidth', 1.5);  % black circle

patch('Faces', obstacle.faces, 'Vertices', obstacle.vertices, ...
      'FaceColor', [255 165 0]/255, 'EdgeColor', [255 165 0]/255);

axis equal
axis off
xlim([-5 5])
ylim([-5 5])

% ---------- Compute Angles of All Vertices ----------
angles_rad = zeros(1, 4);
for i = 1:4
    x_i = obstacle.vertices(i, 1);
    y_i = obstacle.vertices(i, 2);
    angles_rad(i) = mod(atan2(y_i, x_i), 2*pi);
end

% ---------- Identify Pair with Max Angular Separation ----------
max_diff = 0;
idx1 = 0;
idx2 = 0;
for i = 1:3
    for j = i+1:4
        diff = abs(angles_rad(i) - angles_rad(j));
        diff = min(diff, 2*pi - diff);  % shortest arc
        if diff > max_diff
            max_diff = diff;
            idx1 = i;
            idx2 = j;
        end
    end
end

% ---------- Ensure Smallest Sector is Shaded ----------
a1 = angles_rad(idx1);
a2 = angles_rad(idx2);
a1 = mod(a1, 2*pi);
a2 = mod(a2, 2*pi);

% Make sure the smaller sector is selected
if mod(a2 - a1, 2*pi) <= mod(a1 - a2, 2*pi)
    theta_start = a1;
    theta_end = a2;
else
    theta_start = a2;
    theta_end = a1;
end

% ---------- Draw Radius Lines ----------
v1 = obstacle.vertices(idx1, :) / norm(obstacle.vertices(idx1, :));
v2 = obstacle.vertices(idx2, :) / norm(obstacle.vertices(idx2, :));
end1 = v1 * r;
end2 = v2 * r;
plot([0 end1(1)], [0 end1(2)], 'r--', 'LineWidth', 1.5);
plot([0 end2(1)], [0 end2(2)], 'r--', 'LineWidth', 1.5);

% ---------- Shade the Sector ----------
num_points_sector = 100;
% Ensure shortest arc is shaded
if theta_end < theta_start
    theta_end = theta_end + 2*pi;
end
theta_sector = linspace(theta_start, theta_end, num_points_sector);
x_sector = r * cos(theta_sector);
y_sector = r * sin(theta_sector);
sector_x = [0, x_sector, 0];
sector_y = [0, y_sector, 0];
patch(sector_x, sector_y, [1, 0.5, 0], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% ---------- Annotate Angles ----------
theta_min_plot = theta_start;
theta_max_plot = theta_end;
if theta_max_plot < theta_min_plot
    theta_max_plot = theta_max_plot + 2*pi;
end

theta_min_deg = mod(rad2deg(theta_min_plot), 360);
theta_max_deg = mod(rad2deg(theta_max_plot), 360);
angle_span_deg = theta_max_deg - theta_min_deg;
if angle_span_deg < 0
    angle_span_deg = angle_span_deg + 360;
end
if angle_span_deg > 360
    angle_span_deg = 360;
end

text(end1(1)*1.1, end1(2)*1.1, ...
     sprintf('\\theta_{min} = %.1f^\\circ', theta_min_deg), ...
     'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'left');

text(end2(1)*1.1, end2(2)*1.1, ...
     sprintf('\\theta_{max} = %.1f^\\circ', theta_max_deg), ...
     'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'left');

text(0.5, 0.3, ...
     sprintf('\\Delta\\theta = %.1f^\\circ', angle_span_deg), ...
     'Color', 'k', 'FontSize', 11, 'FontWeight', 'bold');

hold off;

% ---------- Console Output ----------
fprintf('Theta min (from +X): %.2f degrees\n', theta_min_deg);
fprintf('Theta max (from +X): %.2f degrees\n', theta_max_deg);
fprintf('Angle span: %.2f degrees\n', angle_span_deg);
