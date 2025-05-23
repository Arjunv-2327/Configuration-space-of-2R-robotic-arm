% Rectangle parameters
l = 1.2;               % Length
w = 0.6;               % Width
alpha = -0.9;          % Center X
beta = 0.9;            % Center Y
phi = 0.820305;        % Inclination

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

% ---------- Compute Distances from Origin ----------
distances = sqrt(sum(obstacle.vertices.^2, 2));
inside_idx = find(distances <= r);
outside_idx = find(distances > r);

% Helper to compute angle normalized
normalize_angle = @(x, y) mod(atan2(y, x), 2*pi);

% Case 1, 2: Fully or mostly inside circle
if length(outside_idx) <= 1
    angles_rad = arrayfun(@(i) normalize_angle(obstacle.vertices(i,1), obstacle.vertices(i,2)), 1:4);
    max_diff = 0;
    idx1 = 0; idx2 = 0;
    for i = 1:3
        for j = i+1:4
            diff = abs(angles_rad(i) - angles_rad(j));
            diff = min(diff, 2*pi - diff);
            if diff > max_diff
                max_diff = diff;
                idx1 = i;
                idx2 = j;
            end
        end
    end
    points = [obstacle.vertices(idx1,:); obstacle.vertices(idx2,:)];
else
    % Case 3, 4, 5: Rectangle intersects circle
    lines = [1 2; 1 3; 2 4; 3 4];
    intersection_pts = [];
    for k = 1:size(lines,1)
        P1 = obstacle.vertices(lines(k,1),:);
        P2 = obstacle.vertices(lines(k,2),:);
        d = P2 - P1;
        f = P1;

        a = dot(d,d);
        b = 2 * dot(f,d);
        c = dot(f,f) - r^2;
        discriminant = b^2 - 4*a*c;

        if discriminant >= 0
            discriminant = sqrt(discriminant);
            t1 = (-b - discriminant) / (2*a);
            t2 = (-b + discriminant) / (2*a);

            if t1 >= 0 && t1 <= 1
                intersection_pts(end+1,:) = P1 + t1*d;
            end
            if t2 >= 0 && t2 <= 1
                intersection_pts(end+1,:) = P1 + t2*d;
            end
        end
    end

    all_pts = [obstacle.vertices(inside_idx,:); intersection_pts];
    angles_rad = arrayfun(@(i) normalize_angle(all_pts(i,1), all_pts(i,2)), 1:size(all_pts,1));
    max_diff = 0;
    idx1 = 0; idx2 = 0;
    for i = 1:length(angles_rad)-1
        for j = i+1:length(angles_rad)
            diff = abs(angles_rad(i) - angles_rad(j));
            diff = min(diff, 2*pi - diff);
            if diff > max_diff
                max_diff = diff;
                idx1 = i;
                idx2 = j;
            end
        end
    end
    points = [all_pts(idx1,:); all_pts(idx2,:)];
end

% ---------- Final Plot Logic ----------
theta_start = normalize_angle(points(1,1), points(1,2));
theta_end = normalize_angle(points(2,1), points(2,2));
if mod(theta_end - theta_start, 2*pi) > pi
    tmp = theta_start; theta_start = theta_end; theta_end = tmp;
end
if theta_end < theta_start
    theta_end = theta_end + 2*pi;
end

v1 = points(1,:) / norm(points(1,:));
v2 = points(2,:) / norm(points(2,:));
end1 = v1 * r;
end2 = v2 * r;

plot([0 end1(1)], [0 end1(2)], 'r--', 'LineWidth', 1.5);
plot([0 end2(1)], [0 end2(2)], 'r--', 'LineWidth', 1.5);

theta_sector = linspace(theta_start, theta_end, 100);
x_sector = r * cos(theta_sector);
y_sector = r * sin(theta_sector);
patch([0 x_sector 0], [0 y_sector 0], [1 0.5 0], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% ---------- Annotation ----------
theta_min_deg = mod(rad2deg(theta_start), 360);
theta_max_deg = mod(rad2deg(theta_end), 360);
angle_span_deg = theta_max_deg - theta_min_deg;
if angle_span_deg < 0
    angle_span_deg = angle_span_deg + 360;
end
if angle_span_deg > 360
    angle_span_deg = 360;
end

%text(end1(1)*1.1, end1(2)*1.1, ...
     %sprintf('\\theta_{min} = %.1f^\\circ', theta_min_deg), ...
    % 'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'left');

%text(end2(1)*1.1, end2(2)*1.1, ...
     %sprintf('\\theta_{max} = %.1f^\\circ', theta_max_deg), ...
     %'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'left');

%text(0.5, 0.3, ...
     %sprintf('\\Delta\\theta = %.1f^\\circ', angle_span_deg), ...
     %'Color', 'k', 'FontSize', 11, 'FontWeight', 'bold');

hold off;

% ---------- Console Output ----------
fprintf('Min: %.2f degrees\n', theta_min_deg);
fprintf('Max: %.2f degrees\n', theta_max_deg);
fprintf('Span: %.2f degrees\n', angle_span_deg);
