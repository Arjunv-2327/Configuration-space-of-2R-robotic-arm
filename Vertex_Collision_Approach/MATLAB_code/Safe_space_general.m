% Rectangle parameters
l = 1.2;               % Length
w = 0.6;               % Width
alpha = -0.9;          % Center X
beta = 0.9;            % Center Y
phi = 0.820305;        % Inclination
r = 3;

% Draw Circle
theta = linspace(0, 2*pi, 300);
plot(r * cos(theta), r * sin(theta), 'k', 'LineWidth', 1.5);
hold on;

% Compute Rectangle Vertices
M_x = alpha - l*cos(phi)/2;
N_x = alpha + l*cos(phi)/2;
M_y = beta - l*sin(phi)/2;
N_y = beta + l*sin(phi)/2;
obstacle.vertices = [
    M_x - w*sin(phi)/2, M_y + w*cos(phi)/2;
    N_x - w*sin(phi)/2, N_y + w*cos(phi)/2;
    M_x + w*sin(phi)/2, M_y - w*cos(phi)/2;
    N_x + w*sin(phi)/2, N_y - w*cos(phi)/2
];
obstacle.faces = [1 2 4; 1 4 3];

patch('Faces', obstacle.faces, 'Vertices', obstacle.vertices, ...
      'FaceColor', [255 165 0]/255, 'EdgeColor', [255 165 0]/255);

axis equal off
xlim([-5 5]); ylim([-5 5]);

% ---------- ANGULAR UTILS ----------
angle_wrap = @(a) mod(a, 2*pi);

% ---------- DISTANCES ----------
dists = sqrt(sum(obstacle.vertices.^2, 2));
inside = dists < r;
outside = dists > r;

% ---------- INTERSECTION FUNCTION ----------
function P = circle_line_intersect(p1, p2, r)
    dp = p2 - p1;
    a = dp(1)^2 + dp(2)^2;
    b = 2 * (dp(1) * p1(1) + dp(2) * p1(2));
    c = p1(1)^2 + p1(2)^2 - r^2;
    disc = b^2 - 4*a*c;
    P = [];
    if disc >= 0
        sqrt_disc = sqrt(disc);
        t1 = (-b + sqrt_disc) / (2*a);
        t2 = (-b - sqrt_disc) / (2*a);
        for t = [t1 t2]
            if t >= 0 && t <= 1
                P = [P; p1 + t * dp];
            end
        end
    end
end

% ---------- DETERMINE COLLISION POINTS ----------
points = [];
if all(inside)
    points = obstacle.vertices;
elseif sum(inside) == 3
    idx_out = find(outside);   % Only one
    idx_in = find(inside);     % Three
    for i = 1:length(idx_in)
        p1 = obstacle.vertices(idx_out, :);
        p2 = obstacle.vertices(idx_in(i), :);
        pts = circle_line_intersect(p1, p2, r);
        points = [points; pts];
    end
    points = [points; obstacle.vertices(inside,:)];
elseif sum(inside) == 2
    idx_out = find(outside);
    idx_in = find(inside);
    for i = 1:2
        for j = 1:2
            p1 = obstacle.vertices(idx_out(i), :);
            p2 = obstacle.vertices(idx_in(j), :);
            pts = circle_line_intersect(p1, p2, r);
            points = [points; pts];
        end
    end
    points = [points; obstacle.vertices(inside,:)];
elseif sum(inside) == 1
    idx_in = find(inside);
    idx_out = find(outside);
    for i = 1:3
        p1 = obstacle.vertices(idx_out(i), :);
        p2 = obstacle.vertices(idx_in, :);
        pts = circle_line_intersect(p1, p2, r);
        points = [points; pts];
    end
    points = [points; obstacle.vertices(idx_in,:)];
elseif all(outside)
    for i = 1:4
        j = mod(i,4)+1;
        p1 = obstacle.vertices(i,:);
        p2 = obstacle.vertices(j,:);
        pts = circle_line_intersect(p1, p2, r);
        points = [points; pts];
    end
end

% ---------- FIND MAX ANGULAR SPAN ----------
angles = atan2(points(:,2), points(:,1));
n = length(angles);
max_span = 0;
for i = 1:n
    for j = i+1:n
        a1 = angle_wrap(angles(i));
        a2 = angle_wrap(angles(j));
        span = mod(a2 - a1, 2*pi);
        if span > pi
            span = 2*pi - span;
        end
        if span > max_span
            max_span = span;
            idx1 = i;
            idx2 = j;
        end
    end
end

% ---------- DRAW COLLISION RADII ----------
p1 = points(idx1, :);
p2 = points(idx2, :);
theta1 = atan2(p1(2), p1(1));
theta2 = atan2(p2(2), p2(1));
plot([0 r*cos(theta1)], [0 r*sin(theta1)], 'r--', 'LineWidth', 1.5);
plot([0 r*cos(theta2)], [0 r*sin(theta2)], 'r--', 'LineWidth', 1.5);

% ---------- EXTEND TO CIRCLE EDGE ----------
colors = ['b', 'm'];
endpoints = zeros(2,2);
vertices_to_check = [idx1, idx2];

is_in_sector = @(x, y, th1, th2) (sqrt(x^2 + y^2) <= r) && ...
    (mod(atan2(y, x) - th1, 2*pi) <= mod(th2 - th1, 2*pi));

theta_min = angle_wrap(atan2(p1(2), p1(1)));
theta_max = angle_wrap(atan2(p2(2), p2(1)));

% Ensure theta_min < theta_max
if mod(theta_max - theta_min, 2*pi) > pi
    temp = theta_min;
    theta_min = theta_max;
    theta_max = temp;
end

for k = 1:2
    vx = points(vertices_to_check(k),1);
    vy = points(vertices_to_check(k),2);
    A = vx; B = vy;
    C = (vx^2 + vy^2) / (2*r);
    R = sqrt(A^2 + B^2);
    delta = atan2(B, A);
    found = false;
    if abs(C/R) <= 1
        sol = mod([delta + acos(C/R), delta - acos(C/R)], 2*pi);
        for a = sol
            px = r * cos(a);
            py = r * sin(a);
            % Check sample points
            t_vals = [0.1, 0.25, 0.5, 0.75, 0.9];
            pts = (1 - t_vals') * [vx vy] + t_vals' * [px py];
            safe = true;
            for i = 1:length(t_vals)
                if is_in_sector(pts(i,1), pts(i,2), theta_min, theta_max)
                    safe = false; break;
                end
            end
            if safe
                plot([vx px], [vy py], [colors(k) '-'], 'LineWidth', 1.5);
                plot([0 px], [0 py], [colors(k) '-'], 'LineWidth', 1.5);
                endpoints(k,:) = [px py];
                break;
            end
        end
    end
end

% ---------- PRINT ANGLES OF CIRCUMFERENCE POINTS (IN DEGREES) ----------
for k = 1:2
    px = endpoints(k,1);
    py = endpoints(k,2);
    angle_deg = mod(rad2deg(atan2(py, px)), 360);  % Ensure angle in [0, 360)
    if k == 1
        fprintf('Blue    - %.2f degrees\n', angle_deg);
    else
        fprintf('Magenta - %.2f degrees\n', angle_deg);
    end
end



