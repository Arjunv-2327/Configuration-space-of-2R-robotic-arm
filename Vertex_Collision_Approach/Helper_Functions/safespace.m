function [blue_angle, magenta_angle] = safespace(r, obstacle)
    figure;
    hold on;
    axis equal off;
    axis([-r-1 r+1 -r-1 r+1]);

    % Draw circle
    theta = linspace(0, 2*pi, 300);
    plot(r * cos(theta), r * sin(theta), 'k', 'LineWidth', 1.5);

    % Draw obstacle as solid orange rectangle
    fill(obstacle.vertices([1 2 4 3 1],1), obstacle.vertices([1 2 4 3 1],2), ...
         [1 0.6 0], 'EdgeColor', 'k', 'LineWidth', 1.5);
    % ---------- ANGLE UTIL ----------
    angle_wrap = @(a) mod(a, 2*pi);

    % ---------- DETERMINE COLLISION POINTS ----------
    dists = sqrt(sum(obstacle.vertices.^2, 2));
    inside = dists < r;
    outside = dists > r;
    points = [];

    function P = circle_line_intersect(p1, p2)
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

    if all(inside)
        points = obstacle.vertices;
    elseif sum(inside) == 3
        idx_out = find(outside);
        idx_in = find(inside);
        for i = 1:length(idx_in)
            pts = circle_line_intersect(obstacle.vertices(idx_out,:), obstacle.vertices(idx_in(i),:));
            points = [points; pts];
        end
        points = [points; obstacle.vertices(inside,:)];
    elseif sum(inside) == 2
        idx_out = find(outside);
        idx_in = find(inside);
        for i = 1:2
            for j = 1:2
                pts = circle_line_intersect(obstacle.vertices(idx_out(i),:), obstacle.vertices(idx_in(j),:));
                points = [points; pts];
            end
        end
        points = [points; obstacle.vertices(inside,:)];
    elseif sum(inside) == 1
        idx_in = find(inside);
        idx_out = find(outside);
        for i = 1:3
            pts = circle_line_intersect(obstacle.vertices(idx_out(i),:), obstacle.vertices(idx_in,:));
            points = [points; pts];
        end
        points = [points; obstacle.vertices(idx_in,:)];
    else
        for i = 1:4
            j = mod(i,4)+1;
            pts = circle_line_intersect(obstacle.vertices(i,:), obstacle.vertices(j,:));
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
                idx1 = i; idx2 = j;
            end
        end
    end

    p1 = points(idx1, :);
    p2 = points(idx2, :);
    theta1 = angle_wrap(atan2(p1(2), p1(1)));
    theta2 = angle_wrap(atan2(p2(2), p2(1)));

    % Plot red dashed radii
    plot([0 r*cos(theta1)], [0 r*sin(theta1)], 'r--', 'LineWidth', 1.5);
    plot([0 r*cos(theta2)], [0 r*sin(theta2)], 'r--', 'LineWidth', 1.5);

    % ---------- EXTEND TO CIRCLE ----------
    colors = ['b', 'm'];
    endpoints = zeros(2,2);
    from_pts = [p1; p2];
    theta_min = min(theta1, theta2);
    theta_max = max(theta1, theta2);

    is_in_sector = @(x, y, th1, th2) ...
        (sqrt(x^2 + y^2) <= r) && ...
        (mod(atan2(y,x) - th1, 2*pi) <= mod(th2 - th1, 2*pi));

    for k = 1:2
        vx = from_pts(k,1); vy = from_pts(k,2);
        A = vx; B = vy;
        C = (vx^2 + vy^2)/(2*r);
        R = sqrt(A^2 + B^2);
        delta = atan2(B,A);
        if abs(C/R) <= 1
            sol = mod([delta + acos(C/R), delta - acos(C/R)], 2*pi);
            for a = sol
                px = r*cos(a); py = r*sin(a);
                t_vals = [0.1 0.25 0.5 0.75 0.9];
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

    % ---------- ANGLE OUTPUT ----------
    blue_angle = mod(rad2deg(atan2(endpoints(1,2), endpoints(1,1))), 360);
    magenta_angle = mod(rad2deg(atan2(endpoints(2,2), endpoints(2,1))), 360);

    fprintf('Blue    - %.2f degrees\n', blue_angle);
    fprintf('Magenta - %.2f degrees\n', magenta_angle);
end
