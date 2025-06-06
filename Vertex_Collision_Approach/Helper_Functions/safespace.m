function [angle1_deg, angle2_deg] = safespace(r, d, obstacle)
    figure;
    hold on;
    axis equal off;
    axis([-r-1 r+1 -r-1 r+1]);
    angle1_deg = NaN;
    angle2_deg = NaN;

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
    theta_min = min(theta1, theta2);
    theta_max = max(theta1, theta2);

    % Plot red dashed radii
    plot([0 r*cos(theta1)], [0 r*sin(theta1)], 'r--', 'LineWidth', 1.5);
    plot([0 r*cos(theta2)], [0 r*sin(theta2)], 'r--', 'LineWidth', 1.5);

    is_in_sector = @(x, y, th1, th2) ...
        (sqrt(x^2 + y^2) <= r) && ...
        (mod(atan2(y,x) - th1, 2*pi) <= mod(th2 - th1, 2*pi));

    V = obstacle.vertices;
    count_endpoints = 0;
    endpoints = zeros(2,2);

    % ----------- LOOP OVER 4 EDGES -----------
    for j = 1:4
        Va = V(j,:);
        Vb = V(mod(j, size(V,1)) + 1, :);

        % Skip if vertical line to avoid division by zero in slope
        if abs(Vb(1) - Va(1)) < 1e-6
            continue;
        end

        m = (Vb(2) - Va(2)) / (Vb(1) - Va(1));
        c = Va(2) - m * Va(1);

        eqs = @(vars) [
            vars(2) - m*vars(1) - c;
            vars(1) - (r*cos(vars(3)) - m*c + m*r*sin(vars(3))) / (1 + m^2);
            (vars(1) - r*cos(vars(3)))^2 + (vars(2) - r*sin(vars(3)))^2 - d^2
        ];
        guesses = [0 1; 0 1; 0 pi/2];
        opts = optimoptions('fsolve','Display','off');

        for g = 1:2
            guess = guesses(:,g);
            [sol, ~, exitflag] = fsolve(eqs, guess, opts);
            if exitflag <= 0
                continue;
            end
            x = sol(1); y = sol(2); theta_val = sol(3);

            % YOUR CHECK: if x outside segment bounds then run vertex-based method
            if x < min(Va(1), Vb(1)) || x > max(Va(1), Vb(1))
                % Run vertex based method for all vertices (A,B)
                for k = 1:4
                    vx = V(k,1);
                    vy = V(k,2);

                    A = vx; B = vy;
                    C = (vx^2 + vy^2 + r^2 - d^2)/(2*r);
                    R = sqrt(A^2 + B^2);
                    delta = atan2(B,A);

                    if abs(C/R) <= 1
                        sol_angles = mod([delta + acos(C/R), delta - acos(C/R)], 2*pi);
                        for a = sol_angles
                            px = r*cos(a);
                            py = r*sin(a);
                            t_vals = linspace(0.01, 0.99, 100);
                            pts = (1 - t_vals') * [vx vy] + t_vals' * [px py];
                            safe = true;
                            for i_t = 1:length(t_vals)
                                if is_in_sector(pts(i_t,1), pts(i_t,2), theta_min, theta_max)
                                    safe = false; 
                                    break;
                                end
                            end
                            if safe
                                theta_full = linspace(0, 2*pi, 10000);
                                cx = px + d*cos(theta_full);
                                cy = py + d*sin(theta_full);
                                in = inpolygon(cx, cy, V(:, 1), V(:, 2));
                                if sum(in) <= 10
                                    plot(cx, cy, 'b:', 'LineWidth', 2)
                                    plot([vx px], [vy py], 'g-', 'LineWidth', 4);
                                    plot([0 px], [0 py], 'k-', 'LineWidth', 4);
                                    pt = [px py];
                                    count_endpoints = count_endpoints + 1;
                                    endpoints(count_endpoints,:) = pt;
                                    if count_endpoints == 2
                                        return;
                                    end
                                end
                            end
                        end
                    end
                end

            else
                % Else run perpendicular method exactly as you wrote
                pt = [r * cos(theta_val), r * sin(theta_val)];
                t = linspace(0, 1, 10000);
                seg_x = pt(1) + t * (x - pt(1));
                seg_y = pt(2) + t * (y - pt(2));
                seg_x2 = t * pt(1);
                seg_y2 = t * pt(2);
                cx = pt(1) + d*cos(theta);
                cy = pt(2) + d*sin(theta);
                in_seg = inpolygon(seg_x, seg_y, V(:,1), V(:,2));
                in_seg2 = inpolygon(seg_x2, seg_y2, V(:,1), V(:,2));
                in3 = inpolygon(cx, cy, V(:, 1), V(:, 2));
                if sum(in_seg) <= 10 && sum(in_seg2) <= 10 && sum(in3) <= 10
                    plot([pt(1) x], [pt(2) y], 'g-', 'LineWidth', 4);
                    plot([0 pt(1)], [0 pt(2)], 'k-', 'LineWidth', 4);
                    count_endpoints = count_endpoints + 1;
                    endpoints(count_endpoints,:) = pt;
                    if count_endpoints == 2
                        return;
                    end
                end
            end
        end
        if count_endpoints == 2
            return;
        end
    end

    angle1_deg = mod(rad2deg(atan2(endpoints(1,2), endpoints(1,1))), 360);
    angle2_deg = mod(rad2deg(atan2(endpoints(2,2), endpoints(2,1))), 360);

    fprintf('Link1 angle 1: %.2f degrees\n', angle1_deg);
    fprintf('Link1 angle 2: %.2f degrees\n', angle2_deg);
end
