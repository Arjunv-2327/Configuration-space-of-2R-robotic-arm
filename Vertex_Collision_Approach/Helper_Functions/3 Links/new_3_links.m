function [angle_degs, Sol] = new_3_links(l1, l2, l3, obstacle)
V = obstacle.vertices;
F = obstacle.faces;
R = l2 + l3; % Total radius for new blue circles

Sol = [];
angle_degs = [];

x_vertices = V(:,1);
y_vertices = V(:,2);

figure;
hold on;
axis equal;
axis off;

patch('Vertices', V, 'Faces', F, 'FaceColor', [1 0.5 0], 'EdgeColor', 'k', 'LineWidth', 1.5);
plot(0, 0, 'bo', 'MarkerFaceColor', 'b');  % Origin

theta_base = linspace(0, 2*pi, 1000)';
base_circle = l1 * [cos(theta_base), sin(theta_base)];
plot(base_circle(:,1), base_circle(:,2), 'm--', 'LineWidth', 1);

any_solution_found = false;

for i = 1:4
    alpha = V(i, 1);
    beta = V(i, 2);

    k_val = alpha^2 + beta^2 + l1^2 - R^2;
    A = 4 * (alpha^2 + beta^2);
    B = -4 * k_val * alpha;
    C = k_val^2 - 4 * beta^2 * l1^2;
    D = B^2 - 4 * A * C;

    circle_found = false;

    if D >= 0
        sqrtD = sqrt(D);
        x1 = (-B + sqrtD) / (2 * A);
        y1 = (k_val - 2 * x1 * alpha) / (2 * beta);
        x2 = (-B - sqrtD) / (2 * A);
        y2 = (k_val - 2 * x2 * alpha) / (2 * beta);

        P1 = [x1 y1];
        P2 = [x2 y2];
        Vx = [alpha, beta];

        for P = [P1; P2]'
            pt = P';
            t = linspace(0,1,10000)';
            seg1 = (1 - t) * Vx + t * pt;
            inside_seg = inpolygon(seg1(:,1), seg1(:,2), x_vertices, y_vertices);

            theta_circ = linspace(0, 2*pi, 10000)';
            circ_pts = R * [cos(theta_circ), sin(theta_circ)] + pt;
            inside_circ = inpolygon(circ_pts(:,1), circ_pts(:,2), x_vertices, y_vertices);

            seg_origin = (1 - t) * [0,0] + t * pt;
            inside_origin = inpolygon(seg_origin(:,1), seg_origin(:,2), x_vertices, y_vertices);

            if ~any(inside_circ) && ~any(inside_origin)
                % Plot links
                plot([0, pt(1)], [0, pt(2)], 'black', 'LineWidth', 2);
                plot([Vx(1), pt(1)], [Vx(2), pt(2)], 'g', 'LineWidth', 2);
                plot(Vx(1), Vx(2), 'ro', 'MarkerFaceColor', 'r');    % Joint on obstacle
                plot(pt(1), pt(2), 'ko', 'MarkerFaceColor', 'k');    % End-effector

                % Remove old blue dotted circle here (do nothing)

                % Store solution point (base circle point)
                theta_l1 = atan2d(pt(2), pt(1));
                angle_degs(end+1) = mod(theta_l1, 360);
                Sol(end+1,:) = pt;
                circle_found = true;
                any_solution_found = true;
            end
        end
    end

    % Fallback using edges
    if ~circle_found
        for j = 1:4
            Va = V(j,:);
            Vb = V(mod(j,size(V,1))+1,:);

            if abs(Vb(1) - Va(1)) < 1e-6
                continue;
            end

            m = (Vb(2) - Va(2)) / (Vb(1) - Va(1));
            c = Va(2) - m * Va(1);

            eqs = @(vars) [
                vars(2) - m*vars(1) - c;
                vars(1) - (l1*cos(vars(3)) - m*c + m*l1*sin(vars(3))) / (1 + m^2);
                (vars(1) - l1*cos(vars(3)))^2 + (vars(2) - l1*sin(vars(3)))^2 - R^2
            ];

            guess = [0; 0; 0];
            opts = optimoptions('fsolve','Display','off');
            [sol, ~, exitflag] = fsolve(eqs, guess, opts);

            if exitflag <= 0
                continue;
            end

            x = sol(1); y = sol(2); theta = sol(3);

            if x < min(Va(1), Vb(1)) || x > max(Va(1), Vb(1))
                continue;
            end

            pt = [l1 * cos(theta), l1 * sin(theta)];
            obstacle_pt = [x, y];

            t = linspace(0, 1, 10000)';
            seg = (1 - t) * obstacle_pt + t * pt;
            seg_origin = (1 - t) * [0,0] + t * pt;

            inside_seg = inpolygon(seg(:,1), seg(:,2), x_vertices, y_vertices);
            inside_origin = inpolygon(seg_origin(:,1), seg_origin(:,2), x_vertices, y_vertices);
            circ_pts = R * [cos(theta_base), sin(theta_base)] + pt;
            inside_circ = inpolygon(circ_pts(:,1), circ_pts(:,2), x_vertices, y_vertices);

            if ~any(inside_seg) && ~any(inside_origin) && ~any(inside_circ)
                plot([0, pt(1)], [0, pt(2)], 'black', 'LineWidth', 2);
                plot([x, pt(1)], [y, pt(2)], 'g', 'LineWidth', 2);
                plot(x, y, 'ro', 'MarkerFaceColor', 'r');
                plot(pt(1), pt(2), 'ko', 'MarkerFaceColor', 'k');

                % Remove old blue dotted circle here (do nothing)

                theta_l1 = atan2d(pt(2), pt(1));
                angle_degs(end+1) = mod(theta_l1, 360);
                Sol(end+1,:) = pt;
                any_solution_found = true;
                break;
            end
        end
    end
end

% Now plot blue dotted circles of radius (l2+l3) centered at all base circle points in Sol:
theta_circle = linspace(0, 2*pi, 500)';
for k = 1:size(Sol,1)
    center = Sol(k,:);
    circle_pts = center + R * [cos(theta_circle), sin(theta_circle)];
    plot(circle_pts(:,1), circle_pts(:,2), 'b:', 'LineWidth', 1.2);
end

if ~any_solution_found
    disp('No safe region for entire robot.');
end
end
