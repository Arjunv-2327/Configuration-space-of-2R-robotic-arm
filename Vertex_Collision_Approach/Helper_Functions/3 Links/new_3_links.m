function [angle_degs, Sol] = new_3_links(l1, l2, l3, obstacle)
    V = obstacle.vertices;
    F = obstacle.faces;
    R = l2 + l3;

    Sol = [];
    angle_degs = [];

    x_vertices = V(:,1);
    y_vertices = V(:,2);

    figure;
    hold on;
    axis equal;
    axis off;

    patch('Vertices', V, 'Faces', F, 'FaceColor', [1 0.4 0], 'EdgeColor', 'k', 'LineWidth', 1.5);
    plot(0, 0, 'ko', 'MarkerFaceColor', 'red', 'MarkerSize', 10);

    theta_base = linspace(0, 2*pi, 10000)';
    base_circle = l1 * [cos(theta_base), sin(theta_base)];
    plot(base_circle(:,1), base_circle(:,2), 'black--', 'LineWidth', 1);

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
            sqrtD = D^0.5;
            x1 = (-B + sqrtD) / (2 * A);
            y1 = (k_val - 2 * x1 * alpha) / (2 * beta);
            x2 = (-B - sqrtD) / (2 * A);
            y2 = (k_val - 2 * x2 * alpha) / (2 * beta);

            for P = [x1 y1; x2 y2]'
                pt = P';  % pt points are the valid solution points on the base circle
                Vx = [alpha, beta];  % Vx represent an instant vertex of obstacle (looped across all 4 vertices)

                dir_vec = pt - Vx;
                if norm(dir_vec) == 0
                    continue;
                end
                unit_vec = dir_vec / norm(dir_vec);
                joint = Vx + l3 * unit_vec;

                % All three links
                t = linspace(0,1,10000)';
                seg1 = (1 - t) * [0,0] + t * pt;
                seg2 = (1 - t) * pt + t * joint;
                seg3 = (1 - t) * joint + t * Vx;

                inside1 = sum(inpolygon(seg1(:,1), seg1(:,2), x_vertices, y_vertices));
                inside2 = sum(inpolygon(seg2(:,1), seg2(:,2), x_vertices, y_vertices));
                inside3 = sum(inpolygon(seg3(:,1), seg3(:,2), x_vertices, y_vertices));

                theta_circ = linspace(0, 2*pi, 10000)';
                blue_circle = R * [cos(theta_circ), sin(theta_circ)] + pt;
                pink_circle = l3 * [cos(theta_circ), sin(theta_circ)] + joint;

                [in_blue, on_blue] = inpolygon(blue_circle(:,1), blue_circle(:,2), x_vertices, y_vertices);
                inside_blue = sum(in_blue & ~on_blue);

                [in_pink, on_pink] = inpolygon(pink_circle(:,1), pink_circle(:,2), x_vertices, y_vertices);
                inside_pink = sum(in_pink & ~on_pink);


                if inside1 <= 10 && inside2 <= 10 && inside3 <= 10 && inside_blue <= 10 && inside_pink <= 10
                    plot([0, pt(1)], [0, pt(2)], 'black', 'LineWidth', 4);     % Link 1 (L1)
                    midpoint = Vx + (l3/R)*dir_vec;   % this is not mathematically midpoint, its the location of link_2_3_joint
                    plot([Vx(1), midpoint(1)], [Vx(2), midpoint(2)], 'm', 'LineWidth', 4);  % Link 3 (L3)
                    plot([midpoint(1), pt(1)], [midpoint(2), pt(2)], 'g', 'LineWidth', 4);  % Link 2 (L2)

                    plot(pt(1), pt(2), 'ko', 'MarkerFaceColor', 'k');     % Elbow joint
                    plot(Vx(1), Vx(2), 'ro', 'MarkerFaceColor', 'r');     % Obstacle vertex
                    plot(joint(1), joint(2), 'ko', 'MarkerFaceColor', 'k'); % link_2_3_joint

                    plot(blue_circle(:,1), blue_circle(:,2), 'b:', 'LineWidth', 1.2); % Blue circle
                    plot(pink_circle(:,1), pink_circle(:,2), 'm:', 'LineWidth', 1.2); % Pink circle

                    angle_degs(end+1) = mod(atan2d(pt(2), pt(1)), 360);
                    Sol(end+1,:) = pt;
                    circle_found = true;
                    any_solution_found = true;
                end
            end
        end

        if ~circle_found && D >= 0
            for j = 1:4
                Va = V(j,:);
                Vb = V(mod(j, size(V,1)) + 1, :);

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
                dir_vec = pt - [x, y];
                if norm(dir_vec) == 0
                    continue;
                end
                unit_vec = dir_vec / norm(dir_vec);
                joint = [x, y] + l3 * unit_vec;

                t = linspace(0, 1, 10000)';
                seg1 = (1 - t) * [0,0] + t * pt;
                seg2 = (1 - t) * pt + t * joint;
                seg3 = (1 - t) * joint + t * [x, y];

                inside1 = sum(inpolygon(seg1(:,1), seg1(:,2), x_vertices, y_vertices));
                inside2 = sum(inpolygon(seg2(:,1), seg2(:,2), x_vertices, y_vertices));
                inside3 = sum(inpolygon(seg3(:,1), seg3(:,2), x_vertices, y_vertices));

                theta_circ = linspace(0, 2*pi, 10000)';
                blue_circle = R * [cos(theta_circ), sin(theta_circ)] + pt;
                pink_circle = l3 * [cos(theta_circ), sin(theta_circ)] + joint;

                [in_blue, on_blue] = inpolygon(blue_circle(:,1), blue_circle(:,2), x_vertices, y_vertices);
                inside_blue = sum(in_blue & ~on_blue);

                [in_pink, on_pink] = inpolygon(pink_circle(:,1), pink_circle(:,2), x_vertices, y_vertices);
                inside_pink = sum(in_pink & ~on_pink);


                if inside1 <= 10 && inside2 <= 10 && inside3 <= 10 && inside_blue <= 10 && inside_pink <= 10
                    plot([0, pt(1)], [0, pt(2)], 'black', 'LineWidth', 4);
                    midpoint = [x, y] + (l3/R)*dir_vec;  % this is not mathematically midpoint, its the location of link_2_3_joint
                    plot([x, midpoint(1)], [y, midpoint(2)], 'm', 'LineWidth', 4);
                    plot([midpoint(1), pt(1)], [midpoint(2), pt(2)], 'g', 'LineWidth', 4);

                    plot(pt(1), pt(2), 'ko', 'MarkerFaceColor', 'k');
                    plot(x, y, 'ro', 'MarkerFaceColor', 'r');
                    plot(joint(1), joint(2), 'ko', 'MarkerFaceColor', 'k');

                    plot(blue_circle(:,1), blue_circle(:,2), 'b:', 'LineWidth', 1.2);
                    plot(pink_circle(:,1), pink_circle(:,2), 'm:', 'LineWidth', 1.2);

                    angle_degs(end+1) = mod(atan2d(pt(2), pt(1)), 360);
                    Sol(end+1,:) = pt;
                    any_solution_found = true;
                    break;
                end
            end
        end
    end

    theta_circle = linspace(0, 2*pi, 10000)';
    for k = 1:size(Sol,1)
        center = Sol(k,:);
        circle_pts = center + R * [cos(theta_circle), sin(theta_circle)];
        plot(circle_pts(:,1), circle_pts(:,2), 'b:', 'LineWidth', 1.2);
    end

    if ~any_solution_found
        disp('No safe region for entire robot.');
    end

    if ~isempty(angle_degs)
        arg = unique(angle_degs);
        fprintf('The range of theta_1 for which the entire robot has chances of collision is from %.2f to %.2f degrees.\n', min(arg), max(arg));
    else
        fprintf('No valid angles found.\n');
    end
end
