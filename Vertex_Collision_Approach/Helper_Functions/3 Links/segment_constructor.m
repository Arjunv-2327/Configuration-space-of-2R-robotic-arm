function [locations, args] = segment_constructor(obstacle, centre, radius, length)
    % Initializations 
    locations = [];   
    args = [];

    % Plotting functions
    figure;
    hold on;
    axis equal;
    axis off;

    % Obstacle and circle defining and plotting
    V = obstacle.vertices;
    F = obstacle.faces;
    x_vertices = V(:,1);
    y_vertices = V(:,2);

    patch('Vertices', V, 'Faces', F, 'FaceColor', [1 0.4 0], 'EdgeColor', 'k', 'LineWidth', 1.5); % Obstacle
    plot(centre(1), centre(2), 'ko', 'MarkerFaceColor', 'red', 'MarkerSize', 10); % Relative origin

    circle_util = linspace(0, 2*pi, 10000); % used for circles which are to be plotted 

    %Main base circle centred at centre[h, k] and radius given by user
    x = radius * cos(circle_util) + centre(1);
    y = radius * sin(circle_util) + centre(2);
    plot(x, y, 'k:', 'LineWidth', 1.2);  

    % Computing required line segments 
    for i = 1:4
        vx = V(i, 1);
        vy = V(i, 2);
        alpha = vx;
        beta = vy;

        

        k_val = alpha^2 + beta^2 + radius^2 - length^2;
        A = 4 * (alpha^2 + beta^2);
        B = -4 * k_val * alpha;
        C = k_val^2 - 4 * beta^2 * radius^2;
        D = B^2 - 4 * A * C;

        if D >= 0 
            sqrtD = D^0.5;
            x1 = (-B + sqrtD) / (2 * A);
            y1 = (k_val - 2 * x1 * alpha) / (2 * beta);
            x2 = (-B - sqrtD) / (2 * A);
            y2 = (k_val - 2 * x2 * alpha) / (2 * beta);

            points = [x1, y1;   % point 1
                      x2, y2];  % point 2

            for j = 1:2
                B = points(j, :);  % B is the jth point as (x, y)
                vertex = [vx, vy];
                dir_vec = B - vertex;
                if norm(dir_vec) == 0
                    continue;
                end

                % Initialize segment for intersection checks
                t = linspace(0,1,10000)';
                seg_x = (1 - t) * B(1) + t * vertex(1);
                seg_y = (1 - t) * B(2) + t * vertex(2);

                link1_x = (1 - t) * B(1) + t * 0;
                link1_y = (1 - t) * B(2) + t * 0;

                grz_circle_x = length * cos(circle_util) + B(1);
                grz_circle_y = length * sin(circle_util) + B(2);

                % Count interior-only intersection points using inpolygon (excluding edges)
                [b1, e1] = inpolygon(seg_x, seg_y, x_vertices, y_vertices);
                inter_seg = sum(b1 & ~e1);

                [b2, e2] = inpolygon(link1_x, link1_y, x_vertices, y_vertices);
                inter_link1 = sum(b2 & ~e2);

                [b3, e3] = inpolygon(grz_circle_x, grz_circle_y, x_vertices, y_vertices);
                inter_grz_circle = sum(b3 & ~e3);

                % Finding and plotting only valid configurations
                if inter_seg <= 2 && inter_link1 <= 2 
                    if inter_grz_circle <= 2
                        plot([B(1), vx], [B(2), vy], 'green', 'LineWidth', 4);  % Segment gets plotted
                        plot(grz_circle_x, grz_circle_y, 'b:', 'LineWidth', 1.2); % Blue grazing circle gets plotted
                        plot([centre(1), B(1)], [centre(2), B(2)], 'black', 'LineWidth', 4) % First link gets plotted in black
                        plot(B(1), B(2), 'ko', 'MarkerFaceColor', 'red', 'MarkerSize', 7.5); % Joint

                        locations(end+1, :) = B;
                        args(end+1) = mod(atan2(B(2), B(1)), 2*pi);
                    elseif inter_grz_circle > 2
                        for k = 1:4
                            Va = V(F(k), :);                    % Current vertex using face indices F(1) = 1, F(2) = 2, F(3) = 4, F(4) = 3
                            Vb = V(F(mod(k, 4) + 1), :);  % Next vertex in face order (wrap around) , obstacle.faces = [1 2 4 3] = F 
                            if abs(Vb(1) - Va(1)) < 1e-6
                                continue;
                            end

                            m = (Vb(2) - Va(2)) / (Vb(1) - Va(1));
                            c = Va(2) - m * Va(1);

                            % SYSTEM EQUATIONS for perpendicular tangent
                            
                            eqs = @(vars) [
                                vars(2) - m*vars(1) - c;
                                vars(1) - (radius*cos(vars(3)) - m*c + m*radius*sin(vars(3))) / (1 + m^2);
                                (vars(1) - radius*cos(vars(3)))^2 + (vars(2) - radius*sin(vars(3)))^2 - length^2
                            ];

                            % INITIAL GUESSES FOR BOTH SOLUTIONS (column vectors)
                            guesses = [0, 0, 0; 
                                       1, -1, pi/2]';

                            opts = optimoptions('fsolve','Display','off');
                            solutions = [];  % Store all valid solutions

                            % FIND SOLUTIONS (removed redundant loops)
                            for guess_idx = 1:size(guesses,2)
                                [sol, ~, exitflag] = fsolve(eqs, guesses(:,guess_idx), opts);
                                if exitflag > 0
                                    solutions = [solutions sol];  % Store as columns
                                end
                            end

                            % PROCESS EACH SOLUTION (keep original processing logic)
                            for sol_idx = 1:size(solutions,2)
                                x_edge = solutions(1,sol_idx);
                                y_edge = solutions(2,sol_idx);
                                theta_val = solutions(3,sol_idx);

                                if x_edge >= min([Va(1), Vb(1)]) && x_edge <= max([Va(1), Vb(1)])
                                    pt = [radius * cos(theta_val), radius * sin(theta_val)];

                                    % KEEP ORIGINAL GEOMETRY CODE
                                    t = linspace(0, 1, 10000);
                                    seg1_x = (1 - t) * pt(1) + t * x_edge;
                                    seg1_y = (1 - t) * pt(2) + t * y_edge;

                                    linkone_x = (1 - t) * pt(1);
                                    linkone_y = (1 - t) * pt(2);

                                    grz_circle1_x = length * cos(circle_util) + pt(1);
                                    grz_circle1_y = length * sin(circle_util) + pt(2);

                                    % ORIGINAL COLLISION CHECKS
                                    [b4, e4] = inpolygon(seg1_x, seg1_y, x_vertices, y_vertices);
                                    inter_seg1 = sum(b4 & ~e4);

                                    [b5, e5] = inpolygon(linkone_x, linkone_y, x_vertices, y_vertices);
                                    inter_linkone = sum(b5 & ~e5);

                                    [b6, e6] = inpolygon(grz_circle1_x, grz_circle1_y, x_vertices, y_vertices);
                                    inter_grz_circle1 = sum(b6 & ~e6);

                                    if inter_seg1 <= 2 && inter_linkone <= 2 && inter_grz_circle1 <= 2
                                        plot([pt(1) x_edge], [pt(2) y_edge], 'g-', 'LineWidth', 4);
                                        plot(grz_circle1_x, grz_circle1_y, 'b:', 'LineWidth', 1.2);
                                        plot([centre(1), pt(1)], [centre(2), pt(2)], 'black', 'LineWidth', 4) % First link gets plotted in black
                                        plot(pt(1), pt(2), 'ko', 'MarkerFaceColor', 'red', 'MarkerSize', 7.5); % Joint

                                        locations(end+1, :) = [pt(1), pt(2)];
                                        args(end+1) = mod(atan2(pt(2), pt(1)), 2*pi);
                                    else
                                        
                                        disp('Segment cannot be constructed for the given obstacle and circle');
                                    end
                                else
                                    disp('Segment cannot be constructed for the given obstacle and circle');
                                end
                            end
                        end
                    end
                end
            end
        else
            disp('Segment cannot be constructed for the given obstacle and circle, D is negative');
        end
    end
end
