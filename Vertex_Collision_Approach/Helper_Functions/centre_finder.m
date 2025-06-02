function [angles, P_circle, P_centres] = centre_finder(l1, l2, l3, m, c, xmin, xmax)
    % Inputs:
    % l1, l2, l3 - segment lengths
    % m, c       - slope and intercept of line y = m*x + c
    % xmin, xmax - bounds of line segment on x-axis
    %
    % Outputs:
    % angles     - angles (in radians, 0 to 2*pi) of l1 segment from +x axis
    % P_circle   - points (px, py) on l2 circle
    % P_centres  - centers (h,k) of l2 circle
    
    x_samples = linspace(xmin, xmax, 5000);
    y_samples = m * x_samples + c;
    
    candidates = []; % Store valid solutions: [d_diff, h, k, px, py, xp, yp]
    
    for i = 1:length(x_samples)
        alpha = x_samples(i);
        beta = y_samples(i);
        
        centers = find_circle_center_nosymbolic(l1, l2 + l3, alpha, beta);
        if isempty(centers)
            continue;
        end
        
        for j = 1:size(centers,1)
            h = centers(j,1);
            k = centers(j,2);
            
            xp = (h + m*(k - c)) / (1 + m^2);
            yp = m * xp + c;
            
            if xp < xmin
                xp = xmin; yp = m*xp + c;
            elseif xp > xmax
                xp = xmax; yp = m*xp + c;
            end
            
            dir = [xp - h, yp - k];
            unit_dir = dir / norm(dir);
            
            px = xp - l3 * unit_dir(1);
            py = yp - l3 * unit_dir(2);
            
            dist_l2 = norm([px - h, py - k]);
            d_diff = abs(dist_l2 - l2);
            
            if d_diff < 1e-6
                candidates = [candidates; d_diff, h, k, px, py, xp, yp];
            end
        end
    end
    
    if isempty(candidates)
        disp('No valid centers found that satisfy all constraints.');
        angles = [];
        P_centres = [];
        P_circle = [];
        return;
    end
    
    candidates = sortrows(candidates, 1);
    n_best = min(2, size(candidates,1));
    best = candidates(1:n_best, :);
    
    P_centres = best(:, 2:3); % centers (h,k)
    P_circle = best(:, 4:5);  % points (px, py)
    
    % Calculate angles for each center relative to +x axis (0 to 2*pi)
    angles = atan2(P_centres(:,2), P_centres(:,1));
    angles(angles < 0) = angles(angles < 0) + 2*pi;
    
    % Plotting
    figure; hold on; axis equal; 
    axis off; % TURN OFF axes
    
    theta = linspace(0, 2*pi, 500);
    plot(l1*cos(theta), l1*sin(theta), 'b', 'LineWidth', 1.5);
    scatter(0,0,100,'r','filled');
    
    x_line = linspace(xmin, xmax, 500);
    y_line = m*x_line + c;
    plot(x_line, y_line, 'r', 'LineWidth', 1.5);
    
    color_l1 = 'k';
    color_l2 = [0.5, 0.25, 0];
    color_l3 = [1, 0.4, 0.7];
    
    for i = 1:n_best
        h = best(i, 2);
        k = best(i, 3);
        px = best(i, 4);
        py = best(i, 5);
        xp = best(i, 6);
        yp = best(i, 7);
        
        x_circ = h + l2*cos(theta);
        y_circ = k + l2*sin(theta);
        plot(x_circ, y_circ, '--', 'Color', color_l2, 'LineWidth', 1.5);
        
        scatter(h,k,100,'r','filled');
        
        plot([0, h], [0, k], '-', 'LineWidth', 2.5, 'Color', color_l1);
        plot([h, px], [k, py], '-', 'LineWidth', 2.5, 'Color', color_l2);
        plot([px, xp], [py, yp], '-', 'LineWidth', 2.5, 'Color', color_l3);
        
        scatter(px, py, 100, 'r', 'filled');
        scatter(xp, yp, 60, 'ko', 'filled');
    end
    
    hold off;
end


function centers = find_circle_center_nosymbolic(l1, r2, xp, yp)
    x1 = 0; y1 = 0;
    x2 = xp; y2 = yp;
    d = sqrt((x2 - x1)^2 + (y2 - y1)^2);
    
    if d > l1 + r2 || d < abs(l1 - r2)
        centers = [];
        return;
    end
    
    a = (l1^2 - r2^2 + d^2) / (2*d);
    x3 = x1 + a*(x2 - x1)/d;
    y3 = y1 + a*(y2 - y1)/d;
    
    h = sqrt(l1^2 - a^2);
    rx = -(y2 - y1)*(h/d);
    ry =  (x2 - x1)*(h/d);
    
    centers = [x3 + rx, y3 + ry;
               x3 - rx, y3 - ry];
end
