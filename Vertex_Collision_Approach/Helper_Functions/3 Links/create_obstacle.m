function obstacle = create_obstacle(l, w, alpha, beta, phi)
    % Midpoints of 2 parallel widths
    M_x = alpha - l * cos(phi) / 2;
    N_x = alpha + l * cos(phi) / 2;
    M_y = beta - l * sin(phi) / 2;
    N_y = beta + l * sin(phi) / 2;

    % Rectangle vertices (rotated)
    obstacle.vertices = [
        M_x - w * sin(phi) / 2, M_y + w * cos(phi) / 2;
        N_x - w * sin(phi) / 2, N_y + w * cos(phi) / 2;
        M_x + w * sin(phi) / 2, M_y - w * cos(phi) / 2;
        N_x + w * sin(phi) / 2, N_y - w * cos(phi) / 2
    ];

    % Rectangle face definition (no diagonal)
    obstacle.faces = [1 2 4 3];
end
