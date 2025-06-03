function centers = centre_on_link1_circle(r1, r2, x3, y3)
    d = sqrt(x3^2 + y3^2);
    if d > r1 + r2 || d < abs(r1 - r2) || d == 0
        centers = [];
        return;
    end

    a = (r1^2 - r2^2 + d^2) / (2 * d);
    h = a * x3 / d;
    k = a * y3 / d;

    h_offset = -y3 * sqrt(r1^2 - a^2) / d;
    k_offset = x3 * sqrt(r1^2 - a^2) / d;

    centers = [h + h_offset, k + k_offset;
               h - h_offset, k - k_offset];
end
