function intersects = intersection_checker(p1, p2, q1, q2)
    intersects = false;
    denom = (q2(2) - q1(2)) * (p2(1) - p1(1)) - (q2(1) - q1(1)) * (p2(2) - p1(2));
    if denom == 0, return; end
    ua = ((q2(1) - q1(1)) * (p1(2) - q1(2)) - (q2(2) - q1(2)) * (p1(1) - q1(1))) / denom;
    ub = ((p2(1) - p1(1)) * (p1(2) - q1(2)) - (p2(2) - p1(2)) * (p1(1) - q1(1))) / denom;
    if ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1
        intersects = true;
    end
end