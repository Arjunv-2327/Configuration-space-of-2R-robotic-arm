function fv = transformFV (fv, theta, t)
% Rotate and translate a patch data structure

fv.vertices(:,1) = fv.vertices(:,1) - t(1);   % changing axis of rotation to its own corner point
fv.vertices(:,2) = fv.vertices(:,2) - t(2);

% fv.vertices = bsxfun(@plus, fv.vertices*[cosd(theta) sind(theta); -sind(theta) cosd(theta)], t);

rotated_vertices = fv.vertices * [cosd(theta), sind(theta); -sind(theta), cosd(theta)];

fv.vertices = rotated_vertices + t;


