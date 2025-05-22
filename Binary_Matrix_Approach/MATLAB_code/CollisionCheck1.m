function [out, Link, obs1, obs2] = CollisionCheck1 (fv1, fv2)
% Determine if two sets of triangular faces overlap

out = false;
n1 = size (fv1.faces, 1);
n2 = size (fv2.faces, 1);
Link = 0;
obs1 = 0;
obs2 = 0;
for i = 1:n1
    P1 = fv1.vertices(fv1.faces(i,:), :);
    for j = 1:n2
        P2 = fv2.vertices(fv2.faces(j,:), :);
        
%         [P,Q,R,n] = TriIntersect(P1',P2');
%         
%         if ~isempty(P) || ~isempty(Q) || ~isempty(R) || sum(n)
%             out = true;
%             return
%         end
        if TriIntersect(P1',P2')
            Link = ceil(i/2);
            out = true;
            if j < 3
                obs1 = 1;
            else
                obs2 = 2;
            end
        end    
        
%         if (triangle_intersection(P1,P2))
%             out = true;
%             return;
%         end
    end
    
end

end