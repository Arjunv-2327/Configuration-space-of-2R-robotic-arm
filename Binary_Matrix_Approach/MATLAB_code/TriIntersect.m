function flag = TriIntersect(U,V)
% function [P,Q,R,n]=TriIntersect(U,V)
% TRIINTERSECTRF intersection of two triangles
% [P,Q,R,n] = TriIntersectRF(U,V) produces the vertices of U inside V (P), the
% intersections between the edges of U and V (Q), the vertices of V
% inside U (R) and the neighbours of U that also intersect V (n).
% The result is the same as TriIntersect(U,V) up to error on the order of
% machine epsilon. The calculations are performed without change of
% coordinates to a reference triangle.

%  Conor Mccoid and Martin J Gander. A provably robust
% algorithm for triangle-triangle intersections in floatingpoint arithmetic. ACM Transactions on Mathematical
% Software (TOMS), 48(2):1–30, 2022.

flag = false;
W=V(:,[2,3,1])-V; % vectors between vertices of V
T=[0,-1;1,0]*W; T=sign(T(:,1)'*W(:,2))*T; % vectors perp. to W

% n=zeros(1,3);

indR=zeros(3); indP=indR; p=indR;
Q=zeros(2,9); indQ=zeros(1,9); sq=indR; q0=indR;
for i=1:3 % for each edge of V
    qp=[W(:,i),T(:,i)]\(U-V(:,i)); % parametrization
    p(i,:)=qp(2,:); indP(i,:)=sign(p(i,:))>=0; % p-coordinates
    
    
    q0(i,[2,3,1])=cross(qp(1,:),qp(2,:)); % num. of q0
    sq(i,:)=sign(q0(i,:))>=0; % sign of num. of q0
end

P=U(:,prod(indP)==1);

if ~isempty(P)
    flag = true;
    return
end

for i=1:3
    j=mod(i,3)+1; k=mod(i+1,3)+1; % nhbring ref. lines
    for a=1:3
        b=mod(a,3)+1; m=a+3*(i-1);
        if indP(i,a)~=indP(i,b) % see Sec. 4.1
            if (sq(i,a)~=indP(i,b) && indP(k,a)~=indP(k,b)) || (~indP(k,a) && ~indP(k,b))
                indR(i,[i,j])=indR(i,[i,j])+[0.25,0.75];
            elseif (sq(j,a)==indP(i,b) && indP(j,a)~=indP(j,b)) || (~indP(j,a) && ~indP(j,b))
                indR(i,[i,j])=indR(i,[i,j])+[0.75,0.25];
            else
                indR(i,[i,j])=indR(i,[i,j])+[0.75,0.75];
                
                Q(:,m)=V(:,i) + (q0(i,a)/(p(i,b)-p(i,a)))*W(:,i); % eqn (2)
                indQ(m)=1;
                
                Q=Q(:,indQ==1);
                
                if ~isempty(Q)
                    flag = true;
                    return
                end
                
                %                 n(a)=1; % nhbrs of U intersect V
                
                %                 if sum(n)
                flag = true;
                return
                %                 end
                
            end
            
        end
    end
end

R=V(:,max(indR==1));

if ~isempty(R)
    flag = true;
    return
end
% P=U(:,prod(indP)==1); Q=Q(:,indQ==1); R=V(:,max(indR==1));