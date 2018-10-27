function[k] = insideObs(x,y,A1,A2,A3,B1,B2,B3,C1,C2,C3)

% Obstacle 1 (Square)

    if (A1(1)*x + B1(1)*y + C1(1) <= 0) && (A1(2)*x + B1(2)*y + C1(2) <= 0) && (A1(3)*x + B1(3)*y + C1(3) <= 0) && (A1(4)*x + B1(4)*y + C1(4) <= 0)
            p=0;            %point inside obstacle
    else
            p=1;            %point outside obtascle
    end
    
% Obstacle 2 
    
    %Part 1

    if (A2(1)*x + B2(1)*y + C2(1) <= 0) && (A2(2)*x + B2(2)*y + C2(2) <= 0) && (A2(3)*x + B2(3)*y + C2(3) <= 0) && (A2(4)*x + B2(4)*y + C2(4) <= 0)
            q=0;            %point inside obstacle
    else
            q=1;            %point outside obtascle
    end
    
    %Part 2
    
    if (A3(1)*x + B3(1)*y + C3(1) <= 0) && (A3(2)*x + B3(2)*y + C3(2) <= 0) && (A3(3)*x + B3(3)*y + C3(3) <= 0) && (A3(4)*x + B3(4)*y + C3(4) <= 0)
            r=0;            %point inside obstacle
    else
            r=1;            %point outside obtascle
    end
    
% Obstacle 3 - Circle
        
    if ((x-180)^2 + (y-120)^2 - 225)<=0
            s=0;            %point inside obstacle
    else 
            s=1;            %point outside obtascle
    end
    
% Outside Workspace W
    
    if x<1 || y<1 || x> 250 || y> 150
            l=0;            %point is outside workspace W
    else 
            l=1;            %point is inside workspace W
    end

% Final check whether the given node (point) is either inside obstacle (in
% C_obs) or outside workspace W or whether it is in free space (C_free)

    if p==0 || q==0 || r==0 || s==0 || l==0
            k=1;            %point is in C_obs or outside W
    else 
            k=0;            %point is in C_free 
    end
    
    
end
   


