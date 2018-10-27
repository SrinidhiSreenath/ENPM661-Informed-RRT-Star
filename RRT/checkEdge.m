function valid = checkEdge(node1, node2,dmin,A1,A2,A3,B1,B2,B3,C1,C2,C3)

m = (node2(2) - node1(2))/(node2(1) - node1(1));
c = node1(2) - m*node1(1);

        for i=1:1:floor(dmin)
            x = node1(1) + i;
            y = m*x + c;
            k = insideObs(x,y,A1,A2,A3,B1,B2,B3,C1,C2,C3);
            
            if k==1
                valid = 1;
                return
            end
        end
        
        valid = 0;
        
end