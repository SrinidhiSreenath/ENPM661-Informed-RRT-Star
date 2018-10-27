function [RRT, goalNode, path, updatepath] = informedrrtstarpathplan(start_node, goal_node, x, y,A1,A2,A3,B1,B2,B3,C1,C2,C3)

RRT.nodes(1,:) = start_node;
RRT.parent(1,:) = start_node;
RRT.cost(1) = 0;

maxIterations = 3000;
iter = 1;
delta = 7;
region = 10;
chp = 0;

cmin = pdist2(start_node, goal_node);
goal = false;
cbest = 0;
first = 0;
inclination = atan((goal_node(2) - start_node(2))/(goal_node(1) - start_node(1)));

while iter < maxIterations 
    
    new_node = newNode(start_node, goal_node, goal, cmin, cbest ,x, y, inclination);
    
    s = insideObs(new_node(1),new_node(2),A1,A2,A3,B1,B2,B3,C1,C2,C3);
   
    if s == 1
        iter = iter+1;
        continue
    end
    
    dmin = 10000000;
    for i = 1:1:size(RRT.nodes,1)
        d = dist(new_node,RRT.nodes(i,:));
        if d < dmin
            dmin = d;
            var = i;
        end
    end
        
    if dmin<=delta
        update_node = new_node;
    else 
        update_node = updatedNode(RRT.nodes(var,:),new_node,delta);
        dmin = dist(update_node,RRT.nodes(var,:));
    end
        
     valid = checkEdge(RRT.nodes(var,:),update_node,dmin,A1,A2,A3,B1,B2,B3,C1,C2,C3);
        if valid == 1
            iter = iter+1;
            continue
        end

          update_node = round(update_node); 
          
          
    [region_Nodes,idx] = regionNodes(RRT, update_node, region);
    distances = pdist2(region_Nodes, update_node);
    
    distances = RRT.cost(idx)' + distances;
    [dis,id] = min(distances);
    
    parent = RRT.nodes(idx(id),:);
       
    RRT.nodes(end+1,:) = update_node;
    si = size(RRT.nodes,1);
    RRT.parent(si,:) = parent;
    RRT.cost(si) = dis + RRT.cost(idx(id));
    
    cost = RRT.cost(si) + distances;
    check = cost < RRT.cost(idx)';
    rewire = find(check==1);
    
    for j = 1:1:size(rewire,1)
         RRT.parent(rewire(j),:) = update_node;
         RRT.cost(rewire(j)) = cost(rewire(j));
    end
    
    if ~goal
    goal = reachedGoal(update_node,goal_node);
    end
    
if goal
        
        first = first + 1;
        
        if first == 1
            goalNode = size(RRT.nodes,1);
            path = RRT.nodes(goalNode,:);
            fprintf('Found \n');
            
            chp = chp + 1;
        updatepath(chp) = si; 
        test = goalNode;
        path(1,:,chp) = RRT.nodes(test,:);
        pathindex = 2;
        index = 0;

        while index ~=1
                [~, index]=ismember(RRT.parent(test,:),RRT.nodes,'rows');
                path(pathindex,:, chp) = RRT.nodes(index,:);
                test = index;
                pathindex = pathindex + 1;
        end
            
        end
        
%         test = goalNode;
%         path = RRT.nodes(test,:);
        
%                
%         index = 0;
%         while index ~=1
%             [~, index]=ismember(RRT.parent(test,:),RRT.nodes,'rows');
%             path(end+1,:) = RRT.nodes(index,:);
%             test = index;
%         end
%       
         
        
    if mod(iter,100) == 0
             
        chp = chp + 1;
        updatepath(chp) = si; 
        test = goalNode;
        path(1,:,chp) = RRT.nodes(test,:);
        pathindex = 2;
        index = 0;

        while index ~=1
                [~, index]=ismember(RRT.parent(test,:),RRT.nodes,'rows');
                path(pathindex,:, chp) = RRT.nodes(index,:);
                test = index;
                pathindex = pathindex + 1;
        end
        
        for u=1:1:size(path,1)        
            update_node = path(u,:,chp);
            [region_Nodes,idx] = regionNodes(RRT, update_node, region);
            distances = pdist2(region_Nodes, update_node);

            distances = RRT.cost(idx)' + distances;
            [dis,id] = min(distances);

            parent = RRT.nodes(idx(id),:);
            
            [~, upindex]=ismember(update_node,RRT.nodes,'rows');
            if upindex == 1
                break
            end
            RRT.parent(upindex,:) = parent;
            RRT.cost(upindex) = dis + RRT.cost(idx(id));

            cost = RRT.cost(upindex) + distances;
            check = cost < RRT.cost(idx)';
            rewire = find(check==1);

            for j = 1:1:size(rewire,1)
                 RRT.parent(rewire(j),:) = update_node;
                 RRT.cost(rewire(j)) = cost(rewire(j));
            end    
        end     
    end
    
    cbest = 0;
        for j=1:1:size(path,1)-1
            cbest = cbest + pdist2(path(j,:), path(j+1,:));
        end 
    
    iter = iter+1; 
    clear s valid

end

end