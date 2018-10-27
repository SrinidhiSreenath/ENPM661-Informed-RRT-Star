% Function that generates all the possible children nodes for a given node.
% In the given problem statement, since it is a 8-connected graph, all the
% possible 8 children nodes for a given node is calculated.

function [child] = childrenNodes(point)

    child(:,:,1) = point + [0 1];
    child(:,:,2) = point + [1 1];
    child(:,:,3) = point + [1 0];
    child(:,:,4) = point + [1 -1];
    child(:,:,5) = point + [0 -1];
    child(:,:,6) = point + [-1 -1];
    child(:,:,7) = point + [-1 0];
    child(:,:,8) = point + [-1 1];
    
end