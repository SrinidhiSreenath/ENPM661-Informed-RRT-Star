function [region_Nodes,idx] = regionNodes(tree, newNode, regionvalue)

indices = (tree.nodes(:,1) > (newNode(1)-regionvalue)) & (tree.nodes(:,1) < (newNode(1) + regionvalue)) & (tree.nodes(:,2) > (newNode(2) -regionvalue)) & (tree.nodes(:,2) < (newNode(2) + regionvalue));

region_Nodes = tree.nodes(indices,:);

idx = find(indices == 1);
end