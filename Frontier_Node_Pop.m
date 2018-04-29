function [Node_i, R]= Frontier_Node_Pop(R)

% This function is used to pop the node out whose kinetic energy is the
% minimum from all the other nodes in the current Frontier

Kinetic_Array = zeros(length(R),1);

for i = 1:length(R)
    Kinetic_Array(i) = R{i}.KE;
end

[~,Ind] = min(Kinetic_Array);

Node_i = R{Ind};

R(Ind,:) = [];
end

