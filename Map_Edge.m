function Edges = Map_Edge(Envi_Map)

% This function is used to compute the edge for a given map
size_Map = size(Envi_Map);

m = size_Map(1);
% For m row there are m edging points including the default origin

Edges = zeros(m+1,2);

Temp_Edge = [ 0 0 ];

Edges(1,:) = Temp_Edge;

for i = 1:m
    Temp_r = Envi_Map(i,1);
    Temp_theta = Envi_Map(i,2);
    
    Temp_Edge_x = Temp_Edge(1) + Temp_r * cos(Temp_theta);
    Temp_Edge_y = Temp_Edge(2) + Temp_r * sin(Temp_theta);
    
    Edges(i+1,:) = [Temp_Edge_x Temp_Edge_y];
    
    Temp_Edge = Temp_Edge + [Temp_Edge_x Temp_Edge_y];
end
end