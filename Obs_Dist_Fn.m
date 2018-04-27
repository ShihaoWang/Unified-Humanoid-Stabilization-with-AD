function [Pos_Dist, Env_Ang] = Obs_Dist_Fn(End_Pos)

% This function is used to calculate the relative distance between the
% robot end effector and the nearby environment

%% 1. Retrieve the environment obstacle information
Envi_Map = Envi_Map_Defi(0);
size_Map = size(Envi_Map);
m_Map = size_Map(1);

%% 2. Compute the polyline normal
Edges = Map_Edge(Envi_Map);
Edges_Normal_Angle = Polyline_Normal_fn(Envi_Map);

%% 3. Then is to compute the minimum distance
size_End_Pos = size(End_Pos);  % Here the m is a stack of the position vectors: 6 
                               %          n is the number of coordinates: 2
m_Pos = size_End_Pos(1);
Pos_Dist = zeros(m_Pos,1);
Env_Ang = Pos_Dist;
for i = 1:m_Pos
    Pos_i = End_Pos(i,:);
    Pos_Dist_temp = zeros(m_Map,1);
    for j = 1:m_Map
        Edge_temp = Edges(j,:);
        Edge_Normal_temp = Edges_Normal_Angle(j,:);
        Edge_Normal_vec_temp = [cos(Edge_Normal_temp) sin(Edge_Normal_temp)];
        Edge_Offset = Pos_i - Edge_temp;
        Pos_Dist_temp(j) = dot(Edge_Offset, Edge_Normal_vec_temp);
    end         
    [Pos_Dist_i, Env_Ind_vec] = Min_Ind_Sel(Pos_Dist_temp);
    Pos_Dist(i) = Pos_Dist_i;
    Env_Ang(i) = dot(Env_Ind_vec, Edges_Normal_Angle);
end
end
