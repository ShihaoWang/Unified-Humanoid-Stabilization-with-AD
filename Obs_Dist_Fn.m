function Pos_Dist = Obs_Dist_Fn(End_Pos)

% This function is used to calculate the relative distance between the
% robot end effector and the nearby environment

%% 1. Retrieve the environment obstacle information
Envi_Map = Envi_Map_Defi(0);
size_Map = size(Envi_Map);
m_Map = size_Map(1);

%% 2. Compute the polyline normal
Edges = Map_Edge(Envi_Map);
Edges_Normal_Angle = Polyline_Normal_fn(Envi_Map);

%% 3. Then is to determine which bisector the current position lies within
size_End_Pos = size(End_Pos);  % Here the m is a stack of the position vectors: 6 
                                %          n is the number of coordinates: 2
m_Pos = size_End_Pos(1);
Pos_Dist = zeros(m_Pos,1);
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
    Pos_Dist(i) = min(Pos_Dist_temp);
end
end
