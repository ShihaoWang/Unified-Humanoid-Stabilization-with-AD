function Edges_Normal_Angle = Polyline_Normal_fn(Envi_Map)

% This funciton is used to calculate the surface normal for a given map

size_Map = size(Envi_Map);
m_Map = size_Map(1);
Edges_Normal_Angle = zeros(m_Map,1);
Edges_Normal_Angle_temp = Envi_Map(1,2) + pi/2;
for i = 1:m_Map
    Edges_Normal_Angle(i) = Edges_Normal_Angle_temp;
    if i == m_Map
        Edges_Normal_Angle_temp = 0;       
    else
        Edges_Normal_Angle_temp = Edges_Normal_Angle_temp + Envi_Map(i+1,2);     
    end
end
end

