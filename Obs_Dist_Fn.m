function Obs_Dist = Obs_Dist_Fn(Point_i, Envi_Map, Direction)
% This function gives the signed distance from a point to the surface of
% environmental obstacle
Obs_Dist_Array = [];
if nargin<3
    [m,~] = size(Envi_Map);
    for i = 1:m
        Envi_Map_i = Envi_Map(i,:);
        Dist_i = Dist_Cal_Fn(Point_i, Envi_Map_i);       
        Obs_Dist_Array_i = Dist_i;
        Obs_Dist_Array = [Obs_Dist_Array;Obs_Dist_Array_i ];
    end
else
    [m,~] = size(Envi_Map);
    for i = 1:m
        Envi_Map_i = Envi_Map(i,:);
        Dist_i = Dist_Cal_Fn(Point_i, Envi_Map_i, Direction);
        Obs_Dist_Array_i = Dist_i;
        Obs_Dist_Array = [Obs_Dist_Array;Obs_Dist_Array_i ];
    end
end
if length(Obs_Dist_Array)>1
    Obs_Dist = min(Obs_Dist_Array);
else
    Obs_Dist = Obs_Dist_Array;
end

end
function Dist_i = Dist_Cal_Fn(Point_i, Envi_Map_i, Direction)
% This function will compute the environmental obstacle geometry to have
% the linear expression
Envi_Map_Point_A_x = Envi_Map_i(1);
Envi_Map_Point_A_y = Envi_Map_i(2);
Envi_Map_Point_B_x = Envi_Map_i(3);
Envi_Map_Point_B_y = Envi_Map_i(4);
Dist_i = [];
if nargin<3
    if Envi_Map_Point_A_x == Envi_Map_Point_B_x
        Dist_i = Envi_Map_Point_A_x - Point_i(1);
        return
    end
    if Envi_Map_Point_A_y == Envi_Map_Point_B_y
        Dist_i = Point_i(2) - Envi_Map_Point_A_y;
        return
    end
else
    if Direction =='x'
        if Envi_Map_Point_A_x == Envi_Map_Point_B_x
            Dist_i = Envi_Map_Point_A_x - Point_i(1);
            return
        end
    else
        if Envi_Map_Point_A_y == Envi_Map_Point_B_y
            Dist_i = Point_i(2) - Envi_Map_Point_A_y;
            return
        end
    end
end
end