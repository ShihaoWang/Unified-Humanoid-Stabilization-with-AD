function Envi_Map = Envi_Map_Defi(flag)

% This function is used to define the environment map for the simulation
% Whenever this function gets called, it will return the array with the
% environment obstacle information

% This is the default flat ground
% This map is defined in a polyline manner with the first value denoting
% the line length and the second value denoting the relative angle
Envi_Map = zeros(2,2);
Envi_Map(1,:) = [5, 0]; 
Envi_Map(2,:) = [3, pi/2];
% [m,n] = size(Envi_Map);
% if flag == 1
%     Envi_Map = reshape(Envi_Map',m*n,1);
% end
end

