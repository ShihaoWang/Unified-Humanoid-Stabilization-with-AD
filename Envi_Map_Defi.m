function P = Envi_Map_Defi(P)

% This function is used to define the environment map for the simulation
P.Envi_Map = [-100 0 100 0]; % This is the default flat ground

P.Envi_Map = [P.Envi_Map; 5 0 5 10]; % This is a vertical wall at 5m

end

