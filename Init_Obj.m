function obj = Init_Obj(z0)
global x0_init

% [End_Pos, End_Vel]= End_Effector_Pos_Vel(z0);

% obj = -End_Pos(end-1,2);
obj = dot(z0 - x0_init, z0 - x0_init);

end