function Obj = Seed_Conf_Obj(z)
global Node_i

% Obj = -z(2);
robotstate_off = Node_i.robotstate - z;
Obj = dot(robotstate_off,robotstate_off);
% Obj = Kinetic_Energy_Cal(z);
% Obj = 1;

% [End_Pos, ~]= End_Effector_Pos_Vel(z);
% Obj = - End_Pos(7,2);

end