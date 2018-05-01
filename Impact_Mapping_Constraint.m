function [c,ceq] = Impact_Mapping_Constraint(z,sth)

Active_Ind = sth.Active_Ind;
Null_Jac = sth.Null_Jac;
Temp_Impulse = sth.Temp_Impulse;
stateNdot_i = sth.robotstate;
mini = 0.005;

Node_i_child_Pos = End_Effector_Pos_Vel(stateNdot_i);
[~, Node_i_child_Norm_Ang]= Obs_Dist_Fn(Node_i_child_Pos);

lamda_Full_i = Temp_Impulse + Contact_Force_Back2Full(Null_Jac * z,Active_Ind);

Normal_Force = Contact_Force_Cal_fn(Node_i_child_Norm_Ang(1:end-2,:), lamda_Full_i);
c = -(Normal_Force);
ceq = [];
end

