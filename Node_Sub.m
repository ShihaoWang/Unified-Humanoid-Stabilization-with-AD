function Node = Node_Sub(Node, sigma0, x0)
Node.mode = sigma0;
Node.robotstate = x0;
KE0 = Kinetic_Energy_Cal(x0);
Node.KE = KE0;
[Node_i_Pos, Node_i_Vel]= End_Effector_Pos_Vel(x0);
Node.End_Pos = Node_i_Pos;
Node.End_Vel = Node_i_Vel;
end

