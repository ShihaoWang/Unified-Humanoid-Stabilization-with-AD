function [c,ceq] = Init_Cons(z0)

global sigma0 mini

c = []; ceq = [];

[End_Pos, End_Vel]= End_Effector_Pos_Vel(z0);

Pos_Dist = Obs_Dist_Fn(End_Pos);

End_Vel = reshape(End_Vel',16,1);

Eqn_Pos_Matrix = blkdiag(sigma0(1),sigma0(1), sigma0(2),sigma0(2),...
                         sigma0(3),sigma0(4),0,0);
                    
Eqn_Vel_Matrix = blkdiag(sigma0(1),sigma0(1),sigma0(1),sigma0(1),...
                         sigma0(2),sigma0(2),sigma0(2),sigma0(2),...
                         sigma0(3),sigma0(3),sigma0(4),sigma0(4),0,0,0,0);
ceq = [ceq; Eqn_Pos_Matrix * Pos_Dist];
ceq = [ceq; Eqn_Vel_Matrix * End_Vel];


Inq_Pos_Matrix = blkdiag(not(sigma0(1)),not(sigma0(1)), not(sigma0(2)),not(sigma0(2)),...
                         not(sigma0(3)),not(sigma0(4)),0,0);

c = [c; - (Inq_Pos_Matrix * Pos_Dist - Inq_Pos_Matrix * ones(8,1) * mini)];

end