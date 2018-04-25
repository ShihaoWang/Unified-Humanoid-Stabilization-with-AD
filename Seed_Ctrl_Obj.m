function Obj = Seed_Ctrl_Obj(z, P)
% Here z is the vector of the coefficient with the first element to be the
% time. We would like to minimize the time
D_q = P.D_q;
B_q = P.B_q;
C_q_qdot = P.C_q;
qddot = P.qddot;
Jac = P.Jac;
Jacdot = P.Jacdot;

lamda = (Jac*D_q^(-1)*Jac')\(Jac*D_q^(-1)*C_q_qdot-Jacdot-Jac*D_q^(-1)*B_q*z);

Dyn_Via = D_q * qddot + C_q_qdot - Jac' * lamda - B_q * z;

Obj = dot(Dyn_Via, Dyn_Via);

end