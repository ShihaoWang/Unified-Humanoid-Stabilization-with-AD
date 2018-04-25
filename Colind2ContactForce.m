function Contact_Force_i = Colind2ContactForce(colind, u, P)
% This function is used to generate the contact force from Colind2

Contact_Force_i = zeros(12,1);

D_q = P.D_q;
B_q = P.B_q;
C_q_qdot = P.C_q;
Jac = P.Jac;
Jacdot = P.Jacdot;

lamda_raw_i = (Jac * (D_q\Jac'))\(Jac *(D_q\ C_q_qdot) - Jacdot - Jac * (D_q\ B_q) * u);
for i = 1:length(lamda_raw_i)
    Contact_Force_i(colind(i)) = lamda_raw_i(i);   
end
end
