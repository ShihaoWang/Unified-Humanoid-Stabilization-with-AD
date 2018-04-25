function [c, ceq] = Seed_Ctrl_Constraint(z, P)
c = []; ceq = [];
D_q = P.D_q;
B_q = P.B_q;
C_q_qdot = P.C_q;
Jac = P.Jac;
Jacdot = P.Jacdot;

lamda = (Jac*D_q^(-1)*Jac')\(Jac*D_q^(-1)*C_q_qdot-Jacdot-Jac*D_q^(-1)*B_q*z);
Active_In = P.Active_In;
% The main constraint here is to make sure the force is supportive
if find(Active_In)==0
    return
else
    for i = 1:length(find(Active_In))
        Lamda_Ind = Active_In(i);
        if Lamda_Ind<9
            if mod(Lamda_Ind,2)==0
                c = [c; -lamda(i)];
            end
        else
            if mod(Lamda_Ind,2)==1
                c = [c; lamda(i)];
            end
            
        end
    end
end

end