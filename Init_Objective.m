function obj = Init_Objective(z,P)

rIx        = z(1);
rIy        = z(2);
theta      = z(3);
q1         = z(4);
q2         = z(5);
q3         = z(6); 
q4         = z(7);

q_now = [rIx rIy theta q1 q2 q3 q4]';

rIx_ref = P.x0(1);
rIy_ref= P.x0(2);
theta_ref = P.x0(3);
q1_ref= P.x0(4);
q2_ref= P.x0(5);
q3_ref= P.x0(6);
q4_ref= P.x0(7);

q_ref = [rIx_ref rIy_ref theta_ref q1_ref q2_ref q3_ref q4_ref]';

rA_fn = P.rA_fn;
rB_fn = P.rB_fn;

rA = rA_fn(q1,q2,rIx,rIy,theta);
rB = rB_fn(q3,q4,rIx,rIy,theta);

rI = P.rI_fn(rIx,rIy);

if sum(P.init_contas) == 2
    obj = (-rI(2) * (rA(1) - rB(1))^2) + dot(q_now - q_ref, q_now - q_ref);
else
    obj = 1;
end

end