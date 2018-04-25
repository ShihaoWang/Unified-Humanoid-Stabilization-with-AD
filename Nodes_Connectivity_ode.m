function zdot = Nodes_Connectivity_ode(t,z,P)
rIx = z(1);         rIy = z(2);         theta = z(3);
q1 = z(4);          q2 = z(5);          q3 = z(6);
q4 = z(7);          q5 = z(8);          q6 = z(9);
q7 = z(10);         q8 = z(11);         q9 = z(12);
q10 = z(13); 

rIxdot = z(1+13);          rIydot = z(2+13);          thetadot = z(3+13);
q1dot = z(4+13);           q2dot = z(5+13);           q3dot = z(6+13);
q4dot = z(7+13);           q5dot = z(8+13);           q6dot = z(9+13);
q7dot = z(10+13);          q8dot = z(11+13);          q9dot = z(12+13);
q10dot = z(13+13); 
xdot = [rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot]'; 
D_q = P.D_q_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
B_q = P.B_q_fn();
C_q_qdot = P.C_q_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);
coefs_i_child = P.coefs_i_child;
u = zeros(10,1);

Holo_Ind = Holo_Ind_fn(sigma_i);

for i = 1:10
    u(i) = dot(coefs_i_child(:,i), [t^3 t^2 t 1]);           % The default cubic spline is written as a*t^3 + b*t^2 + c*t + d 
end

if sum(Holo_Ind)==0
    xddot = D_q\(-C_q_qdot+ B_q * u);
else
    % Full rank distillation
    Jac_Full = P.Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
    Jacdot_qdot = P.Jacdot_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);
    
    Active_Rows = Jac_Ind_Fn(Act_Holo_Ind);
    Jac_Full = Jac_Full(Active_Rows,:);
    Jacdot_qdot = Jacdot_qdot(Active_Rows,:);
    
    Jac_Temp = Jac_Full';
    [~,colind] = rref(Jac_Temp);
    Jac = Jac_Temp(:, colind)';
    Jacdot_qdot_temp = Jacdot_qdot';
    Jacdot = Jacdot_qdot_temp(:, colind)';
    lamda = (Jac*D_q^(-1)*Jac)\(Jac*D_q^(-1)*C_q_qdot-Jacdot*xdot-Jac*D_q^(-1)*B_q*u);   
    xddot = D_q\(-C_q_qdot + Jac' * lamda + B_q * u);
end

zdot = [xdot; xddot]; 

end