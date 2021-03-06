function Obj = Nodes_Connectivity_Obj(z)
global Ctrl_No
stateNdot_ref = z(2:1 + (26 + 10) * (Ctrl_No - 1));
StateNdot_tot = stateNdot_ref(1:13 * 2 * (Ctrl_No - 1),:);
StateNdot_tot = reshape(StateNdot_tot, 26, (Ctrl_No - 1));

% stateNdotNCtrl_ref = z(2:end);
% StateNdot_tot = stateNdotNCtrl_ref(1:13*2*(Ctrl_No - 1),:);
Ctrl_tot = stateNdot_ref(1+26*(Ctrl_No - 1):end,:);

Obj_Ctrl = dot(Ctrl_tot,Ctrl_tot);

% Ctrl_tot = reshape(Ctrl_tot, 10, (Ctrl_No - 1));
% Ctrl_tot = [zeros(10,1) Ctrl_tot];
% 

Tdot = 0;
for i = 1: Ctrl_No - 1
    x_i = StateNdot_tot(:,i);   
    rIx = x_i(1);             rIy = x_i(2);             theta = x_i(3);
    q1 = x_i(4);              q2 = x_i(5);              q3 = x_i(6);
    q4 = x_i(7);              q5 = x_i(8);              q6 = x_i(9);
    q7 = x_i(10);             q8 = x_i(11);             q9 = x_i(12);
    q10 = x_i(13);
    rIxdot = x_i(1+13);          rIydot = x_i(2+13);          thetadot = x_i(3+13);
    q1dot = x_i(4+13);           q2dot = x_i(5+13);           q3dot = x_i(6+13);
    q4dot = x_i(7+13);           q5dot = x_i(8+13);           q6dot = x_i(9+13);
    q7dot = x_i(10+13);          q8dot = x_i(11+13);          q9dot = x_i(12+13);
    q10dot = x_i(13+13);
%     T_i = T_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);
        Tdot_i = Tdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);
    Tdot = Tdot + Tdot_i*Tdot_i;
%     Tdot = Tdot + T_i*T_i + 100*Tdot_i * Tdot_i;
    
end
Obj_KEdot = Tdot;
% if T<0.01
%     Obj = 0.01;
% end

% Obj = Obj_Ctrl + Obj_KEdot;
Obj = 0*Obj_Ctrl + Obj_KEdot;
end