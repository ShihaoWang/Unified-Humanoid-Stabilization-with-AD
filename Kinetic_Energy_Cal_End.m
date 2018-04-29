function value = Kinetic_Energy_Cal_End(z)
global Ctrl_No

stateNdotNCtrl_ref = z(2:end);
StateNdot_tot = stateNdotNCtrl_ref(1:13*2*Ctrl_No,:);
StateNdot_tot = reshape(StateNdot_tot, 26, Ctrl_No);

% This function is used to calculate the kinetic energy of the system at
% the ending time
StateNdot_End = StateNdot_tot(:,end);

rIx = StateNdot_End(1);             rIy = StateNdot_End(2);             theta = StateNdot_End(3);
q1 = StateNdot_End(4);              q2 = StateNdot_End(5);              q3 = StateNdot_End(6);
q4 = StateNdot_End(7);              q5 = StateNdot_End(8);              q6 = StateNdot_End(9);
q7 = StateNdot_End(10);             q8 = StateNdot_End(11);             q9 = StateNdot_End(12);
q10 = StateNdot_End(13);

rIxdot = StateNdot_End(1+13);          rIydot = StateNdot_End(2+13);          thetadot = StateNdot_End(3+13);
q1dot = StateNdot_End(4+13);           q2dot = StateNdot_End(5+13);           q3dot = StateNdot_End(6+13);
q4dot = StateNdot_End(7+13);           q5dot = StateNdot_End(8+13);           q6dot = StateNdot_End(9+13);
q7dot = StateNdot_End(10+13);          q8dot = StateNdot_End(11+13);          q9dot = StateNdot_End(12+13);
q10dot = StateNdot_End(13+13);

value = T_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);

end

