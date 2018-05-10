function Optimized_State_Plot(Var_Opt)
global Ctrl_No
StateNdot_tot = Var_Opt(2:26*(Ctrl_No-1)+1,:);
StateNdot_tot = reshape(StateNdot_tot,26,(Ctrl_No-1));

[~,n] = size(StateNdot_tot);

Exp_Rate = 5;
Time_Span = linspace(0,Var_Opt(1),(Ctrl_No-1));
Time_Inte_Span = linspace(0,Var_Opt(1),Exp_Rate * (Ctrl_No-1));

StateNdot_Inte_tot = spline(Time_Span, StateNdot_tot, Time_Inte_Span);
StateNdot_tot = StateNdot_Inte_tot;

for i = 1:Exp_Rate*(Ctrl_No-1)
    q_array_i = StateNdot_tot(:,i)';
    Single_Frame_Plot(q_array_i);
    pause(0.01);
end
end

