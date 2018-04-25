function Optimized_State_Plot(Var_Opt, P)
StateNdot_tot = Var_Opt(2:26*P.Ctrl_No+1,:);
StateNdot_tot = reshape(StateNdot_tot,26,P.Ctrl_No);
[~,n] = size(StateNdot_tot);
axes_plot = axes;
for i = 1:n   
    q_array_i = StateNdot_tot(:,i)';
    Single_Frame_Plot(q_array_i, P,axes_plot);
    pause(0.01);
end
end

