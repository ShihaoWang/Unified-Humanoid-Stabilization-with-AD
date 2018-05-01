function Ctrl_Torque_Plot(Var_Opt)
Ctrl_No = 15;
% This funciton is ued to plot the control torque along the given Var_Opt
time_tot = linspace(0,Var_Opt(1),Ctrl_No);



Ctrl_Torque_tot = Var_Opt(2:26*Ctrl_No+1,:);
Ctrl_Torque_tot = reshape(Ctrl_Torque_tot,26,Ctrl_No);
Ctrl_Torque_tot = Ctrl_Torque_tot(14:end,:);
plot(time_tot, Ctrl_Torque_tot);

% Ctrl_Torque_tot = Var_Opt(26*Ctrl_No+2:end,:);
Ctrl_Torque_tot = Ctrl_Torque_tot(:,14:end)

time_inte_tot = linspace(0,Var_Opt(1),50*Ctrl_No);
Ctrl_inte_tot = spline(time_tot, Ctrl_Torque_tot, time_inte_tot);
plot(time_inte_tot,Ctrl_inte_tot,'linewidth',1);
xlabel('time(s)')
ylabel('kinetic energy(J)')

end

