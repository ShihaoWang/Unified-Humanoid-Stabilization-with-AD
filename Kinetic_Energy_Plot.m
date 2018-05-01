function Kinetic_Energy_Plot(Var_Opt)
Ctrl_No = 15;
% This function is used to calculate the kinetic energy of the given
% locomotion during the whole process
time_tot = linspace(0,Var_Opt(1),Ctrl_No);
StateNdot_tot = Var_Opt(2:26*Ctrl_No+1,:);
StateNdot_tot = reshape(StateNdot_tot,26,Ctrl_No);
KE_tot = zeros(Ctrl_No,1);
for i = 1:Ctrl_No
    RobotState_i = StateNdot_tot(:,i);
    KE_i = Kinetic_Energy_Cal(RobotState_i); 
    KE_tot(i) =  KE_i;  
end
time_inte_tot = linspace(0,Var_Opt(1),50*Ctrl_No);
KE_inte_tot = spline(time_tot, KE_tot, time_inte_tot);
plot(time_inte_tot,KE_inte_tot,'linewidth',2);
xlabel('time(s)')
ylabel('kinetic energy(J)')
xlim([time_inte_tot(1) time_inte_tot(end)])

end

