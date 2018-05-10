function StateNdot_End = Edge_State_Distill(z)
global Ctrl_No
stateNdotNCtrl_ref = z(2:end);
StateNdot_tot = stateNdotNCtrl_ref(1:13*2*(Ctrl_No-1),:);
StateNdot_tot = reshape(StateNdot_tot, 26, Ctrl_No-1);

% This function is used to calculate the kinetic energy of the system at
% the ending time
StateNdot_End = StateNdot_tot(:,end);

end

