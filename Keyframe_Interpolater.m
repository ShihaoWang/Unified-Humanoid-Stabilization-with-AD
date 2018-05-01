function Keyframe_Interpolater(Opt_Seed)
% This function is used to interpolate the given keyframes to a smooth traj

Var_Opt = Opt_Seed;
Frame_No = (length(Var_Opt) - 1)/36;
Time = Var_Opt(1);
State_Traj = Var_Opt(2:26*Frame_No+1,:);
State_Traj = reshape(State_Traj, 26, length(State_Traj)/26);
Ctrl_Traj = Var_Opt(26*Frame_No+2:end,:);
Ctrl_Traj = reshape(Ctrl_Traj, 10, length(Ctrl_Traj)/10);

Exp_Rate = 10;Time_Span = linspace(0,Time, Frame_No);
Time_Span_Inte = linspace(0, Time, Exp_Rate * Frame_No);

State_Traj_Inte = spline(Time_Span, State_Traj,Time_Span_Inte);
Ctrl_Traj_Inte = spline(Time_Span, Ctrl_Traj,Time_Span_Inte);

%% HRP2 path file generation
Path_File = []; 

filename = ['phase3' num2str(Exp_Rate) 'x.path'];

fid = fopen(filename,'wt');

for i = 1:length(Time_Span_Inte)
    State_Traj_i = State_Traj_Inte(:,i);
    time_i = Time_Span_Inte(i);
    Row_i = Path_File_Each_Row(State_Traj_i, time_i, fid);
    Path_File = [Path_File Row_i];
end

fclose(fid);

end