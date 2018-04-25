function Keyframe_Interpolater()
load('7thrun.mat');
% This function is used to interpolate the given keyframes to a smooth traj
Var_Opt = z
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

filename = ['hrp2_7_' num2str(Exp_Rate) 'x.path'];

fid = fopen(filename,'wt');

for i = 1:length(Time_Span_Inte)
    State_Traj_i = State_Traj_Inte(:,i);
    time_i = Time_Span_Inte(i);
    Row_i = Path_File_Each_Row(State_Traj_i, time_i, fid);
    Path_File = [Path_File Row_i];
end

fclose(fid);

end

function Row_Start = Path_File_Each_Row(State_Traj, time,fid)

% This function is used to generate the format for each row

rIx = State_Traj(1);             rIy = State_Traj(2);             theta = State_Traj(3);
q1 = State_Traj(4);              q2 = State_Traj(5);              q3 = State_Traj(6);
q4 = State_Traj(7);              q5 = State_Traj(8);              q6 = State_Traj(9);
q7 = State_Traj(10);             q8 = State_Traj(11);             q9 = State_Traj(12);
q10 = State_Traj(13);

% rIxdot = State_Traj(1+13);          rIydot = State_Traj(2+13);          thetadot = State_Traj(3+13);
% q1dot = State_Traj(4+13);           q2dot = State_Traj(5+13);           q3dot = State_Traj(6+13);
% q4dot = State_Traj(7+13);           q5dot = State_Traj(8+13);           q6dot = State_Traj(9+13);
% q7dot = State_Traj(10+13);          q8dot = State_Traj(11+13);          q9dot = State_Traj(12+13);
% q10dot = State_Traj(13+13);

% Row_Start = ['36\t' num2str(rIx) ' 0 ' num2str(rIy) ' 0 ' num2str(theta) ' 0 ']; % Up to link 5
Row_Start = [num2str(time) '\t36\t' num2str(rIx) ' 0.0 ' num2str(rIy) ' 0.0 ' num2str(theta) ' 0.0 ']; % Up to link 5
Row_Start = [Row_Start '0.0 ' '0.0 ' num2str(q1) ' ' num2str(q2) ' ' num2str(q3)]; % Start from link 6 to link 10
Row_Start = [Row_Start ' 0.0 ' '0.0 ' '0.0 '];% Link 11 to Link 13
Row_Start = [Row_Start num2str(q4) ' ' num2str(q5) ' ' num2str(q6)]; % Link 14 to Link 16
Row_Start = [Row_Start ' 0.0 ' '0.0 ' '0.0 ' '0.0 ' '0.0 '];% Link 17 to 21
Row_Start = [Row_Start num2str(q7) ' 0' ' 0 ' num2str(q8)]; % Link 22 25
Row_Start = [Row_Start ' 0.0' ' 0.0 ' '0.0 ' num2str(q9)]; % Link 26 to 29
Row_Start = [Row_Start ' 0.0' ' 0.0 ' num2str(q10)]; % Link 30 to 32
Row_Start = [Row_Start ' 0.0' ' 0.0' ' 0.0'];

fprintf(fid,Row_Start);
fprintf(fid,' \n');

end

