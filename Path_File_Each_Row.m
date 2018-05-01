function Row_Start = Path_File_Each_Row(State_Traj, time,fid)

% This function is used to generate the format for each row

rIx = State_Traj(1);             rIy = State_Traj(2);             theta = State_Traj(3);
q1 = State_Traj(4);              q2 = State_Traj(5);              q3 = State_Traj(6);
q4 = State_Traj(7);              q5 = State_Traj(8);              q6 = State_Traj(9);
q7 = State_Traj(10);             q8 = State_Traj(11);             q9 = State_Traj(12);
q10 = State_Traj(13);

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