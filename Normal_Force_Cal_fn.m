function Normal_Force = Normal_Force_Cal_fn(End_Normal_Ang, Contact_Force)

% This function is used to calculate the normal force at a given surface
Normal_Force = zeros(length(Contact_Force)/2,1);
for i = 1:length(Contact_Force)/2    
    End_Normal_Ang_i = End_Normal_Ang(i);
    End_Normal_Ang_i_unit = [cos(End_Normal_Ang_i) sin(End_Normal_Ang_i)]';
    Contact_Force_i = Contact_Force(2*i-1:2*i,:);
    Normal_Force(i) = dot(End_Normal_Ang_i_unit, Contact_Force_i);
end
end

