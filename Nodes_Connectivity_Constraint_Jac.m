function [c, ceq, gradc, gradceq] = Nodes_Connectivity_Constraint_Jac(x)
% 
% Wrapper file generated by ADiGator
% �2010-2014 Matthew J. Weinstein and Anil V. Rao
% ADiGator may be obtained at https://sourceforge.net/projects/adigator/ 
% Contact: mweinstein@ufl.edu
% Bugs/suggestions may be reported to the sourceforge forums
%                    DISCLAIMER
% ADiGator is a general-purpose software distributed under the GNU General
% Public License version 3.0. While the software is distributed with the
% hope that it will be useful, both the software and generated code are
% provided 'AS IS' with NO WARRANTIES OF ANY KIND and no merchantability
% or fitness for any purpose or application.

if nargout == 2
    [c, ceq] = Nodes_Connectivity_Constraint(x);
else
    gx.f = x;
    gx.dx = ones(541,1);
    [con, coneq] = Nodes_Connectivity_Constraint_ADiGatorJac(gx);
    c = con.f; ceq = coneq.f;
    gradc = sparse(con.dx_location(:,2),con.dx_location(:,1),con.dx,541,210);
    gradceq = sparse(coneq.dx_location(:,2),coneq.dx_location(:,1),coneq.dx,541,1061);
end
