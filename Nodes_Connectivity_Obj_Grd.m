function [f, gradf] = Nodes_Connectivity_Obj_Grd(x)
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

if nargout == 1
    f = Nodes_Connectivity_Obj(x);
else
    gx.f = x;
    gx.dx = ones(721,1);
    obj = Nodes_Connectivity_Obj_ADiGatorGrd(gx);
    f = obj.f;
gradf = zeros(721,1);
gradf(obj.dx_location) = obj.dx;
end
