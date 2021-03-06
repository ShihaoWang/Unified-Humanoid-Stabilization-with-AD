function Hes = Init_Obj_Hes(x,lambda)
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

global ADiGator_Init_Cons_ADiGatorHes
gx.f = x;
gx.dx = ones(26,1);
obj = Init_Obj_ADiGatorHes(gx);
objHes = sparse(obj.dxdx_location(:,1),obj.dxdx_location(:,2),obj.dxdx,26,26);
[con, coneq] = Init_Cons_ADiGatorHes(gx);
HesData1 = ADiGator_Init_Cons_ADiGatorHes.Init_Cons_ADiGatorHes.Gator2Data.HesData1;
conHesnz = lambda.ineqnonlin.'*sparse(con.dxdx_location(:,1),HesData1,con.dxdx,8,32);
HesData2 = ADiGator_Init_Cons_ADiGatorHes.Init_Cons_ADiGatorHes.Gator2Data.HesData2;
conHes = sparse(HesData2(:,1),HesData2(:,2),conHesnz,26,26);
HesData3 = ADiGator_Init_Cons_ADiGatorHes.Init_Cons_ADiGatorHes.Gator2Data.HesData3;
coneqHesnz = lambda.eqnonlin.'*sparse(coneq.dxdx_location(:,1),HesData3,coneq.dxdx,24,48);
HesData4 = ADiGator_Init_Cons_ADiGatorHes.Init_Cons_ADiGatorHes.Gator2Data.HesData4;
coneqHes = sparse(HesData4(:,1),HesData4(:,2),coneqHesnz,26,26);
Hes = objHes+conHes+coneqHes;
Hes = (Hes+Hes.')/2;