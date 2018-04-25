function Single_Frame_Plot(q_array_i, P, axes_plot)

% This function plot the robot configuration given the current q_array
rIx = q_array_i(1);    rIy = q_array_i(2);   theta = q_array_i(3);
q1 = q_array_i(4);     q2 = q_array_i(5);    q3 = q_array_i(6);
q4 = q_array_i(7);     q5 = q_array_i(8);    q6 = q_array_i(9);
q7 = q_array_i(10);     q8 = q_array_i(11);    q9 = q_array_i(12);
q10 = q_array_i(13);   

rA = P.rA_fn(q1,q2,q3,rIx,rIy,theta);
rB = P.rB_fn(q1,q2,q3,rIx,rIy,theta);
rC = P.rC_fn(q4,q5,q6,rIx,rIy,theta);
rD = P.rD_fn(q4,q5,q6,rIx,rIy,theta);
rE = P.rE_fn(q7,q8,rIx,rIy,theta);
rF = P.rF_fn(q9,q10,rIx,rIy,theta);
rG = P.rG_fn(q1,q2,rIx,rIy,theta);
rH = P.rH_fn(q1,rIx,rIy,theta);
rI = P.rI_fn(rIx,rIy);
rJ = P.rJ_fn(q4,q5,rIx,rIy,theta);
rK = P.rK_fn(q4,rIx,rIy,theta);
rL = P.rL_fn(rIx,rIy,theta);
rM = P.rM_fn(q7,rIx,rIy,theta);
rN = P.rN_fn(q9,rIx,rIy,theta);
rT = P.rT_fn(rIx,rIy,theta);

r_vec = [rA; rB; rC; rD; rE; rF; rG; rH; rI; rJ; rK; rL; rM; rN; rT];
if nargin <3 
    axes_plot = axes;
end
link_plot_2points('AB',r_vec,axes_plot,'r');
link_plot_2points('AG',r_vec,axes_plot,'r');
link_plot_2points('GB',r_vec,axes_plot,'r');
link_plot_2points('GH',r_vec,axes_plot,'r');
link_plot_2points('HI',r_vec,axes_plot,'r');
link_plot_2points('IK',r_vec,axes_plot,'l');
link_plot_2points('KJ',r_vec,axes_plot,'l');
link_plot_2points('JC',r_vec,axes_plot,'l');
link_plot_2points('JD',r_vec,axes_plot,'l');
link_plot_2points('CD',r_vec,axes_plot,'l');
link_plot_2points('IT',r_vec,axes_plot,'m');
link_plot_2points('LN',r_vec,axes_plot,'l');
link_plot_2points('NF',r_vec,axes_plot,'l');
link_plot_2points('LM',r_vec,axes_plot,'r');
link_plot_2points('ME',r_vec,axes_plot,'r');

MarkSize_Coef = 5;
MarkerEdgeColor_Coef = [28 148 149] ./ 255;
MarkerFaceColor_Coef = [225 9 92] ./ 255;

% Edge point plots
for i = 1:14
    r_Point = strcat('r',char(i + 64));
    r_Point_Var = genvarname(r_Point);
    evalc(['r_Point_Plot', ' = ', r_Point_Var]);
    plot(axes_plot, r_Point_Plot(1),r_Point_Plot(2),'o','MarkerSize',MarkSize_Coef,...
        'MarkerEdgeColor',MarkerEdgeColor_Coef,...
        'MarkerFaceColor',MarkerFaceColor_Coef);  hold on;
    if i == 14
        r_Point = 'rT';
        r_Point_Var = genvarname(r_Point);
        evalc(['r_Point_Plot', ' = ', r_Point_Var]);
        plot(axes_plot, r_Point_Plot(1),r_Point_Plot(2),'o','MarkerSize',MarkSize_Coef,...
            'MarkerEdgeColor',MarkerEdgeColor_Coef,...
            'MarkerFaceColor',MarkerFaceColor_Coef);  hold on;
    end
end

axis(axes_plot, 'equal');
hold(axes_plot,'off');

end

function link_plot_2points(Edges,r_vec,axes_plot,side)

left_color = [24 193 8] ./ 255;
midl_color = [255 24 22] ./ 255;
rght_color = [8 128 224] ./ 255;
Linewidth_Coef = 2.5;

for i = 1:14  
    r_Point = strcat('r',char(i + 64));
    r_Point_Var = genvarname(r_Point);
    evalc([r_Point_Var, ' = ', 'r_vec(2*i-1:2*i,:)']);
end
rT = r_vec(end-1:end,:);

evalc(['Edge1', ' = ', strcat('r',Edges(1))]);
evalc(['Edge2', ' = ', strcat('r',Edges(2))]);

if nargin<3
    plot(axes_plot, [Edge1(1),Edge2(1)]',[Edge1(2),Edge2(2)]','LineWidth',Linewidth_Coef, 'color', left_color);
    hold(axes_plot,'on');
else
    if side =='l'
        plot(axes_plot, [Edge1(1),Edge2(1)]',[Edge1(2),Edge2(2)]','LineWidth',Linewidth_Coef, 'color', left_color);
    else
        if side == 'r'
            plot(axes_plot, [Edge1(1),Edge2(1)]',[Edge1(2),Edge2(2)]','LineWidth',Linewidth_Coef, 'color', rght_color);
        else
             plot(axes_plot, [Edge1(1),Edge2(1)]',[Edge1(2),Edge2(2)]','LineWidth',Linewidth_Coef, 'color', midl_color);       
        end
    end
end
hold(axes_plot,'on');
% axes_plot.Visible = 'off';

end