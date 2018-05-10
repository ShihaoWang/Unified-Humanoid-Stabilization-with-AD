function stop = Time_Termination_Fur_fn(x, optimValues, state)
% global time_count

stop = false;

 if optimValues.constrviolation<0.001
     stop = true;
 end

% Elapse_Min = 3.5 * 60;
% if etime(clock, time_count)>Elapse_Min
%     stop = true;
% end
end

