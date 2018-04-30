function stop = Time_Termination_fn(x, optimValues, state)
global time_count

stop = false;
Elapse_Min = 4 * 60;
if etime(clock, time_count)>Elapse_Min
    stop = true;
end
end

