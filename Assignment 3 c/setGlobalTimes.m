function setGlobalTimes(time)
    global times;
    times = [times, time];
    global steps;
    steps = [steps, getGlobalStep];
end