function [position, velocity, acceleration, time] = load_trajectory(path)

load(fullfile(path,'time.mat'), 'time')
load(fullfile(path,'pos.mat'), 'position')
load(fullfile(path,'vel.mat'), 'velocity')
load(fullfile(path,'acc.mat'), 'acceleration')

end

