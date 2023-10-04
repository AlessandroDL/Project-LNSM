function plotEvaluationPoints(x,y,z)
%PLOTEVALUATIONPOINTS Summary of this function goes here
%   Detailed explanation goes here
for i = 1:1:length(x)
        for j = 1:1:length(y)
            for k = 1:1:length(z)
                plot3(x(i),y(j), z(k),'-*')
                end
            end %k 
        end %j
    end %i

