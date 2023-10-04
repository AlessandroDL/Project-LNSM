function plotScenario( parameters , AP)

nAP = parameters.numberOfAP;
xmin = parameters.xmin;
ymin = parameters.ymin;
xmax = parameters.xmax;
ymax = parameters.ymax;
zmin = parameters.zmin;
zmax = parameters.zmax;

figure();
grid on;
for a = 1:parameters.numberOfAP
    % if parameters.numberOfAP>2
    %     subplot(round(parameters.numberOfAP/2),round(parameters.numberOfAP/2),a)
    % else
    %     subplot(2,1,a)
    % end
    hold on
    plot3( AP(a,1) , AP(a,2), AP(a, 3), '^');
end



end
