function plot3Dlikelihood(parameters, x , y , z , likelihood)

%figure();
for a = 1:parameters.numberOfAP
    if parameters.numberOfAP>2
        subplot(round(parameters.numberOfAP/2),round(parameters.numberOfAP/2),a)
    else
        subplot(2,1,a)
    end
    %surf(  x, y, squeeze(likelihood(a,:,:,1))' ),hold on
    surf(  x, y, squeeze(likelihood(a,:,:))' ),hold on
    shading flat
    colorbar;

    % xlabel('[m]'), ylabel('[m]');
    % plot3( AP(:,1) , AP(:,2) ,0.01+zeros(1,size(AP,1)), '^','MarkerSize',10,'MarkerEdgeColor',[0.64,0.08,0.18],'MarkerFaceColor',[0.64,0.08,0.18] )
    % plot3( AP(a,1) , AP(a,2) ,0.01, '^','MarkerSize',10,'MarkerEdgeColor',[102,254,0]./255,'MarkerFaceColor',[102,254,0]./255 )
    % plot3( UE(:,1) , UE(:,2) ,0.01, 'o','MarkerSize',10,'MarkerEdgeColor',[0.30,0.75,0.93],'MarkerFaceColor',[0.30,0.75,0.93] )
    % 

    title(['Likelihood ','tdoa',', ${AP}$ = 1-',num2str(a),' , $\sigma $ = ',num2str(parameters.sigmaTDOA),' m '],'Interpreter','Latex')
    

end



end