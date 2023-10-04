function plotEllipse( parameters, ellipsePoints , UE)
x_ell = ellipsePoints( 1 , : );
y_ell = ellipsePoints( 2 , : );

%fig = figure(); 
hold on
fig.WindowState = 'maximized';
% patch( [parameters.xmin parameters.xmax parameters.xmax parameters.xmin parameters.xmin],[parameters.ymin parameters.ymin parameters.ymax parameters.ymax parameters.ymin], [54 114 91]./255 , 'FaceAlpha', .1)
%plot( UE(1) , UE(2) , 'o','MarkerSize',10,'MarkerEdgeColor',[0.30,0.75,0.93],'MarkerFaceColor' , [0.30,0.75,0.93])
fill( x_ell , y_ell , [.9 .95 1],'edgecolor',[0, 0.4470, 0.7410],'linewidth',2);alpha(.5)

grid on
box on
axis equal


end