function calculateEllipseEKF(parameters, P, UE)
% input k is the k-sigma ellipse value (e.g., k=3 sigma)

% CRB


[V,D] = eig(P);

% singular value decomposition

S = sqrt(D);              
semiaxis_x = S(1,1);
semiaxis_y = S(2,2);
semiaxis_z = S(3,3);
 
theta_azimuth= atan2(  V(2,1), V(1,1) );  % orientation of semiaxis x on xy
theta_polar = atan2(V(3,1), V(1,1));% orientation of semiaxis x on xz

% ellipse equation
axisTheta = 0:.01:2*pi;
ellipsePoints(1,:) = semiaxis_x*cos(axisTheta)*cos(theta_azimuth) - semiaxis_y*sin(axisTheta)*sin(theta_azimuth) + UE(1);
ellipsePoints(2,:) = semiaxis_x*cos(axisTheta)*sin(theta_azimuth) + semiaxis_y*sin(axisTheta)*cos(theta_azimuth) + UE(2);

% Plot ellipse
plotEllipse( parameters , ellipsePoints , UE)
%plot3(ellipsePoints(1,:),ellipsePoints(2,:) ,repmat(UE(3), length(ellipsePoints(1,:))))


% ellipsePoints(1,:) = semiaxis_x*cos(axisTheta)*cos(theta_polar) - semiaxis_z*sin(axisTheta)*sin(theta_polar) + UE(1);
% ellipsePoints(3,:) = semiaxis_x*cos(axisTheta)*sin(theta_polar) + semiaxis_z*sin(axisTheta)*cos(theta_polar) + UE(3);
% 
% plot3(ellipsePoints(1,:),repmat(UE(2), length(ellipsePoints(3,:))),ellipsePoints(1,:))
