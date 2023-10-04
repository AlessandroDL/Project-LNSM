function [ H ] = buildJacobianMatrixH(parameters,UE,AP)
%% compute the distance between UE and APs
distanceUEAP = zeros(6,1);
for a=1:6
    distanceUEAP(a) = sqrt( sum( [UE - AP(a,:)].^2) ); 
end
directionCosineMainX =( -AP(2,1) + UE(1) ) / distanceUEAP(2);
directionCosineMainY =( -AP(2,2) + UE(2) ) / distanceUEAP(2);
directionCosineMainZ =( -AP(2,3) + UE(3)) / distanceUEAP(2);

H = zeros( parameters.numberOfAP-1 , 3 );
%% evaluate direction cosine
for a = 1:parameters.numberOfAP
    directionCosineX = ( -AP(a,1) + UE(1)) / distanceUEAP(a);
    directionCosineY = ( -AP(a,2) + UE(2)) / distanceUEAP(a);
    directionCosineZ = ( -AP(a,3) + UE(3)) / distanceUEAP(a);

    if(a<2)
        H(a,1) = +directionCosineMainX -directionCosineX;
        H(a,2) = +directionCosineMainY -directionCosineY;
        H(a,3) = +directionCosineMainZ -directionCosineZ;
    elseif(a>2)
        H(a-1,1) = +directionCosineMainX -directionCosineX;
        H(a-1,2) = +directionCosineMainY -directionCosineY;
        H(a-1,3) = +directionCosineMainZ -directionCosineZ;
    end

end

end