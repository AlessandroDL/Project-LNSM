function [ h ] = measurementModel(parameters,UE,AP)

%% compute the distance between UE and APs
distanceUEAP = zeros(6,1);
for a=1:6
    distanceUEAP(a) = sqrt( sum( [UE - AP(a,:)].^2) ); 
end

%% build the vector/matrix of observation
h = zeros( 1 , parameters.numberOfAP-1 );

for a = 1:parameters.numberOfAP    
    if(a<2)
        h(a) = - distanceUEAP(a) + distanceUEAP(2);
    elseif(a>2)
        h(a-1) = - distanceUEAP(a) + distanceUEAP(2);
    
    end

end

end