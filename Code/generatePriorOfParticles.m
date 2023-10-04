function [PR] = generatePriorOfParticles(parameters)

NP  = parameters.numberOfParticles;

PR.samples = zeros(3,NP);
PR.weights = zeros(1,NP);

%prior
PR.samples(1,:) = ( rand( [1,NP] ) -0.5 ).*2.*parameters.xmax;
PR.samples(2,:) = ( rand( [1,NP] ) -0.5 ).*2.*parameters.ymax;
PR.samples(3,:) = ( rand( [1,NP] ) -0.5 ).*2.*parameters.zmax;

PR.weights = ones(1,NP)./NP;

end