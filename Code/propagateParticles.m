function PR = propagateParticles(parameters,PR)

sigma_v =0.1; %m/s

%motion model
PR.samples = PR.samples + parameters.samplingTime*sigma_v*randn(3,parameters.numberOfParticles);
PR.weights = ones(1,parameters.numberOfParticles) ./ parameters.numberOfParticles;

end