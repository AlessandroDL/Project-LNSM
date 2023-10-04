clear all  clc, close all
set(0,'DefaultTextFontSize',22)
set(0,'DefaultLineLineWidth',2);
set(0,'DefaultTextInterpreter','latex')
set(0,'DefaultAxesFontSize',16)

%%Loadin data from dataset
dataset = importdata("Project_data.mat");
parameters.AP = dataset.AP;
parameters.rho = dataset.rho;
parameters.numberOfAP = 6;

%%Definition of setting, the room is 6x3.5x2 m^3 
parameters.xmin = 0;
parameters.ymin = 0;
parameters.zmin = 0;
parameters.xmax = 6;
parameters.ymax = 3.5;
parameters.zmax = 2;

%Definition of sampling points each one takes 0.1 m step
x = linspace( parameters.xmin , parameters.xmax , 61);
y = linspace( parameters.ymin , parameters.ymax , 36);
z = linspace (parameters.zmin, parameters.zmax, 21);

%Definition of accuracy of measurement
parameters.sigmaTDOA = 0.05; %m according to slides of UWB measurements





%Cleaning of data
for a=1:4
    
    %compute vector of differences between two consecutive samples
    %find the average mean difference between two consecutive samples and
    %use it as a tolerance treshold to delete outliers 
    diff_vector = zeros(1,length(parameters.rho{a})-1);
    
    
    
    for b=1:5
        %%fill in missing data for each measurement
        parameters.rho{a}(b,:)= fillmissing(parameters.rho{a}(b,:),'linear');


        for j=1:length(parameters.rho{a})-1
            diff_vector(1,j) = abs(parameters.rho{a}(b,j+1)-parameters.rho{a}(b,j));
        end
        %average difference between consecutive samples
        diff_mean = mean(diff_vector(1,:));
        
        %deletion of outliers, index of outlier marked in 'indexes' vector
        indexes = zeros(1,length(parameters.rho{a}));
        for j=1:length(parameters.rho{a})-2
            if ((abs( parameters.rho{a}(b,j+1) - parameters.rho{a}(b,j)) ) > diff_mean) || ((abs( parameters.rho{a}(b,j) - parameters.rho{a}(b,j+2)) ) > diff_mean)
                   %parameters.rho{a}(b,j+1)= NaN;
                   indexes(j)=1;
            end
        end
        for j=1:length(parameters.rho{a})
            if (indexes(j)==1)
                   parameters.rho{a}(b,j)= NaN;
            end
        end




        %putting in paramters the cleaned data
        parameters.rho{a}(b,:) = movmean(fillmissing(parameters.rho{a}(b,:),'linear'), 20);
        


     
     
        
        %visualization of the data, first the real ones then the cleaned
        %ones
        % figure()
        % plot(dataset.rho{a}(b,:))
        % hold on
        % plot(parameters.rho{a}(b,:))

        
    end 


    
       
end




%Localization algorithm using maximum likelihood
for tag=1:4
    %computation of likelihood matrix for all the evaluation points in the
    %scenario
    figure();
    plotScenario(parameters, parameters.AP);
    hold on
for sample=1:length(parameters.rho{tag})
    
    hold on
    likelihood = zeros(parameters.numberOfAP , length(x) , length(y), length(z));
    for a=1:6
        for i = 1:1:length(x)
            for j = 1:1:length(y)
                for k = 1:1:length(z)
                    if(a~=2)
                        if(a<3)
                            likelihood(a,i,j,k) =evaluateLikelihoodTDOA ( parameters , parameters.rho{tag}(a,sample) , parameters.AP(2,:) , parameters.AP(a,:) , [x(i) , y(j), z(k)]);          
                        else   
                            likelihood(a,i,j,k) =evaluateLikelihoodTDOA ( parameters , parameters.rho{tag}(a-1,sample) , parameters.AP(2,:) , parameters.AP(a,:) , [x(i) , y(j), z(k)]);                        
                        end
                    end
                end %k 
            end %j
        end %i
    end %a

    %complete likelihood matrix for all the measurements
    maximumLikelihood = ones(length(x),length(y), length(z));
    for a=1:parameters.numberOfAP
        if(a~=2)
            maximumLikelihood = maximumLikelihood .* squeeze(likelihood(a,:,:,:));
        end
    end

    %finding the index of the maximum likelihood value in the matrix in
    %order to find the position that generates that value
    maxValue = max( maximumLikelihood(:));
    uhat=zeros;
    index = 1;
    for i = 1:1:length(x)
        for j = 1:1:length(y)
            for k = 1:1:length(z)
                if(maximumLikelihood(i,j,k)==maxValue)
                    uhat(1,index) = i;
                    uhat(2,index) = j;
                    uhat(3, index) = k;
                    index= index+1;
                end
                % if(maximumLikelihood(i,j,k) > 0)
                %     plot3(x(i), y(j), z(k),'.');
                % end

            end %k 
        end %j
    end %i
    position(1) = x(uhat(1));
    position(2) = y(uhat(2));
    position(3) = z(uhat(3));

    % if(sample == 1) || (sample == 100) || (sample == 200) || (sample == 300) || (sample == 400)
    %     H = buildJacobianMatrixH(parameters, position, parameters.AP);
    %     calculateEllipse(parameters, H, parameters.sigmaTDOA^2, [position(1), position(2), position(3)]);
    % end
    plot3(position(1), position(2), position(3),'.', 'MarkerSize',30);
    pause(0.5)
end
end




%TRACKING WITH PARTICLE FILTER
for tag=1:4
    plotScenario(parameters, parameters.AP);
    hold on

    %definition of parameters for the particle filter
    parameters.numberOfParticles = 1000;
    parameters.samplingTime =1;
    PR = generatePriorOfParticles(parameters);
    uHat = zeros(length(parameters.rho{tag}),3);

    for time = 1 : length(parameters.rho{tag})
                likelihood = ones(parameters.numberOfParticles,1);
            
                %evaluation of likelihood for each particle
            for index=1:parameters.numberOfParticles
                for a = 1:parameters.numberOfAP
                     if(a~=2)
                            if(a<3)
                                likelihood(index) = likelihood(index).*evaluateLikelihoodTDOA ( parameters , parameters.rho{tag}(a,time) , parameters.AP(2,:) , parameters.AP(a,:) , PR.samples(:,index)');          
                            else   
                                likelihood(index) = likelihood(index).*evaluateLikelihoodTDOA ( parameters , parameters.rho{tag}(a-1,time) , parameters.AP(2,:) , parameters.AP(a,:) , PR.samples(:,index)');                        
                            end
                     end
                end
            end

    %normalization
    likelihood = likelihood./sum(likelihood);
    PR.weights = likelihood';


    %resampling
    indexes = resamplingAlgorithm( PR.weights , parameters.numberOfParticles );
    PR.samples = PR.samples(1:3,indexes);

    %UE estimate as mean of all the particles
    uHat(time,1) = mean(PR.samples(1,:));
    uHat(time,2) = mean(PR.samples(2,:));    
    uHat(time,3) = mean(PR.samples(3,:));




    %Propagation of particles
    PR = propagateParticles(parameters,PR);
    plot3( uHat(time,1), uHat(time,2), uHat(time,3), 'o');
    hold on
    pause(.05)
    end
end





% TRACKING WITH EXTENDED KALMAN FILTER 
   for tag=1:4  

    % Definition of paramters for the extened kalman filter
    R = diag( repmat(parameters.sigmaTDOA.^2 , 1 , parameters.numberOfAP-1));
    sigma_driving = 0.05; %m/s which results in an error of 5 cm in each position estmation
    samplingTime = 0.1; %s
    plotScenario(parameters, parameters.AP);
    hold on


    %initialization
    UE_init = [3,1.75,1];
    plot3(UE_init(1),UE_init(2),UE_init(3),'o');
    UE_init_COV = diag([100, 100, 100]);
    x_hat = NaN( length(parameters.rho{tag}) , 3);
    P_hat = NaN( 3 , 3 , length(parameters.rho{tag} ));

    %using Nearly Constant Velocity model 
    L = [samplingTime  0 0; 
         0 samplingTime 0;
         0 0 samplingTime];

    Q = sigma_driving^2 *( L * L');
    F = [1 0 0; 
         0 1 0 ;
         0 0 1];


    %update over time
    for time = 1 : length(parameters.rho{tag})

        %prediction
        if time == 1
            x_pred = UE_init';
            P_pred = UE_init_COV;
        else
            x_pred = F * x_hat(time-1,:)'; %3x3 x 3x1 = 3x1
            P_pred = F * P_hat(:,:,time-1) *F' + Q; %3x3 x3x3 x 3x3 +3x3 = 3x3
        end
        %computation of Jacobian matrix for each position prediction
        H = buildJacobianMatrixH(parameters, x_pred([1,2,3])', parameters.AP); 

        %update
        G = P_pred * H' * inv(H*P_pred*H' +R); %3x3
        x_hat(time,:) = x_pred + G*(parameters.rho{tag}(:,time) - measurementModel(parameters, x_pred([1,2,3])', parameters.AP)');
        P_hat(:,:,time) = P_pred - G*H*P_pred;
        
        %UE estimate
        position(1) = x_hat(time,1);
        position(2) = x_hat(time,2);
        position(3) = x_hat(time,3);
        
        
        plot3( position(1), position(2), position(3), '.');
        %calculateEllipseEKF(parameters, P_pred ,position );
        %calculateEllipseEKF(parameters, P_hat(:,:,time) ,position );
    end
  end
