%%
mdl = 'Drone_PID_rev5_Manual_PID_RL';
open_system(mdl)

Ts = 1;
Tf = 100;

[env,obsInfo,actInfo] = localCreatePIDEnv(mdl);

numObs = obsInfo.Dimension(1);
numAct = prod(actInfo.Dimension);

rng(0)

% Specify the number of outputs for the hidden layers.
hiddenLayerSize = 500; 

observationPath = [
    featureInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc2')];
actionPath = [
    featureInputLayer(numAct,'Normalization','none','Name','action')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc3')];
commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

% Create the layer graph.
criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,observationPath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
% Connect actionPath to observationPath.
criticNetwork = connectLayers(criticNetwork,'fc2','add/in1');
criticNetwork = connectLayers(criticNetwork,'fc3','add/in2');
% figure
% plot(criticNetwork)

criticOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);

critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

actorNetwork = [
    featureInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(hiddenLayerSize,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(numAct,'Name','fc3')
    regressionLayer('Name', 'actions')];

actorOptions = rlRepresentationOptions('LearnRate',1e-05,'GradientThreshold',1);

actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'fc3'},actorOptions);

agentOptions = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6 ,...
    'DiscountFactor',1,...
    'MiniBatchSize',64);
agentOptions.NoiseOptions.Variance = 0.3;
agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;

agent = rlDDPGAgent(actor,critic,agentOptions);

maxepisodes = 100;
maxsteps = ceil(Tf/Ts);
trainingOptions = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes,...
    'MaxStepsPerEpisode',maxsteps,...
    'StopOnError',"on",...
    'Verbose',false,...
    'Plots',"training-progress",...
    'StopTrainingCriteria',"AverageReward",...
    'StopTrainingValue',2500,...
    'ScoreAveragingWindowLength',20,...
    'SaveAgentCriteria',"AverageReward",...
    'SaveAgentValue',1000); 

trainingStats = train(agent,env,trainingOptions);

function [env,obsInfo,actInfo] = localCreatePIDEnv(mdl)

% Define the observation specification obsInfo and action specification actInfo.
obsInfo = rlNumericSpec([3 1]);
obsInfo.Name = 'Observations';
obsInfo.Description = 'Error, Integrated Error and Derivated Error';

actInfo = rlNumericSpec([3 1]);
actInfo.Name = 'Actions';
actInfo.LowerLimit = -10;
actInfo.UpperLimit = 0;

% Build the environment interface object.
env = rlSimulinkEnv(mdl,[mdl '/Controller/Altitude Controller/Z-Controller (Auto-Tune)/RL Agent'],obsInfo,actInfo);
end