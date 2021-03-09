%% Script to test COMMES MLD functionality

close all;
clear all;
yalmip clear;

%% Data and simulation paramters
%prediction horizon
N = 24;

%load data
dataset = 'USAALAuburnOpelika';
datatable = load('USAALAuburnOpelika',dataset);
datatable = datatable.(dataset);

%number of hubs
nHubs = 3;

%simulation length
sim_length = 1;

%% Load instantiation
for i = 1:nHubs
    %electricity load
    gdName = strcat('gd',num2str(i));
    gds.(matlab.lang.makeValidName(gdName)) = FixedDemandModel(gdName,N);
end

%% Network instantiation
bigM = 100;
pipeK = 1e-1;
adjMatrix = [   0,  pipeK,      0,          0;
                0,  0,          pipeK,      0;
                0,  0,          0,          pipeK;
                0,  0,          0,          0];
pr1 = 75;
penalty = 0.01;
engDen = 38.7;
gNet = GasNetworkModel('gNet',N,bigM,adjMatrix,pr1,penalty);
gNet.add_val_cost('gNet_y_f1');
for i = 2:size(adjMatrix,1)
    gNet.change_bounds(strcat('gNet_y_pr',num2str(i)),1,75);
end

%% District instantiation
D = Aggregation('D',N,gds.gd1,gds.gd2,gds.gd3,gNet);
D.connect('gNet_y_f2','gd1_y',-engDen)
D.connect('gNet_y_f3','gd2_y',-engDen)
D.connect('gNet_y_f4','gd3_y',-engDen)

%% initialise controller
ops = sdpsettings('solver','GUROBI','verbose',2,'gurobi.MIPGap',1e-4,'savesolveroutput',1);

inputNames = [D.w;D.l];
inputVars = D.get_vars(inputNames);

outputNames = [D.y;D.z;D.d];
outputVars = D.get_vars(outputNames);

MPC = optimizer(D.C,D.J,ops,inputVars,outputVars);

%% Input parameters
%demand and generation forecasts
gd_f = datatable{1:nHubs*N,'HeatingGaskWHourly'};

%price forecasts
gBuy_f = datatable{1:N,'eBuy'};

inputs = {};

%% Simulation loop
for k = 1:sim_length
    
    for i = 1:nHubs
        input_load_g = gd_f(N*(i-1)+1:N*i);
        inputs = [inputs,input_load_g];
    end
    
    ecm_P_g1_cost = gBuy_f(1:N);
    inputs = [inputs,ecm_P_g1_cost];
    
    [solution,diagnostics,infostr,dual,output] = MPC(inputs);
    
    if diagnostics > 0
        keyboard
    end
    
    %iterate inputs
    gd_f = circshift(gd_f,-1);
    gBuy_f = circshift(gBuy_f,-1);
    
end

%% tabulate results
results = table(solution{:},'variablenames',outputNames');

%% check exactness of constraints
check12 = results{:,'gNet_z_dp1_2'} - pipeK*results{:,'gNet_z_F1_2'}.^2;