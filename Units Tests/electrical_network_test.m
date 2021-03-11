% Script to test Falsone decentralised optimisation approach

close all;
clear all;
yalmip clear;

% Data and simulation paramters
%prediction horizon
N = 12;

%load data
dataset = 'USAALAuburnOpelika';
datatable = load('USAALAuburnOpelika',dataset);
datatable = datatable.(dataset);

%number of hubs
nHubs = 3;

%simulation length
sim_length = 1;

% Load instantiation
for i = 1:nHubs
    %electricity load
    edName = strcat('ed',num2str(i));
    eds.(matlab.lang.makeValidName(edName)) = FixedDemandModel(edName,N);
end

% Network instantiation
bigM = 20;
lineImp = complex(0.003,0.006); %arbitrary values
adjMatrix = [   0,  lineImp,    0,          0;
                0,  0,          lineImp,    0;
                0,  0,          0,          lineImp;
                0,  0,          0,          0];
v1 = 1;
PF = 0.9*ones(nHubs+1,1); %arbitrary values
eNet = ElecNetworkModel('eNet',N,bigM,adjMatrix,v1,PF,[]);
eNet.add_val_cost('eNet_y_p1');
for i = 2:size(adjMatrix,1)
    eNet.change_bounds(strcat('eNet_y_v',num2str(i)),0.9^2,1.1^2);
end

% District instantiation
D = Aggregation('D',N,eds.ed1,eds.ed2,eds.ed3,eNet);
D.connect('eNet_y_p2','ed1_y',-10) %arbitrary conversion factor
D.connect('eNet_y_p3','ed2_y',-10)
D.connect('eNet_y_p4','ed3_y',-10)

% initialise controller
ops = sdpsettings('solver','GUROBI','verbose',2,'gurobi.MIPGap',1e-4,'savesolveroutput',1);

inputNames = [D.w;D.l];
inputVars = D.get_vars(inputNames);

outputNames = [D.y;D.z];
outputVars = D.get_vars(outputNames);

MPC = optimizer(D.C,D.J,ops,inputVars,outputVars);

% Input parameters
%demand and generation forecasts
ed_f = datatable{1:nHubs*N,'ElectricityFacilitykWHourly'};

%price forecasts
eBuy_f = datatable{1:N,'eBuy'};

inputs = {};

% Simulation loop
for k = 1:sim_length
    
    for i = 1:nHubs
        input_load_e = ed_f(N*(i-1)+1:N*i);
        inputs = [inputs,input_load_e];
    end
    
    ecm_P_e1_cost = eBuy_f(1:N);
    inputs = [inputs,ecm_P_e1_cost];
    
    [solution,diagnostics,infostr,dual,output] = MPC(inputs);
    
    if diagnostics > 0
        keyboard
    end
    
    %iterate inputs
    ed_f = circshift(ed_f,-1);
    eBuy_f = circshift(eBuy_f,-1);
    
end

% tabulate results
results = table(solution{:},'variablenames',outputNames');

% check exactness of constraints
check12 = results{:,'eNet_y_v1'}.*results{:,'eNet_z_I1_2'} - (results{:,'eNet_z_P1_2'}.^2 + results{:,'eNet_z_Q1_2'}.^2);
check23 = results{:,'eNet_y_v2'}.*results{:,'eNet_z_I2_3'} - (results{:,'eNet_z_P2_3'}.^2 + results{:,'eNet_z_Q2_3'}.^2);
check34 = results{:,'eNet_y_v3'}.*results{:,'eNet_z_I3_4'} - (results{:,'eNet_z_P3_4'}.^2 + results{:,'eNet_z_Q3_4'}.^2);