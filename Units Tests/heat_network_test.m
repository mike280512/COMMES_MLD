% Script to test COMMES MLD functionality

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
    hdName = strcat('hd',num2str(i));
    hds.(matlab.lang.makeValidName(hdName)) = FixedDemandModel(hdName,N);
end

% Network instantiation
bigM = 100;
pipeC = 1e-3; %arbitrary value
adjMatrix = [   0,  pipeC,      0,          0;
                0,  0,          pipeC,      0;
                0,  0,          0,          pipeC;
                0,  0,          0,          0];

hexCs = 1e-3*ones(nHubs+1,1); %arbitrary values
penalty = 0.01; %penalty for relaxation inexactness
hNet = HeatNetworkModel('hNet',N,bigM,adjMatrix,hexCs,penalty);
hNet.add_val_cost('hNet_y_phi1');
for i = 1:size(adjMatrix,1)
    hNet.change_bounds(strcat('hNet_y_pumpDH',num2str(i)),0,100);
end

% District instantiation
D = Aggregation('D',N,hds.hd1,hds.hd2,hds.hd3,hNet);
D.connect('hNet_y_phi2','hd1_y',-1)
D.connect('hNet_y_phi3','hd2_y',1)
D.connect('hNet_y_phi4','hd3_y',-1)

% initialise controller
ops = sdpsettings('solver','GUROBI','verbose',2,'gurobi.MIPGap',1e-4,'savesolveroutput',1);

inputNames = [D.w;D.l];
inputVars = D.get_vars(inputNames);

outputNames = [D.y;D.z;D.d];
outputVars = D.get_vars(outputNames);

MPC = optimizer(D.C,D.J,ops,inputVars,outputVars);

% Input parameters
%demand and generation forecasts
hd_f = datatable{1:nHubs*N,'HeatingGaskWHourly'};

%price forecasts
hBuy_f = datatable{1:N,'eBuy'};

inputs = {};

% Simulation loop
for k = 1:sim_length
    
    for i = 1:nHubs
        input_load_h = hd_f(N*(i-1)+1:N*i);
        inputs = [inputs,input_load_h];
    end
    
    ecm_P_h1_cost = hBuy_f(1:N);
    inputs = [inputs,ecm_P_h1_cost];
    
    [solution,diagnostics,infostr,dual,output] = MPC(inputs);
    
    if diagnostics > 0
        keyboard
    end
    
    %iterate inputs
    hd_f = circshift(hd_f,-1);
    hBuy_f = circshift(hBuy_f,-1);
    
end

% tabulate results
results = table(solution{:},'variablenames',outputNames');

% check exactness of constraints
check12 = results{:,'hNet_z_pipeDH1_2'} - pipeC*results{:,'hNet_z_Phi1_2'}.^2;
check23 = results{:,'hNet_z_pipeDH2_3'} - pipeC*results{:,'hNet_z_Phi2_3'}.^2;
check34 = results{:,'hNet_z_pipeDH3_4'} - pipeC*results{:,'hNet_z_Phi3_4'}.^2;