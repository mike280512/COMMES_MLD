%% Script to test the functionality of COMMES-MLD
clear all
yalmip clear

%prediction horizon
N = 24;

%load data
dataset = 'USAALAuburnOpelika';
datatable = load('USAALAuburnOpelika',dataset);
datatable = datatable.(dataset);

%% initialise devices
%transformer
etaTx = 0.98;
bigM = 20;

Tx = TransformerModel('Tx',N,bigM,etaTx);
Tx.add_val_cost('Tx_y_in');

%battery
bigM = 10;
alpha = 0.999;
dT = 0.5;
etaChg = 0.98;
etaDchg = 1/etaChg;

batt = BatteryModel('batt',N,bigM,alpha,dT,etaChg,etaDchg);
batt.change_bounds('batt_x',0,20);

%% initialise loads
ed = FixedDemandModel('ed',N);

%% initialise electrical power balance
bigM = 20;
Nsnk = 1;
Nsrc = 2;

eBal = PowerBalanceModel('eBal',N,bigM,Nsnk,Nsrc);

%% initialise hub
hub = Aggregation('hub',N,eBal,batt,ed,Tx);

%connect ports
hub.connect('Tx_y_out','eBal_y_snk1',1);
hub.connect('eBal_y_src1','batt_y_chg',1);
hub.connect('eBal_y_src2','ed_y',1);

%% initialise controller
ops = sdpsettings('solver','GUROBI','verbose',2,'gurobi.MIPGap',1e-4,'savesolveroutput',1);

inputNames = [hub.m;hub.w;hub.l];
inputVars = hub.get_vars(inputNames);

outputNames = [hub.uc;hub.ud;hub.y;hub.x];
outputVars = hub.get_vars(outputNames);

MPC = optimizer(hub.C,hub.J,ops,inputVars,outputVars);

%% run controller to test feasibility
batt_m_x0 = 20;
ed_f =  datatable{1:N,'ElectricityFacilitykWHourly'};
eBuy_f = datatable{1:N,'eBuy'};

[solution,diagnostics,infostr,~,output] = MPC({batt_m_x0,ed_f,eBuy_f});

%% plot results
idx = false(1,length(outputNames));
for i = 1:length(outputNames)
    if any(strcmp(hub.y,outputNames{i}))
        idx(i) = true;
    end
end
results = table(solution{idx},'VariableNames',hub.y');

figure; stairs(1:N,results{1:N,'batt_y_chg'});