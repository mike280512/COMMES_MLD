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

tx = TransformerModel('tx',N,bigM,etaTx);
tx.add_val_cost('tx_y_in');

%chp
etaE = 0.33;
etaH = 0.4;
bigM = 20;
minUp = 3;
minDown = 3;
rampUp = 6;
rampDown = 6;

chp = CHPModel('chp',N,bigM,etaE,etaH);
chp.add_val_cost('chp_y_in');
chp.add_on_off('chp_y_in',2,20);
chp.add_min_up_down('chp_y_in',minUp,minDown);
chp.add_ramp('chp_y_in',rampUp,rampDown);

%hp
cop = 3.9;
bigM = 20;

hp = AirSourceHPModel('hp',N,bigM,cop);
hp.add_on_off('hp_y_e',0.5,20);

%% initialise loads
ed = FixedDemandModel('ed',N);

hd = FixedDemandModel('hd',N);

%% initialise electrical power balance
bigM = 20;
Nsnk = 2;
Nsrc = 2;

eBal = PowerBalanceModel('eBal',N,bigM,Nsnk,Nsrc);

%% initialise heat power balance
bigM = 20;
Nsnk = 2;
Nsrc = 1;

hBal = PowerBalanceModel('hBal',N,bigM,Nsnk,Nsrc);

%% initialise hub
hub = Aggregation('hub',N,eBal,hBal,ed,hd,tx,chp,hp);

%connect ports
hub.connect('tx_y_out','eBal_y_snk1',1);
hub.connect('chp_y_e','eBal_y_snk2',1);
hub.connect('eBal_y_src1','ed_y',1);
hub.connect('eBal_y_src2','hp_y_e',1);
hub.connect('chp_y_h','hBal_y_snk1',1);
hub.connect('hp_y_h','hBal_y_snk2',1);
hub.connect('hBal_y_src1','hd_y',1);

%% initialise controller
ops = sdpsettings('solver','GUROBI','verbose',2,'gurobi.MIPGap',1e-4,'savesolveroutput',1);

inputNames = [hub.m;hub.w;hub.l];
inputVars = hub.get_vars(inputNames);

outputNames = [hub.uc;hub.ud;hub.y;hub.x];
outputVars = hub.get_vars(outputNames);

MPC = optimizer(hub.C,hub.J,ops,inputVars,outputVars);

%% run controller to test feasibility
ed_f =  datatable{1:N,'ElectricityFacilitykWHourly'};
hd_f = datatable{1:N,'HeatingGaskWHourly'};
eBuy_f = datatable{1:N,'eBuy'};
gBuy_f = 3*ones(N,1);
chpOn = 0;
minUp = zeros(minUp-1,1);
minDown = zeros(minDown-1,1);
chpOutput = 0;

[solution,diagnostics,infostr,~,output] = MPC({chpOn,minUp,minDown,chpOutput,ed_f,hd_f,eBuy_f,gBuy_f});

%% plot results
idx = false(1,length(outputNames));
for i = 1:length(outputNames)
    if any(strcmp(hub.y,outputNames{i}))
        idx(i) = true;
    end
end
results = table(solution{idx},'VariableNames',hub.y');

figure; stairs(1:N,results{1:N,'chp_y_in'});
hold on
stairs(1:N,results{1:N,'hp_y_e'});
