% Script to test the functionality of COMMES-MLD
clear all
yalmip clear
close all

%prediction horizon
N = 12;

%load data
dataset = 'USAALAuburnOpelika';
datatable = load('USAALAuburnOpelika',dataset);
datatable = datatable.(dataset);

% initialise devices
%transformer
etaTx = 0.98;
bigM = 20;

tx = TransformerModel('tx',N,bigM,etaTx);
tx.add_val_cost('tx_y_in');

%hp
cop = 3.9;
eer = 3.9;
bigM = 80;

hp = RevAirSourceHPModel('hp',N,bigM,cop,eer);
hp.add_on_off('hp_y_e',0.5,20);

% initialise loads
ed = FixedDemandModel('ed',N);
hd = FixedDemandModel('hd',N);
cd = FixedGenerationModel('cd',N);

% initialise electrical power balance
bigM = 20;
Nsnk = 1;
Nsrc = 2;

eBal = PowerBalanceModel('eBal',N,bigM,Nsnk,Nsrc);

% initialise hub
hub = Aggregation('hub',N,eBal,ed,tx,hd,cd,hp);

%connect ports
hub.connect('tx_y_out','eBal_y_snk1',1);
hub.connect('eBal_y_src1','hp_y_e',1);
hub.connect('eBal_y_src2','ed_y',1);
hub.connect('hp_y_h','hd_y',1);
hub.connect('cd_y','hp_y_c',1);

% initialise controller
ops = sdpsettings('solver','GUROBI','verbose',2,'gurobi.MIPGap',1e-4,'savesolveroutput',1);

inputNames = [hub.m;hub.w;hub.l];
inputVars = hub.get_vars(inputNames);

outputNames = [hub.uc;hub.ud;hub.y;hub.x];
outputVars = hub.get_vars(outputNames);

MPC = optimizer(hub.C,hub.J,ops,inputVars,outputVars);

% run controller to test feasibility
ed_f =  datatable{1:N,'ElectricityFacilitykWHourly'};
hd_f = [datatable{1:6,'HeatingGaskWHourly'};zeros(6,1)];
cd_f = [zeros(6,1);datatable{7:N,'HeatingGaskWHourly'}]; %use heating data for ease
eBuy_f = datatable{1:N,'eBuy'};

[solution,diagnostics,infostr,~,output] = MPC({ed_f,hd_f,cd_f,eBuy_f});

% plot results
idx = false(1,length(outputNames));
for i = 1:length(outputNames)
    if any(strcmp(hub.y,outputNames{i}))
        idx(i) = true;
    end
end
results = table(solution{idx},'VariableNames',hub.y');

% figure; stairs(1:N,results{1:N,'hp_y_e'});
