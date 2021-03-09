% Script to test the functionality of COMMES-MLD
clear all
close all
yalmip clear

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

%chp
etaE = 0.33;
etaH = 0.4;
bigM = 20;
rampUp = 6;
rampDown = 6;

chp = CHPModel('chp',N,bigM,etaE,etaH);
chp.add_val_cost('chp_y_in');
chp.add_on_off('chp_y_in',2,20);
chp.add_ramp('chp_y_in',rampUp,rampDown);

% initialise loads
ed = FixedDemandModel('ed',N);

Nn = 2;
bigM = 15;
hd = PDemandModel('hd',N,bigM,Nn);

% initialise electrical power balance
bigM = 20;
Nsnk = 2;
Nsrc = 1;

eBal = PowerBalanceModel('eBal',N,bigM,Nsnk,Nsrc);

% initialise hub
hub = Aggregation('hub',N,eBal,ed,tx,hd,chp);

%connect ports
hub.connect('tx_y_out','eBal_y_snk1',1);
hub.connect('chp_y_e','eBal_y_snk2',1);
hub.connect('eBal_y_src1','ed_y',1);
hub.connect('chp_y_h','hd_y_L',1);

% initialise logic controller
hd_f = datatable{1:N,'HeatingGaskWHourly'};
Nc = N/Nn; %number of periods per energy consumption cycle (unless baseline consumption satisfied)
hd_f = sum(reshape(hd_f,Nc,length(hd_f)/Nc));
hdLC = PDemandLC(hd,hd_f,Nc); %heat demand

% initialise controller
ops = sdpsettings('solver','GUROBI','verbose',2,'gurobi.MIPGap',1e-4,'savesolveroutput',1);

inputNames = [hub.m;hub.w;hub.l];
inputVars = hub.get_vars(inputNames);

outputNames = [hub.uc;hub.ud;hub.y;hub.x;hub.d];
outputVars = hub.get_vars(outputNames);

MPC = optimizer(hub.C,hub.J,ops,inputVars,outputVars);

% obtain simultation inputs 
ed_f =  datatable{1:N,'ElectricityFacilitykWHourly'};
hd_f = datatable{1:N,'HeatingGaskWHourly'};
eBuy_f = datatable{1:N,'eBuy'};
gBuy_f = 3*ones(N,1);
chp_in_m = 0;

lBase = zeros(1,Nn);
procStatus = zeros(1,Nn);
compStatus = zeros(1,Nn);

% simulation loop
for k = 1: N
    
    hd_m = hdLC(lBase,procStatus,compStatus);
    
    inputs = [hd_m,chp_in_m,ed_f,eBuy_f,gBuy_f];

    [solution,diagnostics,infostr,~,output] = MPC(inputs);
    
    if diagnostics > 0
        keyboard
    end
    
    %iterate inputs
    ed_f = circshift(ed_f,-1);
    eBuy_f = circshift(eBuy_f,-1);
    
    %update from solution
    lBase = solution{strcmp(outputNames,'hd_uc_l')'}(1,:);
    procStatus = solution{strcmp(outputNames,'hd_d_proc')'}(1,:);
    compStatus = solution{strcmp(outputNames,'hd_d_comp')'}(1,:);
    chp_in_m = solution{strcmp(outputNames,'chp_y_in')'}(1,:);
end
