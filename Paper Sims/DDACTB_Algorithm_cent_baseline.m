%% Script to test Falsone centralised optimisation approach 

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
nHubs = 175;

%simulation length
sim_length = 96;

%define conversion effeciencies
etaTx = 0.98; %transformer conversion efficiency
chg_e = 0.9; %charging efficiency (battery)
dchg_e = 0.85; %discharging efficiency (battery)
chp_e = 0.33; %chp electric efficiency
chp_h = 0.4; %chp heating efficiency
COP = 3.9; %hp COP

%fixed battery parameters
alpha = 0.999^(1/4);
dT = 1/4;

%controller options
ops = sdpsettings('solver','GUROBI','verbose',0,'gurobi.MIPGap',1e-4,'savesolveroutput',1);%,'relax',2);

%random number generator seed
rng('default');
rng(1);


%% Hub controller definition
hubs_input = cell(1,nHubs);
for i = 1:nHubs
    tic
    %define randomised properties
    battCap = 6 + unifrnd(-1,1);
    battChg = 2 + unifrnd(-1,1);
    etaChg = chg_e + unifrnd(-1,1)*0.05;
    etaDchg = dchg_e + unifrnd(-1,1)*0.05;
    etaE = chp_e + unifrnd(-1,1)*0.1;
    etaH = chp_h + unifrnd(-1,1)*0.1;
    
    %transformer definition
    txName = strcat('tx',num2str(i));
    bigM = 5;
    tx = TransformerModel(txName,N,bigM,etaTx); %instantsiate transformer
    tx.add_buy_sell_cost(strcat(txName,'_y_in'));
%     tx.add_dual_cost(strcat(txName,'_lambda_e'),1,{strcat(txName,'_y_in')});
    
    %chp definition
    chpName = strcat('chp',num2str(i));
    bigM = 6;
    chp = CHPModel(chpName,N,bigM,etaE,etaH);
    chp.add_val_cost(strcat(chpName,'_y_in'));
    
    %hp definition
    hpName = strcat('hp',num2str(i));
    bigM = 20;
    hp = AirSourceHPModel(hpName,N,bigM,COP);
    
    %electricity load
    edName = strcat('ed',num2str(i));
    ed = FixedDemandModel(edName,N);
    
    %pliable heat demand
    hdName = strcat('hd',num2str(i));
    hd = FixedDemandModel(hdName,N);
    
    %battery storage model
    battName = strcat('batt',num2str(i));
    bigM = 7;
    batt = BatteryModel(battName,N,bigM,alpha,dT,etaChg,etaDchg);
    batt.change_bounds(strcat(battName,'_x'),battCap*0.2,battCap*0.8);
    batt.change_bounds(strcat(battName,'_y_chg'),-battChg,battChg);
    batt.add_abs_cost(strcat(battName,'_y_chg')); %battery degradation cost
    
    %electrical power balance
    eBalName = strcat('eBal',num2str(i));
    bigM = 20;
    Nsnk = 2;
    Nsrc = 3;
    eBal = PowerBalanceModel(eBalName,N,bigM,Nsnk,Nsrc);
    
    %heat power balance
    hBalName = strcat('hBal',num2str(i));
    bigM = 20;
    Nsnk = 2;
    Nsrc = 1;
    hBal = PowerBalanceModel(hBalName,N,bigM,Nsnk,Nsrc);
    
    %aggregated hub
    hubName = strcat('hub',num2str(i));
    hubs.(matlab.lang.makeValidName(hubName)) = Aggregation(hubName,N,tx,chp,hp,ed,hd,batt,eBal,hBal);
    hub = hubs.(matlab.lang.makeValidName(hubName));
    
    hub.connect(strcat(txName,'_y_out'),strcat(eBalName,'_y_snk1'),1);
    hub.connect(strcat(chpName,'_y_e'),strcat(eBalName,'_y_snk2'),1);
    hub.connect(strcat(eBalName,'_y_src1'),strcat(hpName,'_y_e'),1);
    hub.connect(strcat(eBalName,'_y_src2'),strcat(battName,'_y_chg'),1);
    hub.connect(strcat(eBalName,'_y_src3'),strcat(edName,'_y'),1);
    
    hub.connect(strcat(chpName,'_y_h'),strcat(hBalName,'_y_snk1'),1);
    hub.connect(strcat(hpName,'_y_h'),strcat(hBalName,'_y_snk2'),1);
    hub.connect(strcat(hBalName,'_y_src1'),strcat(hdName,'_y'),1);
    
    hubs_input{i} = hub;
    
    fprintf('hub %i complete \n', i);
    toc
end

%% Network model
%electrical network
%define adjacency matrix

adjMatrix = zeros(nHubs+2);
adjMatrix(1,2) = 1;
for i = 3:nHubs+2
    adjMatrix(2,i) = 1;
end

bigM = 260;

ENet = NetworkBalanceModel('ENet',N,bigM,adjMatrix);
ENet.change_bounds('ENet_y_1',0,260);
ENet.change_bounds('ENet_y_2',0,0);

%% District model
hubsArg = cell(1,nHubs);
for i = 1:nHubs
    hubName = strcat('hub',num2str(i));
    hubsArg{i} = hubs.(matlab.lang.makeValidName(hubName));
end
Dist = Aggregation('Dist',N,ENet,hubsArg{:});

for i = 1:nHubs
    hubConName = strcat('tx',num2str(i),'_y_in');
    netConName = strcat('ENet_y_',num2str(i+2));
    Dist.connect(netConName,hubConName,-1);
end

%% Controller
inputNames = [Dist.m;Dist.w;Dist.l];
inputVars = Dist.get_vars(inputNames);

outputNames = [Dist.uc;Dist.ud;Dist.y;Dist.x;Dist.d];
outputVars = Dist.get_vars(outputNames);

MPC = optimizer(Dist.C,Dist.J,ops,inputVars,outputVars);

%% Input parameters
%state space intitial states
rng(1); %reset rng seed so that experiments can be re-run from here
input_batt = cell(1,nHubs);
for i = 1:nHubs
    input_batt{i} = (battCap + 1)*0.2 + rand;
end

%demand and generation forecasts
ed_f = 0.9*datatable{1:nHubs*N,'ElectricityFacilitykWHourly'};
ed_f = pchip(1:60:nHubs*N*60,ed_f,1:15:nHubs*N*60)'; %interpolate for 15min intervals
hd_f = 0.9*datatable{1:nHubs*N,'HeatingGaskWHourly'};
hd_f = pchip(1:60:nHubs*N*60,hd_f,1:15:nHubs*N*60)'; %interpolate for 15min intervals

%price forecasts
eBuy_f = datatable{1:N,'eBuy'};
eBuy_f = pchip(1:60:N*60,eBuy_f,1:15:N*60)'; %interpolate for 15min intervals
eSell = 1*ones(N,1);
gBuy = 2.8*ones(N,1);
cDeg = 3.15*ones(N,1);


%% Algorithm parameters
t_max = 300;

conv = 0;

%initialise logs
solution_log = cell(sim_length,1);
time_log = zeros(sim_length,1);

tx_y_in = zeros(N,nHubs);
chp_y_in = zeros(N,nHubs);
batt_y_chg = zeros(N,nHubs);

%% Simulation loop
for k = 1:sim_length
    fprintf('k = %i \n',k)
    
    inputs = input_batt;
    
    for i = 1:nHubs
        input_load_e = ed_f(N*4*(i-1)+1:N*4*(i-1)+N);
        input_load_h = hd_f(N*4*(i-1)+1:N*4*(i-1)+N);
        
        inputs = [inputs,input_load_e,input_load_h];
    end
    
    eNet_P_e1_cost = eBuy_f(1:N);
    
    for i = 1:nHubs
        inputs = [inputs,eNet_P_e1_cost,eSell,gBuy,cDeg];
    end
    
    [solution,diagnostics,infostr,dual,output] = MPC(inputs);
    
    if diagnostics > 0
        keyboard
    end
    
    results = table(solution{:},'variablenames',outputNames');
    
    solution_log{k} = solution;
    time_log(k) = output.solveroutput.result.runtime;
    
    %iterate inputs
    ed_f = circshift(ed_f,-1);
    hd_f = circshift(hd_f,-1);
    eBuy_f = circshift(eBuy_f,-1);
    
    %update from solution
    for i = 1:nHubs
        varName_y_chg = strcat('batt',num2str(i),'_y_chg');
        input_batt{i} = alpha*input_batt{i} + dT*results{1,varName_y_chg};
        
        pName = strcat('tx',num2str(i),'_y_in');
        gName = strcat('chp',num2str(i),'_y_in');
        bName = strcat('batt',num2str(i),'_y_chg');
        
        tx_y_in(k,i) = results{1,pName};
        chp_y_in(k,i) = results{1,gName};
        batt_y_chg(k,i) = results{1,bName};
    end
end

%% Costs
cost = 0;
eBuy_f = datatable{1:N,'eBuy'};
eBuy_f = pchip(1:60:N*60,eBuy_f,1:15:N*60)'; %interpolate for 15min intervals
for k = 1:sim_length
    for j = 1:nHubs
        if tx_y_in(k,j) >= 0
            cost = cost + tx_y_in(k,j)*eBuy_f(k) + chp_y_in(k,j)*gBuy(1,1) + abs(batt_y_chg(k,j)*cDeg(1));
        else
            cost = cost + tx_y_in(k,j)*eSell(k) + chp_y_in(k,j)*gBuy(1,1) + abs(batt_y_chg(k,j)*cDeg(1));
        end
    end
end
% 
%% Determine overall consumption profiles 
sim_length = 96;
figure; set(gcf,'Units','centimeters');
set(gcf,'Position',[0 0 w+l+r h+b+t]);
sum_e1 = sum(tx_y_in,2);
stairs(sum_e1);
hold on
ylabel('Electricity, kW'); xlabel('Sampling Period, k');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);
xlim([0 sim_length]);