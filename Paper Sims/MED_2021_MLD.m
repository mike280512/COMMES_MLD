%% Script to test Falsone decentralised optimisation approach
% -MED 2021 case study using MLD software

clear all;
yalmip clear;

%% Data and simulation paramters
%prediction horizon
N = 24;

%number of segments in flexible demand cycle
Nn = 6;

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
alpha = 0.999^(1/4);%0.999;%
dT = 1/4;%1;%

%controller options
ops = sdpsettings('solver','GUROBI','verbose',0,'gurobi.MIPGap',1e-4,'savesolveroutput',1);%,'relax',2);

%random number generator seed
rng('default');
rng(1);

%% Hub controller definition
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
    tx.add_dual_cost(strcat(txName,'_lambda_e'),1,{strcat(txName,'_y_in')});
    
    %chp definition
    chpName = strcat('chp',num2str(i));
    bigM = 6;
    chp = CHPModel(chpName,N,bigM,etaE,etaH);
    chp.add_val_cost(strcat(chpName,'_y_in'));
    chp.add_dual_cost(strcat(chpName,'_lambda_g'),1,{strcat(chpName,'_y_in')});
    
    %hp definition
    hpName = strcat('hp',num2str(i));
    bigM = 100;
    hp = AirSourceHPModel(hpName,N,bigM,COP);
    
    %electricity load
    edName = strcat('ed',num2str(i));
    ed = FixedDemandModel(edName,N);
    
    %pliable heat demand
    hdName = strcat('hd',num2str(i));
    bigM = 100;
    hd = PDemandModel(hdName,N,bigM,Nn);
    
    % initialise logic controller
    hd_f = datatable{N*(i-1)+1:N*i,'HeatingGaskWHourly'};
    hd_f = pchip(1:60:N*60,hd_f,1:15:N*60);
    Nc = N/Nn; %number of periods per energy consumption cycle (unless baseline consumption satisfied)
    hd_f = sum(reshape(hd_f,Nc,length(hd_f)/Nc));
    hdLCName = strcat('hdLC_',num2str(i));
    LCs.(matlab.lang.makeValidName(hdLCName)) = PDemandLC(hd,hd_f,Nc);
    
    %battery storage model
    battName = strcat('batt',num2str(i));
    bigM = 10;
    batt = BatteryModel(battName,N,bigM,alpha,dT,etaChg,etaDchg);
    batt.change_bounds(strcat(battName,'_x'),battCap*0.2,battCap*0.8);
    batt.change_bounds(strcat(battName,'_y_chg'),-battChg,battChg);
    batt.add_abs_cost(strcat(battName,'_y_chg')); %battery degradation cost
    
    %electrical power balance
    eBalName = strcat('eBal',num2str(i));
    bigM = 100;
    Nsnk = 2;
    Nsrc = 3;
    eBal = PowerBalanceModel(eBalName,N,bigM,Nsnk,Nsrc);
    
    %heat power balance
    hBalName = strcat('hBal',num2str(i));
    bigM = 100;
    Nsnk = 2;
    Nsrc = 1;
    hBal = PowerBalanceModel(hBalName,N,bigM,Nsnk,Nsrc);
    
    %aggregated hub
    hubName = strcat('hub',num2str(i));
    hub = Aggregation(hubName,N,tx,chp,hp,ed,hd,batt,eBal,hBal);
    
    hub.connect(strcat(txName,'_y_out'),strcat(eBalName,'_y_snk1'),1);
    hub.connect(strcat(chpName,'_y_e'),strcat(eBalName,'_y_snk2'),1);
    hub.connect(strcat(eBalName,'_y_src1'),strcat(hpName,'_y_e'),1);
    hub.connect(strcat(eBalName,'_y_src2'),strcat(battName,'_y_chg'),1);
    hub.connect(strcat(eBalName,'_y_src3'),strcat(edName,'_y'),1);
    
    hub.connect(strcat(chpName,'_y_h'),strcat(hBalName,'_y_snk1'),1);
    hub.connect(strcat(hpName,'_y_h'),strcat(hBalName,'_y_snk2'),1);
    hub.connect(strcat(hBalName,'_y_src1'),strcat(hdName,'_y_L'),1);
    
    inputNames = [hub.m;hub.w;hub.l];
    inputVars = hub.get_vars(inputNames);

    outputNames = [hub.uc;hub.ud;hub.y;hub.x;hub.d];
    outputVars = hub.get_vars(outputNames);
    
    contName = strcat('controller_',num2str(i));
    
    conts.(matlab.lang.makeValidName(contName)) = optimizer(hub.C,hub.J,ops,inputVars,outputVars);
    
    fprintf('hub %i complete \n', i);
    toc
end

%% Input parameters
%state space intitial states
rng(1); %reset rng seed so that experiments can be re-run from here
input_batt = zeros(nHubs,1);
for i = 1:nHubs
    input_batt(i) = (battCap + 1)*0.2 + rand;
end

%demand and generation forecasts
ed_f = datatable{1:nHubs*N,'ElectricityFacilitykWHourly'};
ed_f = pchip(1:60:nHubs*N*60,ed_f,1:15:nHubs*N*60)';
input_load_h = cell(nHubs,1);

%initial conditions for logic controllers
lBase = zeros(nHubs,Nn);
procStatus = zeros(nHubs,Nn);
compStatus = zeros(nHubs,Nn);

for i = 1:nHubs
    hdLCName = strcat('hdLC_',num2str(i));
    hdLC = LCs.(matlab.lang.makeValidName(hdLCName));
    hdLC.release;
end

%price forecasts
eBuy_f = datatable{1:N,'eBuy'};
eBuy_f = pchip(1:60:N*60,eBuy_f,1:15:N*60)';
eSell = 1*ones(N,1);
gBuy = 5*ones(N,1); %2.8
cDeg = 3.15*ones(N,1);
ptb = unifrnd(-3e-2,3e-2,N,nHubs); %cost perturbation to avoid symmetrical system

%% Algorithm parameters
t_max = 500;
b_e = 500*ones(N,1);
b_g = 500*ones(N,1);

%initialise logs
lambda_e_log = zeros(N,150);
lambda_g_log = zeros(N,150);
sum_e1_log = zeros(N,150);
sum_g1_log = zeros(N,150);
solution_log = cell(nHubs,sim_length);
max_time_log = zeros(150,sim_length);

%% Simulation loop
for k = 1:sim_length
    fprintf('k = %i of %i \n', k,sim_length);
    alpha_num = 1e-1; %initial stepsize numerator

    lambda_e = zeros(N,1);
    lambda_g = zeros(N,1);
    
    s_e_upper = zeros(N,nHubs);
    s_e_upper(:) = -inf;
    s_e_lower = zeros(N,nHubs);
    s_e_lower(:) = inf;
    ro_e = zeros(N,nHubs);
    
    s_g_upper = zeros(N,nHubs);
    s_g_upper(:) = -inf;
    s_g_lower = zeros(N,nHubs);
    s_g_lower(:) = inf;
    ro_g = zeros(N,nHubs);

    count_e = 0;
    count_g = 0;
    
    for i = 1:nHubs
        hdLCName = strcat('hdLC_',num2str(i));
        hdLC = LCs.(matlab.lang.makeValidName(hdLCName));
        input_load_h{i} = hdLC(lBase(i,:),procStatus(i,:),compStatus(i,:));
    end
    
    for t = 1:t_max
        fprintf('Iteration %i of %i \n', t,t_max);
        sum_e1 = zeros(N,1);
        sum_g1 = zeros(N,1);
        max_time = 0;
        
        for i = 1:nHubs
            inputs = input_load_h{i};
            
            inputs = [inputs,input_batt(i)];
            
            input_load_e = ed_f(N*4*(i-1)+1:N*4*(i-1)+N);
            inputs = [inputs,input_load_e];
            
            ecm_P_e1_cost = eBuy_f(1:N) + ptb(:,i);
            
            inputs = [inputs,ecm_P_e1_cost,eSell,lambda_e,gBuy,lambda_g,cDeg];
            
            contName = strcat('controller_',num2str(i));
            controller = conts.(matlab.lang.makeValidName(contName));
            
            [solution,diagnostics,infostr,dual,output] = controller(inputs);
            
            if diagnostics > 0
                keyboard
            end
            
            max_time = max(output.solvertime,max_time);
            
            sum_e1(:) = sum_e1(:) + solution{1,5};
            s_e_upper(:,i) = max(s_e_upper(:,i),solution{1,5});
            s_e_lower(:,i) = min(s_e_lower(:,i),solution{1,5});
            ro_e(:,i) = s_e_upper(:,i) - s_e_lower(:,i);
            
            sum_g1(:) = sum_g1(:) + solution{1,1};
            s_g_upper(:,i) = max(s_g_upper(:,i),solution{1,1});
            s_g_lower(:,i) = min(s_g_lower(:,i),solution{1,1});
            ro_g(:,i) = s_g_upper(:,i) - s_g_lower(:,i);
            
            solution_log{i,k} = solution;
        end
        
        ro_e_max = zeros(N,1);
        for i = 1:N
            ro_e_max(i) = sum(maxk(ro_e(i,:),N,2)); %sum of N largest subproblem budget consumption changes
        end
        
        ro_g_max = zeros(N,1);
        for i = 1:N
            ro_g_max(i) = sum(maxk(ro_g(i,:),N,2)); %sum of N largest subproblem budget consumption changes
        end
        
        lambda_e(:) = max(zeros(N,1),lambda_e(:) + (alpha_num/(10+t))*(sum_e1(:) - b_e(:) + ro_e_max(:)));
        lambda_g(:) = max(zeros(N,1),lambda_g(:) + (alpha_num/(10+t))*(sum_g1(:) - b_g(:) + ro_g_max(:)));
        
        max_time_log(t,k) = max_time;
        
        lambda_e_log(:,t) = lambda_e;
        lambda_g_log(:,t) = lambda_g;
        sum_e1_log(:,t) = sum_e1;
        sum_g1_log(:,t) = sum_g1;
        
        if t == 15
            alpha_num = alpha_num / 10;
        end
        
%         subplot(3,1,1)
%         for i = 1:N
%             plot(1:t,lambda_e_log(i,1:t)); hold on
%         end
%         hold off
%         title('Dual Multipliers (1:N)')
%         xlabel('Iteration, t')
%         subplot(3,1,2);
%         plot(b_e)
%         hold on
%         stairs(sum_e1)
%         stairs(sum_g1)
%         hold off
%         xlabel('Sampling Period, k')
%         ylabel('Elec. Power, kW')
%         subplot(3,1,3);
%         stairs(eBuy_f(1:24)+lambda_e)
%         xlabel('Sampling Period, k')
%         ylabel('Price of Elec. + dual, p/kWh')
%         pause(0.01)
        
        if sum_e1(:) < b_e(:)
            count_e = count_e + 1;
        else
            count_e = 0;
        end
        
        if sum_g1(:) < b_g(:)
            count_g = count_g + 1;
        else
            count_g = 0;
        end
        
        if count_e > 5 && count_g > 5
            break
        end
    end
    
    %iterate inputs
    ptb = circshift(ptb,-1,1);
    ed_f = circshift(ed_f,-1);
    eBuy_f = circshift(eBuy_f,-1);
    
    %update from solution
    for i = 1:nHubs
        solution_tmp = solution_log{i,k};
        input_batt(i) = input_batt(i) + dT*solution_tmp{4}(1,1);
        lBase(i,:) = solution_tmp{3}(1,:);
        procStatus(i,:) = solution_tmp{26}(1,:);
        compStatus(i,:) = solution_tmp{27}(1,:);
    end
end

%% plots
sum_e1 = zeros(sim_length,1);
sum_g1 = zeros(sim_length,1);
for i = 1:sim_length
for j = 1:175
solution = solution_log{j,i};
sum_e1(i,1) = sum_e1(i,1) + solution{1,5}(1);
sum_g1(i,1) = sum_g1(i,1) + solution{1,1}(1);
end
end

x = 1:sim_length;
%% figure values
b = 1.5; t = 0.5; m = 2.5; l = 1.8; r = 0.5;
k = 2;
w = k*8.8; h = w/3;
set(0,'defaultAxesUnits','centimeters');

%% default figure settings
set(0,'defaultAxesGridLineStyle','-');
set(0,'defaultAxesYGrid','on');
set(0,'defaultAxesXGrid','on');
set(0,'defaultAxesFontName','Times New Roman');
set(0,'defaultAxesFontSize',8*k);
set(0,'defaultLegendFontName','Times New Roman');
set(0,'defaultLegendFontSize',5*k);
set(0,'defaultLineLineWidth',1*k);
%%
figure; set(gcf,'Units','centimeters');
set(gcf,'Position',[0 0 w+l+r h+b+t]);
stairs(x,sum_e1);
ylabel('Electricity, kW'); xlabel('Sampling Period, k');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);
xlim([0 sim_length]);
%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Presentations\MED_2021\figures\elec_15', '-pdf');

%%
figure; set(gcf,'Units','centimeters');
set(gcf,'Position',[0 0 w+l+r h+b+t]);
stairs(x,sum_g1);
ylabel('Gas, kW'); xlabel('Sampling Period, k');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);
xlim([0 sim_length]);
%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Presentations\MED_2021\figures\gas_15', '-pdf');
