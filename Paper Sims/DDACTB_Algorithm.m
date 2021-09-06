%% Script to test Falsone decentralised optimisation approach with accelerated convergence heuristic

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
    hub = Aggregation(hubName,N,tx,chp,hp,ed,hd,batt,eBal,hBal);
    
    hub.connect(strcat(txName,'_y_out'),strcat(eBalName,'_y_snk1'),1);
    hub.connect(strcat(chpName,'_y_e'),strcat(eBalName,'_y_snk2'),1);
    hub.connect(strcat(eBalName,'_y_src1'),strcat(hpName,'_y_e'),1);
    hub.connect(strcat(eBalName,'_y_src2'),strcat(battName,'_y_chg'),1);
    hub.connect(strcat(eBalName,'_y_src3'),strcat(edName,'_y'),1);
    
    hub.connect(strcat(chpName,'_y_h'),strcat(hBalName,'_y_snk1'),1);
    hub.connect(strcat(hpName,'_y_h'),strcat(hBalName,'_y_snk2'),1);
    hub.connect(strcat(hBalName,'_y_src1'),strcat(hdName,'_y'),1);
    
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

ptb = unifrnd(-5e-2,5e-2,N,nHubs); %cost perturbation to avoid symmetrical system

%% Algorithm parameters
t_max = 300;

conv = 0;

alpha_num = 5e-4;

%initialise logs
solution_log = cell(nHubs,sim_length);
max_time_log = zeros(t_max,sim_length);

%% %initialise repeated parameters
lambdas = zeros(N,1);
b_e = 260*ones(N,1);

%% Simulation loop
for k = 1:sim_length
    fprintf('k = %i \n',k)
    
    lambdas_log = zeros(N,t_max);
    sum_log = zeros(N,t_max);
%     lambdas = zeros(N,1);
    s_e_upper = -inf*ones(N,nHubs);
    s_e_lower = inf*ones(N,nHubs);
    ro_e = zeros(N,nHubs);
    active = false(N,1);
    g_log = zeros(N,t_max);
    sub_alg = false;

    count_e = 0;
    n = 0;
    
    for t = 1:t_max
        fprintf('Iteration %i of %i \n', t,t_max);

        sum_e1 = zeros(N,1);

        max_time = 0;
        dual_val = 0;
        
        for i = 1:nHubs
            inputs = {input_batt(i)};
            
            input_load_e = ed_f(N*4*(i-1)+1:N*4*(i-1)+N);
            input_load_h = hd_f(N*4*(i-1)+1:N*4*(i-1)+N);
            
            inputs = [inputs,input_load_e,input_load_h];
            
            ecm_P_e1_cost = eBuy_f(1:N) + ptb(:,i);
            
            inputs = [inputs,ecm_P_e1_cost,eSell,lambdas,gBuy,cDeg];
            
            contName = strcat('controller_',num2str(i));
            controller = conts.(matlab.lang.makeValidName(contName));
            
            [solution,diagnostics,infostr,dual,output] = controller(inputs);
            
            if diagnostics > 0
                keyboard
            end
            
            max_time = max(output.solvertime,max_time);
            
            sum_e1 = sum_e1 + solution{1,4};
            
            if sub_alg == false
                s_e_upper(:,i) = max(s_e_upper(:,i),solution{1,4});
                s_e_lower(:,i) = min(s_e_lower(:,i),solution{1,4});
                ro_e(:,i) = s_e_upper(:,i) - s_e_lower(:,i);
            end
            solution_log{i,k} = solution;
        end
        
        max_time_log(t,k) = max_time;
      
        n = n + 1;
        
        %determine dual multipliers
        ro_e_max = zeros(N,1);
        for i = 1:N
            ro_e_max(i) = sum(maxk(ro_e(i,:),N,2)); %sum of N largest subproblem budget consumption changes
        end
            
        v = sum_e1 - b_e;
        g = sum_e1 - b_e + ro_e_max;
            
        if sub_alg == false
            idx = g > 0;
            active = active + idx;
        end
        
        if t > 2
            if sub_alg == false
                if ~any(active(idx) < 3) && issorted([lambdas_log(idx,t-2:t-1) lambdas(idx)],2,'ascend')
                    if any(sum_e1(idx) > b_e(idx))
                        sub_alg = true;
                        delta = 5e-2/2;
                        delta_max = delta;
                        delta_min = 0;
                        g_old = g;
                        v_old = v;
                        lambda_old = lambdas;
                        bisect = false;
                    end
                end
            else
                ascent = all((g_old(idx) > 0) <= (g(idx) > 0));
                if bisect == false
                    if ascent == true
                        delta_min = delta;
                        delta = max(5e-2/2,2*delta);
                        delta_max = delta;
                    elseif ~all((g_old(idx) < 0) < (g(idx) < 0))
                        idx_tmp = ((g_old < 0) < (g < 0));
                        idx = idx - idx_tmp;
                        idx = idx == true;
                        
                        delta = delta_min;
                        lambdas(idx_tmp) = lambda_old(idx_tmp) + delta;
                    else
                        delta_max = delta;
                        delta = (delta_min + delta_max)/2;
                        bisect = true;
                    end
                else
                    if ascent == true
                        delta_min = delta;
                        delta = (delta_min + delta_max)/2;
                    else
                        delta_max = delta;
                        delta = (delta_min + delta_max)/2;
                    end
                    if (delta_max - delta_min)/alpha_num < 1/sqrt(n)*min(abs(g))/2 && ascent == true
                        sub_alg = false;
                        active = false(N,1);
                        count_e = 0;
                    end
                end
                [g(idx) v(idx)]
            end
        end
        
        if sub_alg == false
            lambdas(:) = max(zeros(N,1),lambdas(:) + alpha_num/sqrt(n).*g);
        else
            lambdas(idx) = lambda_old(idx) + delta;
        end
            
        lambdas_log(:,t) = lambdas;
        g_log(:,t) = g;
        sum_log(:,t) = sum_e1;
        
        x = 1:t;
        for w = 1:N
            subplot(3,1,1);
            plot(lambdas_log(w,x))
            hold on
        end
        hold off
        title('Dual Multipliers (1:N)')
        xlabel('Iteration, t')
        subplot(3,1,2)
        plot(b_e)
        hold on
        plot(sum_e1)
        hold off
        title('Total Power Consumed')
        xlabel('Sampling Period, k')
        ylabel('Elec. Power, kW')
        subplot(3,1,3);
        plot(eBuy_f(1:24)+lambdas)
        title('Electricity Price + Dual Price')
        xlabel('Sampling Period, k')
        ylabel('Price of Elec. + dual, p/kWh')
        pause(0.01)
        
        if sub_alg == false && all(sum_e1 < b_e)
            count_e = count_e + 1;
        else
            count_e = 0;
        end
        
        if count_e > 5
            break
        end
        
    end
    
    %iterate inputs
    ptb = circshift(ptb,-1,1);
    ed_f = circshift(ed_f,-1);
    hd_f = circshift(hd_f,-1);
    eBuy_f = circshift(eBuy_f,-1);
    
    lambdas = circshift(lambdas,-1);
    lambdas(end) = 0;
    b_e = circshift(b_e,-1);
    b_e(end) = 260;
    
    %update from solution
    for i = 1:nHubs
        solution_tmp = solution_log{i,k};
        input_batt(i) = alpha*input_batt(i) + dT*solution_tmp{3}(1,1);
    end
end

%% Costs
% cost_flex = sum(eBuy_f(1:24).*sum_e1);
cost_flex = 0;
eBuy_f = datatable{1:N,'eBuy'};
eBuy_f = pchip(1:60:N*60,eBuy_f,1:15:N*60)'; %interpolate for 15min intervals
for k = 1:sim_length
    for j = 1:nHubs
        solution = solution_log{j,k};
        if solution{1,4}(1) >= 0
            cost_flex = cost_flex + solution{1,4}(1)*eBuy_f(k) + solution{1,6}(1)*gBuy(1,1) + abs(solution{1,3}(1))*cDeg(1);
        else
            cost_flex = cost_flex + solution{1,4}(1)*eSell(1) + solution{1,6}(1)*gBuy(1,1) + abs(solution{1,3}(1))*cDeg(1);
        end
    end
end

%% Determine overall consumption profiles 
sim_length = 48;
sum_e1 = zeros(sim_length,1);
% % sum_g1 = zeros(24,1);
for i = 1:sim_length
for j = 1:nHubs
solution = solution_log{j,i};
sum_e1(i,1) = sum_e1(i,1) + solution{1,4}(1);
% % sum_g1(i,1) = sum_g1(i,1) + solution{1,7}(1);
end
end
% hold on
% plot(sum_e1);

%% time comparison
iters = zeros(sim_length,1);
for i = 1:sim_length
    iters(i) = nnz(max_time_log(:,i));
end

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
stairs(eBuy_f(1:96)); hold on
stairs(ones(96,1));
stairs(2.8*ones(96,1));
legend('\pi_e^{buy}','\pi_e^{sell}','\pi_g^{buy}')
ylabel('Price, p/kWh'); xlabel('Sampling Period, k');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);
xlim([0 96]);

%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\prices', '-pdf','-eps');

%%
figure; set(gcf,'Units','centimeters');
set(gcf,'Position',[0 0 w+l+r h+b+t]);
x = 1:96;
hold on
for i = 1:175
    ed = ed_f(N*4*(i-1)+1:N*4*(i-1)+N*4);
    stairs(x,ed);
end
ylabel('Electricity Demand, kW'); xlabel('Sampling Period, k');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);
xlim([1 96])

%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\elec', '-pdf','-eps');

%%
figure; set(gcf,'Units','centimeters');
set(gcf,'Position',[0 0 w+l+r h+b+t]);
x = 1:96;
hold on
for i = 1:175
    hd = hd_f(N*4*(i-1)+1:N*4*(i-1)+N*4);
    stairs(x,hd);
end
ylabel('Heating Demand, kW'); xlabel('Sampling Period, k');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);
xlim([1 96])

%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\heat', '-pdf','-eps');

%%
figure; set(gcf,'Units','centimeters');
set(gcf,'Position',[0 0 w+l+r h+b+t]);
% hold on
% stairs(sum_e1(1:96));
% ylabel('Electricity, kW'); xlabel('Sampling Period, k');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);
xlim([0 sim_length]);

%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\total_power', '-pdf','-eps');

%%
figure; set(gcf,'Units','centimeters');
set(gcf,'Position',[0 0 w+l+r h+b+t]);
plot(sum(max_time_log(:,1:49)));
ylabel('Time, secs'); xlabel('Sampling Instance, k');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);
xlim([0 sim_length]);

%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\comp_time', '-pdf','-eps');

%%
figure; set(gcf,'Units','centimeters');
set(gcf,'Position',[0 0 w+l+r h+b+t]);
plot(iters(1:49));
ylabel('# iterations'); xlabel('Sampling Instance, k');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);
xlim([0 96]);

%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\comp_iters', '-pdf','-eps');

%%
figure; set(gcf,'Units','centimeters');
set(gcf,'Position',[0 0 w+l+r h+b+t]);
x = 1:1000;
% for i = 1:N
    plot(lambdas_log(24,x))
%     hold on
% end
ylabel('Dual Price, p/kWh'); xlabel('# iterations');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);

%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\duals_2', '-pdf','-eps');

%%
figure; set(gcf,'Units','centimeters');
eBuy_f = datatable{1:N,'eBuy'};
eBuy_f = pchip(1:60:N*60,eBuy_f,1:15:N*60)'; %interpolate for 15min intervals
set(gcf,'Position',[0 0 w+l+r h+b+t]);
x = 0:24;
% yyaxis left
plot(x,[b_e;260])
hold on
stairs(x,[sum_e1;sum_e1(end)])
stairs(x,[sum_e1_0_1;sum_e1_0_1(end)])
stairs(x,[sum_e1_bis_new;sum_e1_bis_new(end)])
ylabel('Electriciy, kW'); xlabel('Sampling Period, k');
% yyaxis right
% stairs(x,[eBuy_f(52:75);eBuy_f(75)])
legend('Capacity Limit','a = 0.01','a = 0.1','DDACT-B','location','southwest')
% ylabel('Price, p/kWh');
set(gcf,'Color','w');
set(gca,'Position',[l,b,w,h]);


%%
export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\violation_2', '-pdf','-eps');