%% script to test Vujanic EV case study with both Falsone and DDACT-B algorithms
% close all
clear all
yalmip clear

K = 300; %max iterations
t_max = 50;

%study logs
max_time_log_F = zeros(K,t_max);
max_time_log_SG = zeros(K,t_max);
cost_log_F = zeros(t_max,1);
cost_log_SG = zeros(t_max,1);

%% Parameters

N = 24; %horizon length
I = 300; %number of EVs

charge_only = 1;

for t = 1:t_max
%random samples from uniform distributions (values from paper):
% rng('default');
% rng(1); %use commond rng seed for repeatability 
Pi = unifrnd(3,5,I,1); %charging power
Emin = 1; %minimum battery level
Emax = unifrnd(8,16,I,1); %maximum battery level
Einit = unifrnd(0.2,0.5,I,1).*Emax; %initial battery level
Eref = unifrnd(0.55,0.8).*Emax; %reference battery level
Eff = unifrnd(0.015,0.075,I,1); %efficiency of charging
dT = 1/3; %sampling inerval (hours)
Pmax = 2.8*I; %max power through shared line
Pmin = -Pmax; %min power through shared line (redundant in charging only case)
Cchg =  unifrnd(0.019,0.035,N,1); %charging costs over simulation horizon 
Cdchg = 1.1*Cchg; %discharging payments over simulation horizon
ptb = unifrnd(-3e-4,3e-4,N,I); %cost perturbation to avoid symmetrical system

%resource vectors for shared constraints
b_min = Pmin*ones(N,1);
b_max = Pmax*ones(N,1);

%% Define decision variables
e = sdpvar(N,1); %continuous variables
u = binvar(N,1); %binary variables
v = binvar(N,1);

%% Define controller input parameters (dummy variables used as placeholders - fixed at runtime)
PiIn = sdpvar(1,1);
EmaxIn = sdpvar(1,1);
EinitIn = sdpvar(1,1);
ErefIn = sdpvar(1,1);
EffIn = sdpvar(1,1);
ptbIn = sdpvar(N,1);

%% Define dual input variables (dummy variables used as placeholders - fixed at runtime)
lambdaIn = sdpvar(N,1); %associated with minimum capacity constraint
muIn = sdpvar(N,1); %associated with maximum capacity constraint

%% Define controller constraints
c = []; %initiate empty constraint vector
c = [c, e(1) == EinitIn];
c = [c, e(N) >= ErefIn];
c = [c, Emin <= e <= EmaxIn];
c = [c, u + v <= 1];
if charge_only == 1
    c = [c, v == 0]; %charging case only
end
for k = 1:N-1
    c = [c, e(k+1) == e(k) + PiIn*dT*((1-EffIn)*u(k) - (1+EffIn)*v(k))];
end

%% Define controller objective for inner minimisation problem
obj = PiIn*((Cchg + ptbIn - lambdaIn + muIn)'*u - (Cdchg + ptbIn - lambdaIn + muIn)'*v);

%% Define agent controller
%name variable input parameters and outputs
inputs = {PiIn, EmaxIn, EinitIn, ErefIn, EffIn, ptbIn, lambdaIn, muIn};
outputs = {e, u, v};

%define options
ops = sdpsettings('solver','GUROBI','verbose',0,'savesolveroutput',1);

%instantiate agent controller
agent = optimizer(c,obj,ops,inputs,outputs);

%% Algorithm parameters

%initialise dual variables
lambda = zeros(N,1);
mu = zeros(N,1);

%initialise upper and lower limit variables
s_upper = -inf*ones(N,I);
s_lower = inf*ones(N,I);

ro = zeros(N,I);

conv = 0;

%% Logs
lambda_log = zeros(N,K+1);
mu_log = zeros(N,K+1);
sum_Pi_log = zeros(N,K);
dual_obj_log = zeros(K,1);
g_log = zeros(N,K);

%% Simulation Falsone
for k = 1:K
    disp('Iteration:')
    disp(num2str(k))
    
    sum_Pi = zeros(N,1); %initialise running sum total of power consumed/exported
    inner_obj_tot = 0; %initialise running sum total of inner problem objective values
    max_time = -inf; %initialise upper limit of agent computation time in iteration
    
    %loop to determine inner minimisation solution from each agent
    for i = 1:I
        %define inputs for agent i
        inputs = {Pi(i), Emax(i), Einit(i), Eref(i), Eff(i), ptb(:,i), lambda, mu};
        
        %determine inner minimisation solution for agent i
        [solution,diagnostics,infostr,dual,output] = agent(inputs);
        
        if diagnostics > 0
            keyboard
        end
        
        inner_obj = output.solveroutput.result.objval;
                
        %update running totals
        sum_Pi = sum_Pi + Pi(i)*(solution{1,2} - solution{1,3});        
        inner_obj_tot = inner_obj_tot + inner_obj;
        
        %update adaptive contraction if sub-algorithm concluded
        s_upper(:,i) = max(s_upper(:,i),Pi(i)*(solution{1,2} - solution{1,3}));
        s_lower(:,i) = min(s_lower(:,i),Pi(i)*(solution{1,2} - solution{1,3}));
        ro(:,i) = s_upper(:,i) - s_lower(:,i);
        %save max agent time in iteration
        max_time = max(output.solvertime,max_time);
        
    end
    
    ro_max = zeros(N,1);
    for i = 1:N
        ro_max(i) = sum(maxk(ro(i,:),N,2));
    end
    
    g = sum_Pi(:) - b_max(:) + ro_max(:);
    
    dual_obj_log(k,1) = inner_obj_tot + lambda'*(b_min + ro_max) - mu'*(b_max - ro_max);
    
    %dual variable updates
    lambda = max(zeros(N,1), lambda + (1e-5/(k))*(b_min - ro_max - sum_Pi));
    mu = max(zeros(N,1), mu + (1e-5/k).*(sum_Pi - b_max + ro_max));
    
    
    %update dual logs
    lambda_log(:,k+1) = lambda;
    mu_log(:,k+1) = mu;
    sum_Pi_log(:,k) = sum_Pi;
    max_time_log_F(k,t) = max_time;
    g_log(:,k) = g;
    
    
    subplot(3,1,1);
    title('mus'); hold on;
    for i = 1:N
        plot(1:k,mu_log(i,1:k));
    end
    hold off
    subplot(3,1,2); title('sum Pi');
    plot(b_max)
    hold on
    stairs(sum_Pi)
    hold off
    subplot(3,1,3);
    stairs(Cchg+mu)
    pause(0.01);
    
    if sum_Pi <= b_max
        if sum_Pi >= b_min
            conv = conv + 1;
        end
    else
        conv = 0;
    end
    
    if conv > 5 %stopping criteria
        break
    end
end
% Calculate cost
cost_log_F(t,1) = Cchg'*sum_Pi;

%% Algorithm parameters
%initialise dual variables
lambda = zeros(N,1);
mu = zeros(N,1);

%initialise upper and lower limit variables
s_upper = -inf*ones(N,I);
s_lower = inf*ones(N,I);

ro = zeros(N,I);

alpha_num = 1e-6;

conv = 0;
active = false(N,1);
sub_alg = false;

%% Logs
lambda_log = zeros(N,K+1);
mu_log = zeros(N,K+1);
sum_Pi_log = zeros(N,K);
dual_obj_log = zeros(K,1);
g_log = zeros(N,K);

%% Simulation DDACT-B
for k = 1:K
    disp('Iteration:')
    disp(num2str(k))
    
    sum_Pi = zeros(N,1); %initialise running sum total of power consumed/exported
    inner_obj_tot = 0; %initialise running sum total of inner problem objective values
    max_time = -inf; %initialise upper limit of agent computation time in iteration
    
    %loop to determine inner minimisation solution from each agent
    for i = 1:I
        %define inputs for agent i
        inputs = {Pi(i), Emax(i), Einit(i), Eref(i), Eff(i), ptb(:,i), lambda, mu};
        
        %determine inner minimisation solution for agent i
        [solution,diagnostics,infostr,dual,output] = agent(inputs);
        
        if diagnostics > 0
            keyboard
        end
        
        inner_obj = output.solveroutput.result.objval;
                
        %update running totals
        sum_Pi = sum_Pi + Pi(i)*(solution{1,2} - solution{1,3});        
        inner_obj_tot = inner_obj_tot + inner_obj;
        
        %update adaptive contraction if sub-algorithm concluded
        if sub_alg == false
            s_upper(:,i) = max(s_upper(:,i),Pi(i)*(solution{1,2} - solution{1,3}));
            s_lower(:,i) = min(s_lower(:,i),Pi(i)*(solution{1,2} - solution{1,3}));
            ro(:,i) = s_upper(:,i) - s_lower(:,i);
        end
        %save max agent time in iteration
        max_time = max(output.solvertime,max_time);
        
    end
    
    ro_max = zeros(N,1);
    for i = 1:N
        ro_max(i) = sum(maxk(ro(i,:),N,2));
    end
    
    g = sum_Pi(:) - b_max(:) + ro_max(:);
    
    %implement bisection method
    if sub_alg == false
        idx = g>0;
        active = active + idx;
    end

    dual_obj_log(k,1) = inner_obj_tot + lambda'*(b_min + ro_max) - mu'*(b_max - ro_max);
    
    if k > 2
        if sub_alg == false
            if ~any(active(idx) < 3) && issorted([mu_log(idx,k-2:k-1) mu(idx)],2,'ascend')
                if any(sum_Pi(idx) > b_max(idx))
                    sub_alg = true;
                    delta = 3e-4/2;
                    delta_max = delta;
                    delta_min = 0;
                    g_old = g;
                    mu_old = mu;
                    bisect = false;
                end
            end
        else
            ascent = all((g_old(idx) > 0) <= (g(idx) > 0)); 
            if bisect == false
                if ascent == true
                    delta_min = delta;
                    delta = max(3e-4/2,2*delta);
                    delta_max = delta;
                elseif ~all((g_old(idx) < 0) < (g(idx) < 0))
                    idx_tmp = ((g_old < 0) < (g < 0));
                    idx = idx - idx_tmp;
                    idx = idx == true;
                    
                    delta = delta_min;
                    mu(idx_tmp) = mu_old(idx_tmp) + delta;
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
            end
            if delta_max - delta_min < 3e-4/2 && ascent == true%(delta_max - delta_min)/alpha_num < 1/sqrt(k)*min(abs(g))/2 && ascent == true
                sub_alg = false;
                active = false(N,1);
            end
        end
    end
    
    subplot(3,1,1);
    title('mus'); hold on;
    for i = 1:N
        plot(1:k,mu_log(i,1:k));
    end
    hold off
    subplot(3,1,2); title('sum Pi');
    plot(b_max)
    hold on
    stairs(sum_Pi)
    hold off
    subplot(3,1,3);
    stairs(Cchg+mu)
    pause(0.01);
    
    if sub_alg == false    
        %dual variable updates
        lambda = max(zeros(N,1), lambda + (alpha_num/sqrt(k))*(b_min - ro_max - sum_Pi));
        mu = max(zeros(N,1), mu + (alpha_num/sqrt(k)).*(sum_Pi - b_max + ro_max));
    else
        mu(idx) = mu_old(idx) + delta;
    end
    
    %update dual logs
    lambda_log(:,k+1) = lambda;
    mu_log(:,k+1) = mu;
    sum_Pi_log(:,k) = sum_Pi;
    max_time_log_SG(k,t) = max_time;
    g_log(:,k) = g;
    
    
    if sub_alg == false && all(sum_Pi <= b_max)
            conv = conv + 1;
    else
        conv = 0;
    end
    
    if conv > 5 %stopping criteria
        break
    end
end
% Calculate cost
cost_log_SG(t,1) = Cchg'*sum_Pi;
close all

end

% %% figure values
% b = 1.5; t = 0.5; m = 2.5; l = 1.8; r = 0.5;
% k = 2;
% w = k*8.8; h = w/3;
% set(0,'defaultAxesUnits','centimeters');
% 
% %% default figure settings
% set(0,'defaultAxesGridLineStyle','-');
% set(0,'defaultAxesYGrid','on');
% set(0,'defaultAxesXGrid','on');
% set(0,'defaultAxesFontName','Times New Roman');
% set(0,'defaultAxesFontSize',8*k);
% set(0,'defaultLegendFontName','Times New Roman');
% set(0,'defaultLegendFontSize',5*k);
% set(0,'defaultLineLineWidth',1*k);
% 
% %% Figures
% edges = [-0.2 0:0.2:1.4 1.6];
% figure; set(gcf,'Units','centimeters');
% set(gcf,'Position',[0 0 w+l+r h+b+t]);
% histogram((cost_log_F-cost_log_SG)./cost_log_SG*100,edges)
% ylabel('Frequency'); xlabel('Cost Improvement, %');
% set(gcf,'Color','w');
% set(gca,'Position',[l,b,w,h]);
% 
% %
% % export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\EVcosts', '-pdf','-eps');
% 
% %%
% iters_F = zeros(100,1);
% for i = 1:100
% iters_F(i) = nnz(max_time_log_F(:,i));
% end
% iters_SG = zeros(100,1);
% for i = 1:100
% iters_SG(i) = nnz(max_time_log_SG(:,i));
% end
% edges = [0 10:10:70 80];
% figure; set(gcf,'Units','centimeters');
% set(gcf,'Position',[0 0 w+l+r h+b+t]);
% % histogram((iters_SG)./iters_F,edges)
% histogram(iters_F,edges)
% hold on
% histogram(iters_SG,edges)
% legend('DDACT','DDACT-B')
% ylabel('Frequency'); xlabel('# iterations to convergence');
% set(gcf,'Color','w');
% set(gca,'Position',[l,b,w,h]);
% 
% % export_fig('C:\Users\mjkirmt2\OneDrive - The University of Manchester\PhD_Docs\Papers\Draft Paper\figures\EViters', '-pdf','-eps');
