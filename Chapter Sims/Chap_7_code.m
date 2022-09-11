%% Script for district case study with NAG momentum decentralised algorithm
%networks have two main branches, therefore capacity limits are treated
%independently.

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

%number of agents
nAgents = 84;

%simulation length
sim_length = 2016;

%define conversion effeciencies
etaTx = 0.98; %transformer conversion efficiency
chg_e = 0.9; %charging efficiency (battery)
dchg_e = 0.85; %discharging efficiency (battery)
chp_e = 0.33; %chp electric efficiency
chp_h = 0.4; %chp heating efficiency
COP = 3.9; %hp COP
EER = 2.1; %hp EER

%fixed battery parameters
alphaBatt = 0.999^(1/4);
alphaTank = 0.9^(1/4);
alphaBorehole = 0.99997;
dT = 1/4;

%controller options
ops = sdpsettings('solver','GUROBI','verbose',0,'gurobi.MIPGap',1e-4,'savesolveroutput',1);%,'relax',2);

%random number generator seed
rng('default');
rng(1);

%% Electrical network
nBus = 85;
baseV = 11*1e3;
baseVA = 1e6;
bigM = 10; %big-M value
v1 = 1; %reference voltage at node 1
penalty = 1000; %penalty for violating reference constraints

%specify power factor at each bus
PF = 0.98*ones(nBus,1);

%matpower example case34sa branch matrix
%	fbus	tbus	r	x	b	rateA	rateB	rateC	ratio	angle	status	angmin	angmax
branch = [  %% (r and x specified in ohms here, converted to p.u. below)
	1	2	0.108	0.075	0	0	0	0	0	0	1	-360	360;
	2	3	0.163	0.112	0	0	0	0	0	0	1	-360	360;
	3	4	0.217	0.149	0	0	0	0	0	0	1	-360	360;
	4	5	0.108	0.074	0	0	0	0	0	0	1	-360	360;
	5	6	0.435	0.298	0	0	0	0	0	0	1	-360	360;
	6	7	0.272	0.186	0	0	0	0	0	0	1	-360	360;
	7	8	1.197	0.82	0	0	0	0	0	0	1	-360	360;
	8	9	0.108	0.074	0	0	0	0	0	0	1	-360	360;
	9	10	0.598	0.41	0	0	0	0	0	0	1	-360	360;
	10	11	0.544	0.373	0	0	0	0	0	0	1	-360	360;
	11	12	0.544	0.373	0	0	0	0	0	0	1	-360	360;
	12	13	0.598	0.41	0	0	0	0	0	0	1	-360	360;
	13	14	0.272	0.186	0	0	0	0	0	0	1	-360	360;
	14	15	0.326	0.223	0	0	0	0	0	0	1	-360	360;
	2	16	0.728	0.302	0	0	0	0	0	0	1	-360	360;
	3	17	0.455	0.189	0	0	0	0	0	0	1	-360	360;
	5	18	0.82	0.34	0	0	0	0	0	0	1	-360	360;
	18	19	0.637	0.264	0	0	0	0	0	0	1	-360	360;
	19	20	0.455	0.189	0	0	0	0	0	0	1	-360	360;
	20	21	0.819	0.34	0	0	0	0	0	0	1	-360	360;
	21	22	1.548	0.642	0	0	0	0	0	0	1	-360	360;
	19	23	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	7	24	0.91	0.378	0	0	0	0	0	0	1	-360	360;
	8	25	0.455	0.189	0	0	0	0	0	0	1	-360	360;
	25	26	0.364	0.151	0	0	0	0	0	0	1	-360	360;
	26	27	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	27	28	0.273	0.113	0	0	0	0	0	0	1	-360	360;
	28	29	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	29	30	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	30	31	0.273	0.113	0	0	0	0	0	0	1	-360	360;
	31	32	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	32	33	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	33	34	0.819	0.34	0	0	0	0	0	0	1	-360	360;
	34	35	0.637	0.264	0	0	0	0	0	0	1	-360	360;
	35	36	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	26	37	0.364	0.151	0	0	0	0	0	0	1	-360	360;
	27	38	1.002	0.416	0	0	0	0	0	0	1	-360	360;
	29	39	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	32	40	0.455	0.189	0	0	0	0	0	0	1	-360	360;
	40	41	1.002	0.416	0	0	0	0	0	0	1	-360	360;
	41	42	0.273	0.113	0	0	0	0	0	0	1	-360	360;
	41	43	0.455	0.189	0	0	0	0	0	0	1	-360	360;
	34	44	1.002	0.416	0	0	0	0	0	0	1	-360	360;
	44	45	0.911	0.378	0	0	0	0	0	0	1	-360	360;
	45	46	0.911	0.378	0	0	0	0	0	0	1	-360	360;
	46	47	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	35	48	0.637	0.264	0	0	0	0	0	0	1	-360	360;
	48	49	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	49	50	0.364	0.151	0	0	0	0	0	0	1	-360	360;
	50	51	0.455	0.189	0	0	0	0	0	0	1	-360	360;
	48	52	1.366	0.567	0	0	0	0	0	0	1	-360	360;
	52	53	0.455	0.189	0	0	0	0	0	0	1	-360	360;
	53	54	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	52	55	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	49	56	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	9	57	0.273	0.113	0	0	0	0	0	0	1	-360	360;
	57	58	0.819	0.34	0	0	0	0	0	0	1	-360	360;
	58	59	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	58	60	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	60	61	0.728	0.302	0	0	0	0	0	0	1	-360	360;
	61	62	1.002	0.415	0	0	0	0	0	0	1	-360	360;
	60	63	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	63	64	0.728	0.302	0	0	0	0	0	0	1	-360	360;
	64	65	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	65	66	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	64	67	0.455	0.189	0	0	0	0	0	0	1	-360	360;
	67	68	0.91	0.378	0	0	0	0	0	0	1	-360	360;
	68	69	1.092	0.453	0	0	0	0	0	0	1	-360	360;
	69	70	0.455	0.189	0	0	0	0	0	0	1	-360	360;
	70	71	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	67	72	0.182	0.075	0	0	0	0	0	0	1	-360	360;
	68	73	1.184	0.491	0	0	0	0	0	0	1	-360	360;
	73	74	0.273	0.113	0	0	0	0	0	0	1	-360	360;
	73	75	1.002	0.416	0	0	0	0	0	0	1	-360	360;
	70	76	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	65	77	0.091	0.037	0	0	0	0	0	0	1	-360	360;
	10	78	0.637	0.264	0	0	0	0	0	0	1	-360	360;
	67	79	0.546	0.226	0	0	0	0	0	0	1	-360	360;
	12	80	0.728	0.302	0	0	0	0	0	0	1	-360	360;
	80	81	0.364	0.151	0	0	0	0	0	0	1	-360	360;
	81	82	0.091	0.037	0	0	0	0	0	0	1	-360	360;
	81	83	1.092	0.453	0	0	0	0	0	0	1	-360	360;
	83	84	1.002	0.416	0	0	0	0	0	0	1	-360	360;
	13	85	0.819	0.34	0	0	0	0	0	0	1	-360	360;
];

%equivalent adjacency matrix
adjMatrix = zeros(nBus);
for i = 1:nBus-1
adjMatrix(branch(i,1),branch(i,2)) = complex(branch(i,3),branch(i,4));
end

adjMatrix = adjMatrix./(baseV^2/baseVA); %convert to p.u.

eNet = ElecNetworkModel('eNet',N,bigM,adjMatrix,v1,PF,0.1); %instantiate electrical network component

%define bounds
for i = 2:size(adjMatrix,1)
    eNet.change_bounds(strcat('eNet_y_v',num2str(i)),0.9^2,1.1^2); %voltage limits in p.u. (branch flow model considers squared voltages)
end
%add slack variables and soft constraints to allow reference power values to be
%used in definition of load flow
for i = 2:size(adjMatrix,1)
    eNet.add_slack(strcat('eNet_y_p',num2str(i)),[],[],penalty);
end
inputNamesENet = [eNet.m;eNet.l]; %specify reference values, current limits and costs as inputs
inputVars = eNet.get_vars(inputNamesENet); %convert string references into variable placeholders using get_vars()

outputNamesENet = [eNet.y;eNet.z]; %specify output and auxillary variables as outputs
outputVars = eNet.get_vars(outputNamesENet); %convert string references into variable placeholders

%initialise load flow optimizer
eNetSolver = optimizer(eNet.C,eNet.J,ops,inputVars,outputVars); %optimizer(constraints,objective function,options,input variables,output variable)

%% Heat Network
mbase = 10; %base mass flow
Hbase = mbase^2; %base pressure head
pipeC = 3e-4; %pipe head loss coefficient
ssCs = 5e-4*ones(nAgents,1); %substation head loss coefficient
bigM = 1000; %big-M value

adjMatrix = zeros(nBus);
for i = 1:nBus-1
adjMatrix(branch(i,1),branch(i,2)) = branch(i,3); %adjacency matrix topology follows AC network
end
adjMatrix(:,2:nBus) = adjMatrix(:,2:nBus)./max(max(adjMatrix))*pipeC; %normalise weights and apply pressure drop coefficient

adjMatrix(:,1) = []; adjMatrix(1,:) = []; %remove first node - electrical network to have additional node for slack bus

hNet = HeatNetworkModel('hNet',1,bigM,adjMatrix,ssCs,0.1); %instantiate heat netwok object

hNet.change_bounds('hNet_y_pumpDH1',0,50/Hbase); %limit the pump head gain to 50m at node 1
for i = 2:nBus-1
    hNet.change_bounds(strcat('hNet_y_pumpDH',num2str(i)),0,10/Hbase); %limit the head gain to 10m at remaining nodes
end
%add slack variables and soft constraints to allow reference mass values to be
%used in definition of load flow
for i = 2:size(adjMatrix,1)
    hNet.add_slack(strcat('hNet_y_phi',num2str(i)),[],[],penalty);
end
inputNamesHNet = [hNet.m;hNet.l]; %specify reference values and costs as inputs
inputVars = hNet.get_vars(inputNamesHNet); %convert string references into variable placeholders using get_vars()

outputNamesHNet = [hNet.y;hNet.z]; %specify outputs and auxillary variables as outputs
outputVars = hNet.get_vars(outputNamesHNet); %convert string references into variable placeholders

%controller options
ops = sdpsettings('solver','GUROBI','verbose',0,'gurobi.MIPGap',1e-3,'savesolveroutput',1);

%initialise load flow optimizer
hNetSolver = optimizer(hNet.C,hNet.J,ops,inputVars,outputVars); %optimizer(constraints,objective function,options,input variables,output variable)

%% Determine relevant nodes for shared constraints - AC network
%define digraph in order to analyze network topology
G = digraph(branch(:,1),branch(:,2));
%determine most central node
cNode = centrality(G,'outcloseness');
cNode = find(cNode == max(cNode));
%determine all nodes in tree above cNode
upNodes = toposort(G)';
upNodes = upNodes(2:find(upNodes == cNode));
%determine nodes on main branches
branches = successors(G,cNode);
%obtain all successors from each main branch using tree search
leavesE = cell(length(branches),1);
for i = 1:length(branches) %i = 1:2
    leavesE{i} = [upNodes; bfsearch(G,branches(i))]; %record upper trunk and branch nodes for each main branch
end

% Determine relevant nodes for shared constraints - heat network
%define digraph in order to analyze network topology
G = digraph(adjMatrix); %most recent adjMatrix is for heat network
%determine most central node
cNode = centrality(G,'outcloseness');
cNode = find(cNode == max(cNode));
%determine all nodes in tree above cNode
upNodes = toposort(G)';
upNodes = upNodes(2:find(upNodes == cNode));
%determine nodes on main branches
branches = successors(G,cNode);
%obtain all successors from each main branch using tree search
leavesH = cell(length(branches),1);
for i = 1:length(branches) %i = 1:2
    leavesH{i} = [upNodes; bfsearch(G,branches(i))]; %record upper trunk and branch nodes for each main branch
end

%% Agent controller definition
%controller options
ops = sdpsettings('solver','GUROBI','verbose',0,'gurobi.MIPGap',1e-4,'savesolveroutput',1);

%% heat network plant + storage agent 1
% waste heat from plant
hWaste = FixedGenerationModel('hWaste',N); %instantiate model

% demand from heat network
hd = FixedDemandModel('hd1',N);

% network pump model
bigM = 10000;
etaMotor = 0.7; %motor efficiency
etaHydra = etaMotor; %hydraulic efficiency
pump = VarSpeedPumpModel('pump1',N,bigM,etaMotor,etaHydra); %instantiate pump

% borehole storage field pump model
bigM = 10000;
etaMotor = 0.7;
etaHydra = etaMotor;
pump_sto = VarSpeedPumpModel('pump_sto',N,bigM,etaMotor,etaHydra);

% borehole storage model
btesName = 'BTES';
bigM = 10000;
btes = HotTankModel(btesName,N,bigM,alphaBorehole,dT); %instantiate storage
btes.change_bounds(strcat(btesName,'_x'),0,500e6); %change capacity limits
btes.change_bounds(strcat(btesName,'_y_chg'),-4000,4000); %change thermal transfer limits

%electrical power balance - used to connect aggregated component models
eBalName = 'eBal1';
bigM = 10000;
Nsnk = 1;
Nsrc = 2;
eBal = PowerBalanceModel(eBalName,N,bigM,Nsnk,Nsrc);
eBal.add_dual_cost(strcat(eBalName,'_lambda_e'),1,{strcat(eBalName,'_y_snk1')}); %variable connected to network model - associate with dual cost
eBal.add_val_cost(strcat(eBalName,'_y_snk1')); %variable connected to network - associate actual cost for real power

%heat power balance used to connect aggregated component models
hBalName = 'hBal1';
bigM = 10000;
Nsnk = 1;
Nsrc = 2;
hBal = PowerBalanceModel(hBalName,N,bigM,Nsnk,Nsrc);

%aggregated agent
agent = Aggregation('agent1',N,hWaste,hd,pump,pump_sto,btes,eBal,hBal); %instantiate with all componente models to be aggregated

agent.connect('hWaste_y','hBal1_y_snk1',1/4.18/4); %connect components - y_sink = \gamma * y_source, connect(y_source,y_sink,\gamma)
agent.connect('hBal1_y_src1','pump1_y_in',1);
agent.connect('hBal1_y_src2','pump_sto_y_in',1);
agent.connect('pump_sto_y_out','BTES_y_chg',4.18*4);
agent.connect('pump1_y_out','hd1_y',1);
agent.connect('eBal1_y_src1','pump1_y_e',1);
agent.connect('eBal1_y_src2','pump_sto_y_e',1);

inputNames = [agent.m;agent.w;agent.l]; %specify system measurements, disturbances and costs as inputs
inputVars = agent.get_vars(inputNames); %get variable placeholders from strings

outputNames = [agent.uc;agent.ud;agent.y;agent.x;agent.z]; %specify system input variables, outputs, states and auxillary variables as outputs
outputVars = agent.get_vars(outputNames);

contName = 'controller_1';
conts.(matlab.lang.makeValidName(contName)) = optimizer(agent.C,agent.J,ops,inputVars,outputVars); %instantiate controller of agent at node 1


%% prosumer agents
etaChg = zeros(nAgents,1); %placeholder for random storage parameters
etaDchg = zeros(nAgents,1);
for i = 2:nAgents
    tic
    %define randomised properties
    battCap = 24 + unifrnd(-1,1);
    battChg = 12 + unifrnd(-1,1);
    etaChg(i) = chg_e + unifrnd(-1,1)*0.05;
    etaDchg(i) = dchg_e + unifrnd(-1,1)*0.05;
    
    %hp definition
    hpName = strcat('wshp',num2str(i));
    bigM = 500;
    hp = RevWaterSourceHPModel(hpName,N,bigM,COP,EER);
    hp.add_dual_cost(strcat(hpName,'_lambda_h'),1,{strcat(hpName,'_y_in')}); %heat pump input connected directly to network 
                                                                             %- associated with dual cost for heat network
    
    %electricity load
    edName = strcat('ed',num2str(i));
    ed = FixedDemandModel(edName,N);
    
    % heat demand
    hdName = strcat('hd',num2str(i));
    hd = FixedDemandModel(hdName,N);
    
    %battery storage model
    battName = strcat('batt',num2str(i));
    bigM = 20;
    batt = BatteryModel(battName,N,bigM,alphaBatt,dT,etaChg(i),etaDchg(i));
    batt.change_bounds(strcat(battName,'_x'),battCap*0.2,battCap*0.8);
    batt.change_bounds(strcat(battName,'_y_chg'),-battChg,battChg);
    batt.add_abs_cost(strcat(battName,'_y_chg')); %battery degradation cost
    
    %hot tank storage model
    tankName = strcat('tank',num2str(i));
    bigM = 40;
    tank = HotTankModel(tankName,N,bigM,alphaTank,dT);
    tank.change_bounds(strcat(tankName,'_x'),0,40);
    tank.change_bounds(strcat(tankName,'_y_chg'),-20,20);
    
    % pump definition
    pumpName = strcat('pump',num2str(i));
    bigM = 500;
    etaMotor = 0.7;
    etaHydra = etaMotor;
    pump = VarSpeedPumpModel(pumpName,N,bigM,etaMotor,etaHydra);
    
    %electrical power balance
    eBalName = strcat('eBal',num2str(i));
    bigM = 500;
    Nsnk = 1;
    Nsrc = 4;
    eBal = PowerBalanceModel(eBalName,N,bigM,Nsnk,Nsrc);
    eBal.add_buy_sell_cost(strcat(eBalName,'_y_snk1')); %variable connected to network - associate costs for buying or selling
    eBal.add_dual_cost(strcat(eBalName,'_lambda_e'),1,{strcat(eBalName,'_y_snk1')}); %variable connected to network - associate dual cost
    
    %heat power balance
    hBalName = strcat('hBal',num2str(i));
    bigM = 500;
    Nsnk = 1;
    Nsrc = 2;
    hBal = PowerBalanceModel(hBalName,N,bigM,Nsnk,Nsrc);
    
    %aggregated agent
    agentName = strcat('agent',num2str(i));
    agent = Aggregation(agentName,N,hp,ed,hd,batt,tank,pump,eBal,hBal);
    
    agent.connect(strcat(eBalName,'_y_src1'),strcat(hpName,'_y_e'),1); %connect components - y_sink = \gamma * y_source, connect(y_source,y_sink,\gamma)
    agent.connect(strcat(eBalName,'_y_src2'),strcat(battName,'_y_chg'),1);
    agent.connect(strcat(eBalName,'_y_src3'),strcat(edName,'_y'),1);
    agent.connect(strcat(eBalName,'_y_src4'),strcat(pumpName,'_y_e'),1);
    agent.connect(strcat(pumpName,'_y_out'),strcat(hpName,'_y_in'),4.18*4);
    agent.connect(strcat(hpName,'_y_out'),strcat(hBalName,'_y_snk1'),1);
    agent.connect(strcat(hBalName,'_y_src1'),strcat(hdName,'_y'),1);
    agent.connect(strcat(hBalName,'_y_src2'),strcat(tankName,'_y_chg'),1);
    
    inputNames = [agent.m;agent.w;agent.l];
    inputVars = agent.get_vars(inputNames);

    outputNames = [agent.uc;agent.ud;agent.y;agent.x];
    outputVars = agent.get_vars(outputNames);
    
    contName = strcat('controller_',num2str(i));
    
    conts.(matlab.lang.makeValidName(contName)) = optimizer(agent.C,agent.J,ops,inputVars,outputVars);
    
    fprintf('agent %i complete \n', i);
    toc
end

%% Input parameters - to repeat simulation without initialising agents, run from here
%state space intitial states
rng(1); %reset rng seed so that experiments can be re-run from here
input_batt = zeros(nAgents,1);
for i = 2:nAgents
    input_batt(i) = (battCap + 1)*0.2 + rand; %start each battery with random storage level
end
input_tank = zeros(nAgents,1);
for i = 2:nAgents
    input_tank(i) = 5; %start thermal storage with fixed level
end

input_pumpDH = cell(nAgents,1); %placeholder for fixed values of pump head

input_pump_stoDH = 30*ones(N,1); %fixed pressure drop through borehole thermal energy storage
input_btes = 3e8; %initial value of borehole energy storage
input_wasteH = 1600*ones(N,1); %fixed value of waste heat 

%demand and generation forecasts
ed_f = 8.5*datatable{1:nAgents*24,'ElectricityFacilitykWHourly'};
ed_f = pchip(1:60:nAgents*24*60,ed_f,1:15:nAgents*24*60)'; %interpolate for 15min intervals
hd_f = 8.5*datatable{1:24,'HeatingGaskWHourly'};
hd_f = pchip(1:60:24*60,hd_f,1:15:24*60)'; %interpolate for 15min intervals

input_load_h = zeros(96,nAgents);
for i = 2:nAgents
    input_load_h(:,i) = hd_f*((1.3-0.7)*rand + 0.7); %random noise added to each heat demand
    if i == 32 || i == 60 % cooling demand at these buildings
        input_load_h(:,i) = -input_load_h(:,i);
    end
end

%price forecasts
eBuy_f = datatable{1:24,'eBuy'};
eBuy_f = pchip(1:60:24*60,eBuy_f,1:15:24*60)'; %interpolate for 15min intervals
eSell = 1*ones(N,1);
gBuy = 2.8*ones(N,1);
cDeg = 3.15*ones(N,1);

alpha_numE = 1e-5; %algorithm step-length parameter for AC network
alpha_numH = 1e-4; %algorithm step-length parameter for heat network

ptb = unifrnd(-1e-2,1e-2,N,nAgents); %cost perturbation to avoid symmetrical system

t_max = 500;

%% Allocate memory for data logs
solution_log = cell(nAgents,sim_length);

solution_logE = cell(sim_length,1);
lambdas_logE = cell(1,length(leavesE));
sum_logE = cell(1,length(leavesE));

lambdas_logH = cell(1,length(leavesH));
sum_logH = cell(1,length(leavesH));

max_time_log = zeros(t_max,sim_length); %to log max agent computation time per iteration
time_log = zeros(t_max,sim_length); %to log energy flow computation times

%% Allocate memory for repeated AC network parameters
lambdasE = cell(1,length(leavesE)); %parameters needed for each main branch of network
musE = cell(1,length(leavesE));
s_upperE = cell(1,length(leavesE));
s_lowerE = cell(1,length(leavesE));
roE = cell(1,length(leavesE));
ro_maxE = cell(1,length(leavesE));
brE = cell(1,length(leavesE));
bE = cell(1,length(leavesE));
b_indE = cell(1,length(leavesE));
sumE = cell(1,length(leavesE));
g_logE = cell(1,length(leavesE));
gE = cell(1,length(leavesE));
ro_logE = cell(1,length(leavesE));
idxE = cell(1,length(leavesE));
deltaE = cell(1,length(leavesE));
delta_maxE = cell(1,length(leavesE));
delta_minE = cell(1,length(leavesE));
g_oldE = cell(1,length(leavesE));
lambda_oldE = cell(1,length(leavesE));
bisectE = cell(1,length(leavesE));

for j = 1:length(leavesE)
    lambdasE{j} = zeros(N,1);
    musE{j} = zeros(N,1);
    bE{j} = 5000*ones(N,1);
end

%% Allocate memory for repeated heat network parameters
lambdasH = cell(1,length(leavesH));
musH = cell(1,length(leavesH));
s_upperH = cell(1,length(leavesH));
s_lowerH = cell(1,length(leavesH));
roH = cell(1,length(leavesH));
ro_maxH = cell(1,length(leavesH));
brH = cell(1,length(leavesH));
bH = cell(1,length(leavesH));
b_indH = cell(1,length(leavesH));
sumH = cell(1,length(leavesH));
g_logH = cell(1,length(leavesH));
gH = cell(1,length(leavesH));
ro_logH = cell(1,length(leavesH));
idxH = cell(1,length(leavesH));
deltaH = cell(1,length(leavesH));
delta_maxH = cell(1,length(leavesH));
delta_minH = cell(1,length(leavesH));
g_oldH = cell(1,length(leavesH));
lambda_oldH = cell(1,length(leavesH));
bisectH = cell(1,length(leavesH));

for j = 1:length(leavesH)
    lambdasH{j} = zeros(N,1);
    musH{j} = zeros(N,1);
    bH{j} = 200*ones(N,1);
end

figure('WindowState','maximized');

    %% Simulation loop
for k = 1:sim_length
    fprintf('k = %i \n',k)
    %initialise algorithm parameters for each main branch of each network
    for j = 1:length(leavesE)
        lambdas_logE{j} = zeros(N,t_max);
        sum_logE{j} = zeros(N,t_max);
        s_upperE{j} = -inf*ones(N,nAgents);
        s_lowerE{j} = inf*ones(N,nAgents);
        roE{j} = zeros(N,nAgents);
        g_logE{j} = zeros(N,t_max);
        ro_logE{j} = zeros(N,t_max);

        lambdas_logH{j} = zeros(N,t_max);
        sum_logH{j} = zeros(N,t_max);
        s_upperH{j} = -inf*ones(N,nAgents);
        s_lowerH{j} = inf*ones(N,nAgents);
        roH{j} = zeros(N,nAgents);
        g_logH{j} = zeros(N,t_max);
        ro_logH{j} = zeros(N,t_max);
    end
    countE = zeros(length(leavesE),1);
    countH = zeros(length(leavesH),1);
    n = 0;
% begin iterations    
    for t = 1:t_max
        fprintf('Iteration %i of %i \n', t,t_max);
        %allocate memory to store variable load
        eNet_loads = cell(1,2*nAgents);
        hNet_loads = zeros(N,2*(nAgents-1));
        %reset nodal power and heat summations to zero
        for j = 1:length(leavesE)
            sumE{j} = zeros(N,1);
            sumH{j} = zeros(N,1);
        end
        %reset head loss summation to zero
        sumHBalance = zeros(N,1);
        
        max_time = 0;
        
        for i = 2:nAgents %all building agents solve local problem
            if k == 1 && t == 1 %guess of fixed pump head gain at start of simulation
                input_pumpDH{i} = zeros(N,1);
            end
            %collate system states for controllers
            inputs = {input_batt(i),input_tank(i),input_pumpDH{i}};
            
            input_load_e = ed_f(96*(i-1)+1:96*(i-1)+N);
            
            inputs = [inputs,input_load_e,input_load_h(1:N,i)];
            
            ecm_P_e1_cost = eBuy_f(1:N) + ptb(:,i);
            
            lambda_e = 0;
            lambda_h = 0;
            for j = 1:length(leavesE)
                if any(i+1 == leavesE{j})
                    lambda_e = max(lambda_e,lambdasE{j}); %assign current largest dual value of main branches
                end
            end
            for j = 1:length(leavesH)
                if any(i == leavesH{j})
                    lambda_h = max(lambda_h,lambdasH{j}); %assign current largest dual value of main branches
                end
            end
            
            inputs = [inputs,lambda_h,cDeg,ecm_P_e1_cost,eSell,lambda_e];
            
            contName = strcat('controller_',num2str(i));
            controller = conts.(matlab.lang.makeValidName(contName));
            
            [solution,diagnostics,infostr,dual,output] = controller(inputs); %solve agent optimisation problem
            
            if diagnostics > 0
                keyboard %on error
            end
            
            max_time = max(output.solvertime,max_time); %record current maximum computation time of all agents so far
            
            eNet_loads{1,2*(i-1)+1} = -solution{1,15}/1000; %convert to p.u. for power flow (MW/kW = 1/1000)
            eNet_loads{1,2*(i-1)+2} = -solution{1,15}/1000; %two values represent lower and upper bounds for energy flow, respectively
            
            hNet_loads(:,2*(i-2)+1) = -solution{1,12}/(mbase); %convert to normalised mass flow relative to nominal flow
            hNet_loads(:,2*(i-2)+2) = -solution{1,12}/(mbase);
            
            %determine total heat requirement to be balanced by plant agent
            sumHBalance = sumHBalance + solution{1,12};
            
            for j = 1:length(leavesE) %for each main branch
                if any(i+1 == leavesE{j}) %if agent is on main branch
                    sumE{j} = sumE{j} + solution{1,15}; %add agent power request to power summation for branch
                    s_upperE{j}(:,i) = max(s_upperE{j}(:,i),solution{1,15}); 
                    s_lowerE{j}(:,i) = min(s_lowerE{j}(:,i),solution{1,15});
                    roE{j}(:,i) = s_upperE{j}(:,i) - s_lowerE{j}(:,i); %determine agent's contraction term
                end
                if any(i == leavesH{j})
                    sumH{j} = sumH{j} + solution{1,12};
                    s_upperH{j}(:,i) = max(s_upperH{j}(:,i),solution{1,12});
                    s_lowerH{j}(:,i) = min(s_lowerH{j}(:,i),solution{1,12});
                    roH{j}(:,i) = s_upperH{j}(:,i) - s_lowerH{j}(:,i);
                end
            end
            
            solution_log{i,k} = solution; %log agents current solution
        end
        
        %plant agent
        if k == 1 && t == 1 %guess of fixed pump head gain at start of simulation
            input_pumpDH{1} = zeros(N,1);
        end
        ecm_P_e1_cost = eBuy_f(1:N);
        lambda_e = max(lambdasE{1},lambdasE{2}); %assign current largest dual value of main branches
        
        inputs = [input_pumpDH(1),input_pump_stoDH,input_btes,input_wasteH,sumHBalance,lambda_e,ecm_P_e1_cost];
        [solution,diagnostics,infostr,dual,output] = conts.controller_1(inputs); %plant agent solve local problem
        
        if diagnostics > 0
            keyboard %on error
        end
            
        solution_log{1,k} = solution; %log agents current solution
        
        %update bus injection and branch totals
        eNet_loads{1,1} = -solution{1,13}/1000; %convert to p.u. for power flow (MW/kW = 1/1000)
        eNet_loads{1,2} = -solution{1,13}/1000;
        sumE{1} = sumE{1} + solution{1,13}; %convert to normalised mass flow relative to nominal flow
        sumE{2} = sumE{2} + solution{1,13};
        
        for j = 1:length(leavesE)
            s_upperE{j}(:,1) = max(s_upperE{j}(:,1),solution{1,13});
            s_lowerE{j}(:,1) = min(s_lowerE{j}(:,1),solution{1,13});
            roE{j}(:,1) = s_upperE{j}(:,1) - s_lowerE{j}(:,1);
        end
        
        max_time = max(output.solvertime,max_time);
        
        max_time_log(t,k) = max_time;
        
        % check against heat flow model
        if t == 1 || t == 2 || (all(all([sumH{1} <= bH{1},sumH{2} <= bH{2}])) && any(countH < 1)) %check energy flows on first two iterations to update fixed pump head gains,... 
            %....or whenever solutions appear feasible and last heat flow was infeasible (countH < 1)
            resultsH = array2table(zeros(0,length(outputNamesHNet)),'variablenames',outputNamesHNet'); %allocate empty table for energy flow results
            tic
            for run = 1:N
                inputs = {hNet.bigM*ones(1,1)};
                inputs = [repmat(inputs,1,nAgents-1),repmat(inputs,1,nAgents)];
                inputs = [inputs,num2cell(hNet_loads(run,:))];
                
                [check,diagnostics,infostr,dual,output] = hNetSolver(inputs); %solve heat flow problem for each sampling instance separately, slight performance improvement
                
                if diagnostics > 0
                    fprintf('Diagnostics code: %i \n',diagnostics) %alert if error
                end
                
                time_log(t,k) = time_log(t,k) + output.solvertime;
                
                resultsH{run,:} = cell2mat(check);
            end
            toc
            for i = 1:nAgents
                varName_y_pumpDH = strcat('hNet_y_pumpDH',num2str(i));
                input_pumpDH{i} = max(0,resultsH{:,varName_y_pumpDH}*Hbase); %update fixed pump head gain for each agent
            end
            for j = 1:length(leavesH) %for each main branch
                count_tmp = 1; %assume feasible
                b_indH{j} = false(N,1); %initialise indicator variable
                for i = 1:length(leavesH{j}) %for each agent in branch
                    nodeName = strcat('hNet_y_phi',num2str(leavesH{j}(i)),'_z_slack');
                    if any(resultsH{1:N,nodeName} > 1e-5) %check for deviation from reference value at each node in any sampling instance
                        count_tmp = 0; %combined agent solutions infeasible
                        b_indH{j}(:) = b_indH{j}(:) + resultsH{:,nodeName} > 1e-5; %log sampling instances in which solution infeasible 
                    end
                end
                if t > 1
                    countH(j) = countH(j) + count_tmp; %add number of feasible energy flow problems to counter
                end
                brH{j} = zeros(N,1);
                for i = 1:length(leavesH{j}) %for each agent in branch
                    nodeName = strcat('hNet_y_phi',num2str(leavesH{j}(i)));
                    brH{j}(:) = brH{j}(:) - resultsH{:,nodeName}*(mbase); %obtain branch capacity limit to match nearest feasible energy flow solution
                end
                
                if any(b_indH{j})
                    for i = 1:N
                        if b_indH{j}(i) %for sampling instances in which energy flow solution deviated from agents reference
                            bH{j}(i) = min(bH{j}(i),brH{j}(i)); %update branch capacity limit to match that of feasible solution
                        end
                    end
                end
            end
            n = 0; %reset counter for dual iterations
            countE(:) = 0; %ensure power flow is re-checked to account for heat network changes
        end
        
        % check against load flow model (same as heat network above)
        if t == 1 || t == 2 || (all(all([sumE{1} <= bE{1},sumE{2} <= bE{2}])) && any(countE < 1))
            inputs = {eNet.bigM*ones(N,1)};
            inputs = repmat(inputs,1,nAgents);
            inputs = [inputs,eNet_loads];
            tic
            [check,diagnostics,infostr,dual,output] = eNetSolver(inputs);
            toc
            if diagnostics > 0
                fprintf('Diagnostics code: %i \n',diagnostics)
            end
            
            time_log(t,k) = output.solvertime;
            
            resultsE = table(check{:},'variablenames',outputNamesENet');
            solution_logE{k} = resultsE;
            
            for j = 1:length(leavesE)
                count_tmp = 1;
                b_indE{j} = false(N,1);
                for i = 1:length(leavesE{j})
                    nodeName = strcat('eNet_y_p',num2str(leavesE{j}(i)),'_z_slack');
                    if any(resultsE{1:N,nodeName} > 1e-5)
                        count_tmp = 0;
                        b_indE{j}(:) = b_indE{j}(:) + resultsE{:,nodeName} > 1e-5;
                    end
                end
                if t > 1
                    countE(j) = countE(j) + count_tmp;
                end
                brE{j} = zeros(N,1);
                for i = 1:length(leavesE{j})
                    nodeName = strcat('eNet_y_p',num2str(leavesE{j}(i)));
                    brE{j}(:) = brE{j}(:) - resultsE{:,nodeName}*1000;
                end
                
                if any(b_indE{j})
                    for i = 1:N
                        if b_indE{j}(i)
                            bE{j}(i) = min(bE{j}(i),brE{j}(i));
                        end
                    end
                end
            end
            n = 0;
        end
        
        n = n + 1; %increase counter for dual iterations
        
        for j = 1:length(leavesE)
            % determine power flow dual multipliers
            ro_maxE{j} = zeros(N,1);
            for i = 1:N
                ro_maxE{j}(i) = sum(maxk(roE{j}(i,:),N,2)); %sum of N largest subproblem budget consumption changes
            end
            ro_logE{j}(:,t) = ro_maxE{j}; %log contraction
            
            gE{j} = sumE{j}(:) - bE{j}(:) + ro_maxE{j}(:); %determine subgradients
            
            % determine heat flow dual multipliers
            ro_maxH{j} = zeros(N,1);
            for i = 1:N
                ro_maxH{j}(i) = sum(maxk(roH{j}(i,:),N,2)); %sum of N largest subproblem budget consumption changes
            end
            ro_logH{j}(:,t) = ro_maxH{j}; %log contraction
            
            gH{j} = sumH{j}(:) - bH{j}(:) + ro_maxH{j}(:); %determine subgradients

            m = n/(n+3); %determine diminishing acceleration parameter
            musE_old = musE{j}; %store previous value
            musE{j} = max(zeros(N,1),lambdasE{j} + alpha_numE.*gE{j}); %update mu based on subgradients
            lambdasE{j} = musE{j} + m*(musE{j} - musE_old); %update lambda based on acceleration

            musH_old = musH{j};
            musH{j} = max(zeros(N,1),lambdasH{j} + alpha_numH.*gH{j});
            lambdasH{j} = musH{j} + m*(musH{j} - musH_old);
            
            %log values
            lambdas_logE{j}(:,t) = lambdasE{j};
            g_logE{j}(:,t) = gE{j};
            sum_logE{j}(:,t) = sumE{j};
            
            lambdas_logH{j}(:,t) = lambdasH{j};
            g_logH{j}(:,t) = gH{j};
            sum_logH{j}(:,t) = sumH{j};
        end
        %plots to see iteration progress
        
        x = 1:t;
        for w = 1:N
            subplot(4,1,1);
            plot(lambdas_logE{1,1}(w,x))
            hold on
        end
        title('Dual Updates')
        ylabel('lambda for AC branch 1')
        hold off
        for w = 1:N
            subplot(4,1,2);
            plot(lambdas_logE{1,2}(w,x))
            hold on
        end
        ylabel('lambda for AC branch 2')
        hold off
        for w = 1:N
            subplot(4,1,3);
            plot(lambdas_logH{1,1}(w,x))
            hold on
        end
        ylabel('lambda for heat branch 1')
        hold off
        for w = 1:N
            subplot(4,1,4);
            plot(lambdas_logH{1,2}(w,x))
            hold on
        end
        ylabel('lambda for heat branch 2')
        hold off
        xlabel('Iteration, t')
        pause(0.01)
        
        %check for exactness in energy flow solutions
        if all(countE > 0) && all(countH > 0)
            for i = 1:length(eNet.arcs)
                PName = strcat('eNet_z_P',eNet.arcs{i});
                QName = strcat('eNet_z_Q',eNet.arcs{i});
                IName = strcat('eNet_z_I',eNet.arcs{i});
                vName = strcat('eNet_y_v',extractBefore(eNet.arcs{i},'_'));
                dev = resultsE{:,IName}.*resultsE{:,vName} - resultsE{:,PName}.^2 - resultsE{:,QName}.^2;
                if dev > 1e-4
                    keyboard %if not exact
                end
            end
            for i = 1:length(hNet.arcs)
                PhiName = strcat('hNet_z_Phi',hNet.arcs{i});
                pipeDHName = strcat('hNet_z_pipeDH',hNet.arcs{i});
                iIdx = str2double(extractBefore(hNet.arcs{i},'_'));
                jIdx = str2double(extractAfter(hNet.arcs{i},'_'));
                dev = (resultsH{:,pipeDHName} - adjMatrix(iIdx,jIdx)*(resultsH{:,PhiName}).^2);
                if dev > 1e-4
                    keyboard %if not exact
                end
            end
            break
        end
    end
    
    %iterate inputs
    ptb = circshift(ptb,-1,1);
    ed_f = circshift(ed_f,-1);
    input_load_h = circshift(input_load_h,-1,1);
    eBuy_f = circshift(eBuy_f,-1);
    
    for j = 1:length(input_pumpDH) 
        input_pumpDH{j} = circshift(input_pumpDH{j},-1);
        input_pumpDH{j}(end) = 0;
    end
    
    for j = 1:length(leavesE)
        lambdasE{j} = circshift(lambdasE{j},-1);
        lambdasE{j}(end) = 0;
        musE{j} = circshift(musE{j},-1);
        musE{j}(end) = 0;
        bE{j} = circshift(bE{j},-1);
        bE{j}(end) = 5000;
    end
    
    for j = 1:length(leavesH)
        lambdasH{j} = circshift(lambdasH{j},-1);
        lambdasH{j}(end) = 0;
        musH{j} = circshift(musH{j},-1);
        musH{j}(end) = 0;
        bH{j} = circshift(bH{j},-1);
        bH{j}(end) = 200;
    end
    
    %update system storage states from solution
    solution_tmp = solution_log{1,k};
    input_btes = alphaBorehole*input_btes + dT*solution_tmp{1}(1,1);
    for i = 2:nAgents
        solution_tmp = solution_log{i,k};
        if solution_tmp{2}(1,1) > 0
            input_batt(i) = alphaBatt*input_batt(i) + etaChg(i)*dT*solution_tmp{2}(1,1);
        else
            input_batt(i) = alphaBatt*input_batt(i) + 1/etaDchg(i)*dT*solution_tmp{2}(1,1);
        end
        input_tank(i) = alphaTank*input_tank(i) + dT*solution_tmp{3}(1,1);
    end
end
    

%% Costs
cost = 0;
eBuy_f = datatable{1:24,'eBuy'};
eBuy_f = pchip(1:60:24*60,eBuy_f,1:15:24*60)'; %interpolate for 15min intervals
for k = 1:96
    solution = solution_log{1,k};
    if solution{1,13} >= 0
        cost = cost + solution{1,13}(1)*eBuy_f(k);
    else
        cost = cost + solution{1,13}(1)*eSell_f(1);
    end
    for j = 2:nAgents
        solution = solution_log{j,k};
        if solution{1,15}(1) >= 0
            cost = cost + solution{1,15}(1)*eBuy_f(k) + abs(solution{1,2}(1))*cDeg(1);
        else
            cost = cost + solution{1,15}(1)*eSell(1) + abs(solution{1,2}(1))*cDeg(1);
        end
    end
end