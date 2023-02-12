classdef Component < handle
    %Abstract class for all MLD system objects
        
    properties (SetAccess=protected , GetAccess=public)
        %basic properties used by all objects:
        name            %name (string)     
        N               %MPC prediction horizon length (scalar)
        
        %shared reference to unique YALMIP variables:
        vars            %stores all symbolic YALMIP variables associated with object (struct)
        
        %objects required by YALMIP optimizer:
        C               %constraints (yalmip lmi object)
        J               %MPC cost objective (yalmip sdpvar object)
        
        %Constrained decision variable names:
        x               %MLD system state variables (cell array)
        uc              %MLD system continuous control variables (cell array)
        ud              %MLD system binary control variables (cell array)
        z               %MLD system continuous auxiliary variables (cell array)
        d               %MLD system integer auxiliary variables (cell array)
        y               %MLD system output variables (cell array)
        
        %named inputs to YALMIP optimizer:
        w               %MLD system known disturbance variables (cell array)
        l               %MPC cost variables (cell array)
        m               %MLD system measurement variables for time dependent constraints (cell array)
        
        %properties to define connections:
        snkPorts         %MLD system output variables at source ports (cell array)
        srcPorts         %MLD system output variables at source ports (cell array)

        %internal parameters:
        bigM            %Default maximum variable value for component object (scalar)
        
        %properties to define hierarchical object structure:
        subComponents   %sub-components within object (cell array)
        supComponent    %super-component that object is within (cell array)
        connected       %boolean value to indicate if component has been connected
    end
    
    methods (Abstract)
        C = get_C(self) %all children of component class must specify a constraint set
    end
    
    methods
        %Constructor
        function self = Component(name,N,varargin)
            
            %set object's name and horizon
            self.name = name;
            self.N = N;
            %initialise remaining properties
            self.vars = struct;
            self.C = [];
            self.J = 0;
            self.x = {};
            self.uc = {};
            self.ud = {};
            self.z = {};
            self.d = {};
            self.y = {};
            self.w = {};
            self.l = {};
            self.m = {};
            self.srcPorts = {};
            self.snkPorts = {};
            
            if ~isempty(varargin)
                self.subComponents = varargin;%assign list of sub-components
                for i = 1:length(self.subComponents)
                    self.subComponents{i}.supComponent = self;
                end
            else
                self.subComponents = {};
            end
            
            self.supComponent = {};
            self.connected = false;
        end
        
        function Vars = get_vars(self,varNames,varargin)
            %obtains a reference to the sdpvar in vars struct for a given
            %varName. Works for cell array of varNames or single varName.
            %Uses optional varagin to specify index of sdpvar over horizon 
            %length, scalar or vector indexing allowed.
            
            if iscell(varNames)
                Vars = cell(1,length(varNames));
                for i = 1:length(varNames)
                    Vars{1,i} = self.vars.(matlab.lang.makeValidName(varNames{i}));
                end
            else
                Vars = self.vars.(matlab.lang.makeValidName(varNames));
            end
            
            if length(varargin) == 1
                idx = varargin{1};
                if iscell(varNames)
                    for i = 1:size(Vars,2)
                        Vars{1,i} = Vars{1,i}(idx,:);
                    end
                else
                    Vars = Vars(idx,:);
                end
            end
        end
        
        function change_bounds(self,varName,newMin,newMax)
            %changes the bounds on variable var. Not allowed on binary
            %variables.
            
            try
            var = self.vars.(matlab.lang.makeValidName(varName)); %find variable sdpvar
            catch
                disp('change_bounds(): Variable does not exist.')
                return
            end
            
            try
                self.C([varName,'_bounds']) = [(newMin <= var <= newMax):strcat(varName,'_user_bounds')];
            catch
                disp('Error, new min and max constraints not added.')
            end
        end

        function add_slack(self,varName,prefMin,prefMax,penalty)
            %adds slack variable, additional bounds with slack variable and
            %slack term in objective. Used in conjunction with
            %change_bounds() (hard constraints) to define maximum deviation 
            %from prefMin and prefMax.
            
            try
            var = self.vars.(matlab.lang.makeValidName(varName)); %find variable sdpvar
            catch
                fprintf('Error in add_slack(): %s does not exist.\n',varName)
                return
            end
            
            %create slack variable
            varName_z_slack = [varName,'_z_slack'];
            self.vars.(matlab.lang.makeValidName(varName_z_slack)) = sdpvar(self.N,1);
            var_z_slack = self.vars.(matlab.lang.makeValidName(varName_z_slack));
            self.z = [self.z;varName_z_slack];
            
            %create varying min/max preference variables if preferences are
            %empty
            if isempty(prefMin)
                varName_m_prefMin = [varName,'_m_prefMin'];
                self.vars.(matlab.lang.makeValidName(varName_m_prefMin)) = sdpvar(self.N,1);
                var_m_prefMin = self.vars.(matlab.lang.makeValidName(varName_m_prefMin));
                self.m = [self.m;varName_m_prefMin];
            end
            if isempty(prefMax)
                varName_m_prefMax = [varName,'_m_prefMax'];
                self.vars.(matlab.lang.makeValidName(varName_m_prefMax)) = sdpvar(self.N,1);
                var_m_prefMax = self.vars.(matlab.lang.makeValidName(varName_m_prefMax));
                self.m = [self.m;varName_m_prefMax];
            end
            
            %ensure slack variable is positive
            self.C = self.C + [(0 <= var_z_slack):strcat(varName,'_slack_positivity')];
            
            %add soft constraints
            if ~isempty(prefMin) && ~isempty(prefMax)
                self.C = self.C + [(prefMin - var_z_slack <= var <= prefMax + var_z_slack):strcat(varName,'_slack')];
            elseif ~isempty(prefMin) && isempty(prefMax)
                self.C = self.C + [(prefMin - var_z_slack <= var <= var_m_prefMax + var_z_slack):strcat(varName,'_slack')];
            elseif isempty(prefMin) && ~isempty(prefMax)
                self.C = self.C + [(var_m_prefMin - var_z_slack <= var <= prefMax + var_z_slack):strcat(varName,'_slack')];
            else
                self.C = self.C + [(var_m_prefMin - var_z_slack <= var <= var_m_prefMax + var_z_slack):strcat(varName,'_slack')];
            end
            
            %add penalty term in objective function
            self.J = self.J + penalty*ones(1,self.N)*var_z_slack;
        end
        
        function connect(self,varNameSrc,varNameSnk,coef)
            %adds a constraint var1 == coef*var2
            
            try
            varSrc = self.vars.(matlab.lang.makeValidName(varNameSrc)); %find variable sdpvar
            catch
                disp('connect():')
                disp(varNameSrc)
                disp('Variable does not exist.')
                return
            end
            try
            varSnk = self.vars.(matlab.lang.makeValidName(varNameSnk)); %find variable sdpvar
            catch
                disp('connect():')
                disp(varNameSnk)
                disp('Variable does not exist.')
                return
            end
            
            assert(any(strcmp(self.srcPorts,varNameSrc)),'Connections must be made from source ports to sink ports.')
            assert(any(strcmp(self.snkPorts,varNameSnk)),'Connections must be made from source ports to sink ports.')
            
            self.C = self.C + [(varSnk == coef*varSrc):strcat('connect_',varNameSrc,'_to_',varNameSnk)];
            
            for i = 1:length(self.subComponents)
                if any(strcmp(varNameSrc,fieldnames(self.subComponents{i}.vars)))
                    self.subComponents{i}.connected = true;
                end
                if any(strcmp(varNameSnk,fieldnames(self.subComponents{i}.vars)))
                    self.subComponents{i}.connected = true;
                end
            end
        end
        
        function add_val_cost(self,varName)
            %adds a cost sdpvar, var_l_val, and additional term in objective
            %function
            
            try
            var = self.vars.(matlab.lang.makeValidName(varName)); %find variable sdpvar
            catch
                disp('add_val_cost():')
                disp(varName)
                disp('Variable does not exist.')
                return
            end
               
            varName_l_val = [varName,'_l_val'];
            self.vars.(matlab.lang.makeValidName(varName_l_val)) = sdpvar(self.N,1); %create cost variable
            var_l_val = self.vars.(matlab.lang.makeValidName(varName_l_val)); %find cost variable sdpvar
            
            if size(var,2) > 1 %need to provide a scalar objective term
                self.J = self.J + var_l_val'*var*ones(self.N,1);
            else
                self.J = self.J + var_l_val'*var;
            end
            
            self.l = [self.l;varName_l_val]; %add reference in l
        end
        
        function add_abs_cost(self,varName)
            %adds cost sdpvars, var_l_abs and var_z_abs, additional term 
            %in objective function for cost applied to absolute value of 
            %variable and constraint defining absolute variable.
            
            try
            var = self.vars.(matlab.lang.makeValidName(varName)); %find variable sdpvar
            catch
                disp('add_abs_cost():')
                disp(varName)
                disp('Variable does not exist.')
                return
            end
               
            varName_l_abs = [varName,'_l_abs'];
            self.vars.(matlab.lang.makeValidName(varName_l_abs)) = sdpvar(self.N,1); %create cost variable
            var_l_abs = self.vars.(matlab.lang.makeValidName(varName_l_abs)); %find cost variable sdpvar
            
            varName_z_abs = [varName,'_z_abs'];
            self.vars.(matlab.lang.makeValidName(varName_z_abs)) = sdpvar(self.N,1); %create cost variable
            var_z_abs = self.vars.(matlab.lang.makeValidName(varName_z_abs)); %find cost variable sdpvar
            
            self.C = self.C + [(var_z_abs == abs(var)):strcat(varName,'_abs')];
            
            self.J = self.J + var_l_abs'*var_z_abs;
            
            self.l = [self.l;varName_l_abs]; %add reference in l
            self.z = [self.z;varName_z_abs]; %add reference in z
        end
        
        function add_buy_sell_cost(self,varName)
            %adds cost sdpvars, var_cost and var_aux, additional term 
            %in objective function for cost applied to auxiliary variables 
            %and constraint defining binary and auxiliary variables.
            
            try
            var = self.vars.(matlab.lang.makeValidName(varName)); %find variable sdpvar
            catch
                disp('add_buy_sell_cost():')
                disp(varName)
                disp('Variable does not exist.')
                return
            end
            
            varName_l_buy = [varName,'_l_buy'];
            self.vars.(matlab.lang.makeValidName(varName_l_buy)) = sdpvar(self.N,1); %create cost variable
            var_l_buy = self.vars.(matlab.lang.makeValidName(varName_l_buy)); %find cost variable sdpvar
            
            varName_l_sell = [varName,'_l_sell'];
            self.vars.(matlab.lang.makeValidName(varName_l_sell)) = sdpvar(self.N,1); %create cost variable
            var_l_sell = self.vars.(matlab.lang.makeValidName(varName_l_sell)); %find cost variable sdpvar
            
            varName_z_buy = [varName,'_z_buy'];
            self.vars.(matlab.lang.makeValidName(varName_z_buy)) = sdpvar(self.N,1); %create aux variable
            var_z_buy = self.vars.(matlab.lang.makeValidName(varName_z_buy)); %find cost variable sdpvar
            
            varName_z_sell = [varName,'_z_sell'];
            self.vars.(matlab.lang.makeValidName(varName_z_sell)) = sdpvar(self.N,1); %create aux variable
            var_z_sell = self.vars.(matlab.lang.makeValidName(varName_z_sell)); %find cost variable sdpvar
            
            varName_d_buy = [varName,'_d_buy'];
            self.vars.(matlab.lang.makeValidName(varName_d_buy)) = binvar(self.N,1); %create binary variable
            var_d_buy = self.vars.(matlab.lang.makeValidName(varName_d_buy)); %find cost variable sdpvar
            
            self.C = self.C + [(-self.bigM + self.bigM*var_d_buy <= var <= self.bigM*var_d_buy):varName_d_buy];
            self.C = self.C + [(eps*var_d_buy <= var_z_buy <= self.bigM*var_d_buy):varName_z_buy];
            self.C = self.C + [(eps*var_d_buy <= var_z_sell <= self.bigM - self.bigM*var_d_buy):varName_z_sell];
            self.C = self.C + [(var == var_z_buy - var_z_sell):strcat(varName,'_aux_excl')];
            
            self.J = self.J + var_l_buy'*var_z_buy - var_l_sell'*var_z_sell;
            
            self.l = [self.l;varName_l_buy;varName_l_sell]; %add reference in l
            self.z = [self.z;varName_z_buy;varName_z_sell]; %add reference in l
            self.d = [self.d;varName_d_buy];
        end
        
        function add_dual_cost(self,varName_l_dual,varCoefs,varNameArray)
            %adds a dual multiplier variable which is multiplied by
            %varCoefs*varArray, representing the associated constraint
            %terms
            
            if isfield(self.vars,varName_l_dual) %see if proposed sdpvar name already used
                disp('Variable name in use.')
                return
            else
                self.vars.(matlab.lang.makeValidName(varName_l_dual)) = sdpvar(self.N,1); %create dual multiplier variable
                var_l_dual = self.vars.(matlab.lang.makeValidName(varName_l_dual));
            end
            
            assert(length(varCoefs) == length(varNameArray),'Vector of variable coefficients and variable name cell array must be the same length');
            
            tmp_J = self.J;
            try
                for i = 1:length(varCoefs)
                    varName = varNameArray{i};
                    var = self.vars.(matlab.lang.makeValidName(varName));
                    varCoef = varCoefs(i)*ones(self.N,1);
                    tmp_J = tmp_J + var_l_dual'.*varCoef'*var;
                end
            catch
                disp(strcat('Variable ',varName,' does not exist.'))
                return
            end 
            
            self.J = tmp_J;
            self.l = [self.l;varName_l_dual];
        end
        
        function add_on_off(self,varName,varMin,varMax)
            %adds constraints to enforce minimum power output.
            
            try
                var = self.vars.(matlab.lang.makeValidName(varName)); %find variable sdpvar
            catch
                disp('add_on_off():')
                disp(varName)
                disp('Variable does not exist.')
                return
            end
            
            varName_ud_on = [varName,'_ud_on'];
            self.vars.(matlab.lang.makeValidName(varName_ud_on)) = binvar(self.N,1); %create binary variable
            var_ud_on = self.vars.(matlab.lang.makeValidName(varName_ud_on));
            
            %set binary variable value based on var
            self.C = self.C + [(var_ud_on*varMin <= var <= var_ud_on*varMax):varName_ud_on]; %(8)
            
            self.ud = [self.d;varName_ud_on];
        end
        
        function add_ramp(self,varName,rampUp,rampDown)
            %adds ramp constraints to named variable based on previous
            %values.
            
            try
                var = self.vars.(matlab.lang.makeValidName(varName)); %find variable sdpvar
            catch
                disp('add_ramp():')
                disp(varName)
                disp('Variable does not exist.')
                return
            end
            
            varName_z_ramp = [varName,'_z_ramp'];
            self.vars.(matlab.lang.makeValidName(varName_z_ramp)) = sdpvar(self.N,1); %create ramp variable
            var_z_ramp = self.vars.(matlab.lang.makeValidName(varName_z_ramp));
            
            self.z = [self.z;varName_z_ramp];
            
            varName_m = [varName,'_m'];
            try
                var_m = self.vars.(matlab.lang.makeValidName(varName_m));
            catch
                self.vars.(matlab.lang.makeValidName(varName_m)) = sdpvar(1,1); %create current output status variable
                var_m = self.vars.(matlab.lang.makeValidName(varName_m));
                
                self.m = [self.m;varName_m];
            end
            
            %simple ramp constraints
            self.C = self.C + [(-rampDown <= var_z_ramp <= rampUp):strcat(varName,'_ramp_limits')];
            
            self.C = self.C + [(var(1) == var_m + var_z_ramp(1)):strcat(varName,'_ramp_1')];
            for k = 2:self.N
                self.C = self.C + [(var(k) == var(k-1) + var_z_ramp(k)):strcat(varName,'_ramp_',num2str(k))];
            end
        end
                
        function add_min_up_down(self,varName,minUp,minDown)
            %adds minimum up and down time to specified variable based on
            %specified inputs - time varying indicator inputs which
            %determine initial set of up and down constraints
            
            try
                var = self.vars.(matlab.lang.makeValidName(varName)); %find variable sdpvar
            catch
                disp('add_min_up_down():')
                disp(varName)
                disp('Variable does not exist.')
                return
            end
            
            varName_ud_on = [varName,'_ud_on'];
            try
                var_ud_on = self.vars.(matlab.lang.makeValidName(varName_ud_on));
            catch
                disp('No on_off status variable found to determine start up and shut down status.')
                disp('Please first use add_on_off() method to add this variable and associated constraint.')
                return
            end
            
            varName_m_on = [varName,'_on_status'];
            self.vars.(matlab.lang.makeValidName(varName_m_on)) = binvar(1,1); %create binary variable
            var_m_on = self.vars.(matlab.lang.makeValidName(varName_m_on));
            
            varName_m_up = [varName,'_min_up'];
            self.vars.(matlab.lang.makeValidName(varName_m_up)) = sdpvar(minUp-1,1); %create input indicator variable
            var_m_up = self.vars.(matlab.lang.makeValidName(varName_m_up));
            
            varName_m_down = [varName,'_min_down'];
            self.vars.(matlab.lang.makeValidName(varName_m_down)) = sdpvar(minDown-1,1); %create input indicator variable
            var_m_down = self.vars.(matlab.lang.makeValidName(varName_m_down));
                
            self.m = [self.m;varName_m_on;varName_m_up;varName_m_down];
            
            %minimum up time constraints
            range = 1:minUp - 1;
            self.C = self.C + [(var_ud_on(range) >= var_m_up):strcat(varName,'_min_up_0')];
            
            indicator = var_ud_on(1) - var_m_on;
            range = 1:min(self.N,minUp);
            self.C = self.C + [(var_ud_on(range) >= indicator):strcat(varName,'_min_up_1')];
            
            for k = 2:self.N
                indicator = var_ud_on(k) - var_ud_on(k-1);
                range = k:min(self.N,k+minUp-1);
                self.C = self.C + [(var_ud_on(range) >= indicator):strcat(varName,'_min_up_',num2str(k))];
            end
            
            %minimum down time constraints
            range = 1:minDown - 1;
            self.C = self.C + [(var_ud_on(range) <= 1 - var_m_down):strcat(varName,'_min_down_0')];
            
            indicator = var_m_on - var_ud_on(1);
            range = 1:min(self.N,minDown);
            self.C = self.C + [(var_ud_on(range) <= 1 - indicator):strcat(varName,'_min_down_1')];
            
            for k = 2:self.N
                indicator = var_ud_on(k-1) - var_ud_on(k);
                range = k:min(self.N,k+minDown-1);
                self.C = self.C + [(var_ud_on(range) <= 1 - indicator):strcat(varName,'_min_down_',num2str(k))];
            end
        end  
    end
    
    methods(Static)
        function Vars = merge_vars(vars1,vars2)
            %joins two distinct vars structs into a single vars struct
            Vars = cell2struct([struct2cell(vars1);struct2cell(vars2)],[fieldnames(vars1);fieldnames(vars2)]);
        end
    end
end