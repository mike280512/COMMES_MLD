classdef GasNetworkModel < Component
    
    % GasNetworkModel  A quadratically constrained network model
    % to connect hubs within a district. The model requires an adjacency 
    % matrix to be instantiated.  The adjacency matrix must be square. 
    
    properties (SetAccess = private, GetAccess = public)
        adjMatrix               %adjacency matrix (matrix)
        arcs                    %connecting arcs (cell array)
        pr1                     %reference pressure (scalar)
        penalty                 %penalty to ensure exactness (scalar)
    end
    
    
    methods
        
        function self = GasNetworkModel(name,N,bigM,adjMatrix,pr1,penalty)
            
            %call parent class constructor
            self@Component(name,N)
            
            [r,c] = size(adjMatrix);
            
            %validate input params
            assert(r == c,'Node connection matrix must be square')
            assert(~any(diag(adjMatrix)),'Diagonal of adjMatrix matrix must be zero.  Nodes cannot be connected to themselves.')
            
            %initialise and set object properties
            self.adjMatrix = adjMatrix;
            self.bigM = bigM;
            self.arcs = self.get_arcs;
            self.pr1 = pr1;
            self.penalty = penalty;
            
            %initialise variables
            self.y = self.get_y;
            self.z = self.get_z;
            self.d = self.get_d;
            self.m = self.get_m;
            
            %initialise constraints
            self.C = self.get_C;
            
            %add terms to objective function to ensure exact relaxation
            self.J = self.get_J;
        end%constructor
        
        
        %% OBJECT PROPERTY GET METHODS
                
        function arcs = get_arcs(self)
            %returns cell array of all connecting arcs
            arcs = cell(nnz(self.adjMatrix),1);
            idx = 1;
            [r,c] = size(self.adjMatrix);
            %find all non-zero elements in adjMatrix, label and store
            %position
            for i = 1:r
                for j = 1:c
                    if self.adjMatrix(i,j)>0
                        arcName = strcat(num2str(i),'_',num2str(j));
                        arcs{idx} = arcName;
                        idx = idx + 1;
                    end
                end
            end
        end
        
        function y = get_y(self)
            %initialises MLD system output variables and returns cell array
            y = self.y;
            
            varNames_y_f = cell(size(self.adjMatrix,1),1);
            varNames_y_pr = cell(size(self.adjMatrix,1),1);

            for i = 1:length(varNames_y_f)
                varName_y_f = [self.name,'_y_f',num2str(i)]; %define variable name
                varName_y_pr = [self.name,'_y_pr',num2str(i)];
                
                self.vars.(matlab.lang.makeValidName(varName_y_f)) = sdpvar(self.N,1); %define sdpvar in vars struct
                self.vars.(matlab.lang.makeValidName(varName_y_pr)) = sdpvar(self.N,1);
                
                varNames_y_f{i} = varName_y_f;
                varNames_y_pr{i} = varName_y_pr;
            end
            
            y = [y;varNames_y_f;varNames_y_pr];
            
            self.srcPorts = varNames_y_f;
        end
        
        function z = get_z(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            z = self.z;
            
            varNames_z_F = cell(length(self.arcs),1);
            varNames_z_dp = cell(length(self.arcs),1);
            
            for i = 1:length(self.arcs)
                varName_z_F = [self.name,'_z_F',self.arcs{i}]; %define variable name
                varName_z_dp = [self.name,'_z_dp',self.arcs{i}];
                
                self.vars.(matlab.lang.makeValidName(varName_z_F)) = sdpvar(self.N,1); %define sdpvar in vars struct
                self.vars.(matlab.lang.makeValidName(varName_z_dp)) = sdpvar(self.N,1);
                
                varNames_z_F{i} = varName_z_F;
                varNames_z_dp{i} = varName_z_dp;
            end
            
            z = [z;varNames_z_F;varNames_z_dp];
        end
        
        function d = get_d(self)
            %initialies MLD system continuous binary variables and returns cell array
            d = self.d;
            
            varNames_d_fwd = cell(length(self.arcs),1);
            
            for i = 1:length(self.arcs)
                varName_d_fwd = [self.name,'_d_fwd',self.arcs{i}]; %define variable name
                
                self.vars.(matlab.lang.makeValidName(varName_d_fwd)) = binvar(self.N,1); %define sdpvar in vars struct
                
                varNames_d_fwd{i} = varName_d_fwd;
            end
            
            d = [d;varNames_d_fwd];
        end
        
        function m = get_m(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            m = self.m;
            
            varNames_m_maxDP = cell(length(self.arcs),1);
            
            for i = 1:length(self.arcs)
                varName_m_maxDP = [self.name,'_m_maxDP',self.arcs{i}]; %define variable name            
                
                self.vars.(matlab.lang.makeValidName(varName_m_maxDP)) = sdpvar(self.N,1); %define sdpvar in vars struct
                
                varNames_m_maxDP{i} = varName_m_maxDP;
            end
            
            m = [m;varNames_m_maxDP];
        end
        
        function C = get_C(self)
            %construct constraints for network model
            C = self.C; %constraints are added to existing lmi
            [r,c] = size(self.adjMatrix);
            
            %flow nodal balances
            for i = 1:r    %iterate through nodes
                varName_y_f = [self.name,'_y_f',num2str(i)]; %get name of node variable
                var_y_f = self.vars.(matlab.lang.makeValidName(varName_y_f)); %get node variable
                zVars = [];
                coeffs = [];
                for j = 1:c %iterate through each column of adjMatrix matrix
                    if self.adjMatrix(i,j)>0   %if element at index = (node,current column) is greater than 0
                        varName_z_F = [self.name,'_z_F',num2str(i),'_',num2str(j)]; %get name of adjacent arc variable
                        
                        var_z_F = self.vars.(matlab.lang.makeValidName(varName_z_F));
                        
                        zVars = [zVars, var_z_F]; %get adjacent arc variable
                        coeffs = [coeffs; 1]; %assign coefficient
                    end
                    if self.adjMatrix(j,i)>0
                        varName_z_F = [self.name,'_z_F',num2str(j),'_',num2str(i)];
                        
                        var_z_F = self.vars.(matlab.lang.makeValidName(varName_z_F));
                        
                        zVars = [zVars, var_z_F]; %get adjacent arc variable
                        coeffs = [coeffs; -1]; %assign arc conversion factor as coefficient
                    end
                end
                C = C + [(var_y_f == zVars*coeffs):strcat(varName_y_f,'_balance')]; %tagging is used to identify constraints
            end
            
            %add reference pressure at node 1
            varName_y_pr1 = [self.name,'_y_pr1'];
            var_y_pr1 = self.vars.(matlab.lang.makeValidName(varName_y_pr1));
            C = C + [(var_y_pr1 == self.pr1):strcat(varName_y_pr1,'_reference')];
                
            %add pressure drop relaxation constraints
            for i = 1:r
                for j = 1:c
                    if self.adjMatrix(i,j) > 0
                        K = self.adjMatrix(i,j);
                        
                        varName_y_pri = [self.name,'_y_pr',num2str(i)];
                        varName_y_prj = [self.name,'_y_pr',num2str(j)];
                        varName_z_F = [self.name,'_z_F',num2str(i),'_',num2str(j)];
                        varName_z_dp = [self.name,'_z_dp',num2str(i),'_',num2str(j)];
                        varName_d_fwd = [self.name,'_d_fwd',num2str(i),'_',num2str(j)];
                        
                        var_y_pri = self.vars.(matlab.lang.makeValidName(varName_y_pri));
                        var_y_prj = self.vars.(matlab.lang.makeValidName(varName_y_prj));
                        var_z_F = self.vars.(matlab.lang.makeValidName(varName_z_F));
                        var_z_dp = self.vars.(matlab.lang.makeValidName(varName_z_dp));
                        var_d_fwd = self.vars.(matlab.lang.makeValidName(varName_d_fwd));
                        
                        C = C + [(-self.bigM*(1 - var_d_fwd) <= var_z_F <= self.bigM*var_d_fwd):varName_d_fwd];
                        C = C + [(var_y_prj - var_y_pri - 2*self.bigM*var_d_fwd <= var_z_dp):strcat(varName_z_dp,'_relaxation_l_rev')];
                        C = C + [(var_y_pri - var_y_prj - 2*self.bigM*(1 - var_d_fwd) <= var_z_dp):strcat(varName_z_dp,'_relaxation_l_fwd')];
                        C = C + [(var_z_dp <= var_y_prj - var_y_pri + 2*self.bigM*var_d_fwd):strcat(varName_z_dp,'_relaxation_u_rev')];
                        C = C + [(var_z_dp <= var_y_pri - var_y_prj + 2*self.bigM*(1 - var_d_fwd)):strcat(varName_z_dp,'_relaxation_u_fwd')];
                        
                        C = C + [(K*var_z_F.^2 <= var_z_dp):strcat(varName_z_dp,'_SOCP')]; %suitable for gurobi
                        %other solvers may require the yalmip norm
                        %operator, implemented as follows for each sampling
                        %interval:
                        %for k = 1:self.N
%                           C = C + [(norm([2*sqrt(K)*var_z_F(k); var_z_dp(k) - 1],2) <= var_z_dp(k) + 1):strcat(varName_z_dp,'_SOCP')];
                        %end
                    end
                end
            end
            
            %add default bounds
            for i = 1:r
                varName_y_f = [self.name,'_y_f',num2str(i)];
                varName_y_pr = [self.name,'_y_pr',num2str(i)];
                
                var_y_f = self.vars.(matlab.lang.makeValidName(varName_y_f));
                var_y_pr = self.vars.(matlab.lang.makeValidName(varName_y_pr));
                
                C = C + [(-self.bigM <= var_y_f <= self.bigM):strcat(varName_y_f,'_bounds')];
                C = C + [(0 <= var_y_pr <= self.bigM):strcat(varName_y_pr,'_bounds')];
            end
            for i = 1:r
                for j = 1:c
                    if self.adjMatrix(i,j) > 0
                        varName_z_F = [self.name,'_z_F',num2str(i),'_',num2str(j)];
                        varName_z_dp = [self.name,'_z_dp',num2str(i),'_',num2str(j)];
                        varName_m_maxDP = [self.name,'_m_maxDP',num2str(i),'_',num2str(j)];
                        
                        var_z_F = self.vars.(matlab.lang.makeValidName(varName_z_F));
                        var_z_dp = self.vars.(matlab.lang.makeValidName(varName_z_dp));
                        var_m_maxDP = self.vars.(matlab.lang.makeValidName(varName_m_maxDP));

                        C = C + [(-self.bigM <= var_z_F <= self.bigM):strcat(varName_z_F,'_bounds')];
                        C = C + [(zeros(self.N,1) <= var_z_dp <= var_m_maxDP):strcat(varName_z_dp,'_bounds')]; % if user does not supply m_maxDP...
                        C = C + [(0 <= var_m_maxDP <= self.bigM):strcat(varName_m_maxDP,'_bounds')]; % ...default is bigM
                    end
                end
            end

        end
        
        function J = get_J(self)
            %add objective terms to minimise line currents
            J = self.J;
            for i = 1:length(self.arcs)
                varName_z_dp = [self.name,'_z_dp',self.arcs{i}];
                var_z_dp = self.vars.(matlab.lang.makeValidName(varName_z_dp));
                
                J = J + self.penalty*ones(1,self.N)*var_z_dp;
            end
        end
        
    end
    
end