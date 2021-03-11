classdef HeatNetworkModel < Component
    
    % HeatNetworkModel  A quadratically constrained network model
    % to connect hubs within a district. The model requires an adjacency 
    % matrix to be instantiated.  The adjacency matrix must be square. 
    
    properties (SetAccess = private, GetAccess = public)
        adjMatrix               %adjacency matrix (matrix)
        hexCs                   %nodal head loss factors, dimension equal to adjMatrix (vector)
        arcs                    %connecting arcs (cell array)
        penalty                 %penalty to ensure exactness (scalar)
    end
    
    
    methods
        
        function self = HeatNetworkModel(name,N,bigM,adjMatrix,hexCs,penalty)
            
            %call parent class constructor
            self@Component(name,N)
            
            [r,c] = size(adjMatrix);
            
            %validate input params
            assert(r == c,'Node connection matrix must be square')
            assert(~any(diag(adjMatrix)),'Diagonal of adjMatrix matrix must be zero.  Nodes cannot be connected to themselves.')
            
            %initialise and set object properties
            self.adjMatrix = adjMatrix;
            self.bigM = bigM;
            self.hexCs = hexCs;
            self.arcs = self.get_arcs;
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
            
            varNames_y_phi = cell(size(self.adjMatrix,1),1);
            varNames_y_pumpDH = cell(size(self.adjMatrix,1),1);

            for i = 1:length(varNames_y_phi)
                varName_y_phi = [self.name,'_y_phi',num2str(i)]; %define variable name
                varName_y_pumpDH = [self.name,'_y_pumpDH',num2str(i)];
                
                self.vars.(matlab.lang.makeValidName(varName_y_phi)) = sdpvar(self.N,1); %define sdpvar in vars struct
                self.vars.(matlab.lang.makeValidName(varName_y_pumpDH)) = sdpvar(self.N,1);
                
                varNames_y_phi{i} = varName_y_phi;
                varNames_y_pumpDH{i} = varName_y_pumpDH;
            end
            
            y = [y;varNames_y_phi;varNames_y_pumpDH];
            
            self.srcPorts = varNames_y_phi;
        end
        
        function z = get_z(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            z = self.z;
            
            varNames_z_Phi = cell(length(self.arcs),1);
            varNames_z_pipeDH = cell(length(self.arcs),1);
            varNames_z_hexDH = cell(size(self.adjMatrix,1),1);
            varNames_z_wH = cell(size(self.adjMatrix,1),1);
            varNames_z_cH = cell(size(self.adjMatrix,1),1);
            
            for i = 1:length(self.arcs)
                varName_z_Phi = [self.name,'_z_Phi',self.arcs{i}]; %define variable name
                varName_z_pipeDH = [self.name,'_z_pipeDH',self.arcs{i}];
                
                self.vars.(matlab.lang.makeValidName(varName_z_Phi)) = sdpvar(self.N,1); %define sdpvar in vars struct
                self.vars.(matlab.lang.makeValidName(varName_z_pipeDH)) = sdpvar(self.N,1);
                
                varNames_z_Phi{i} = varName_z_Phi;
                varNames_z_pipeDH{i} = varName_z_pipeDH;
            end
            
            for i = 1:size(self.adjMatrix,1)
                varName_z_hexDH = [self.name,'_z_hexDH',num2str(i)]; %define variable name
                varName_z_wH = [self.name,'_z_wH',num2str(i)];
                varName_z_cH = [self.name,'_z_cH',num2str(i)];
                
                self.vars.(matlab.lang.makeValidName(varName_z_hexDH)) = sdpvar(self.N,1); %define sdpvar in vars struct
                self.vars.(matlab.lang.makeValidName(varName_z_wH)) = sdpvar(self.N,1);
                self.vars.(matlab.lang.makeValidName(varName_z_cH)) = sdpvar(self.N,1);
                
                varNames_z_hexDH{i} = varName_z_hexDH;
                varNames_z_wH{i} = varName_z_wH;
                varNames_z_cH{i} = varName_z_cH;
            end
            
            z = [z;varNames_z_Phi;varNames_z_pipeDH;varNames_z_hexDH;varNames_z_wH;varNames_z_cH];
        end
        
        function d = get_d(self)
            %initialies MLD system continuous binary variables and returns cell array
            d = self.d;
            
            varNames_d_pipe = cell(length(self.arcs),1);
            varNames_d_pump = cell(size(self.adjMatrix,1),1);
            
            for i = 1:length(self.arcs)
                varName_d_pipe = [self.name,'_d_pipe',self.arcs{i}]; %define variable name
                
                self.vars.(matlab.lang.makeValidName(varName_d_pipe)) = binvar(self.N,1); %define sdpvar in vars struct
                
                varNames_d_pipe{i} = varName_d_pipe;
            end
            
            for i = 1:size(self.adjMatrix,1)
                varName_d_pump = [self.name,'_d_pump',num2str(i)]; %define variable name
                
                self.vars.(matlab.lang.makeValidName(varName_d_pump)) = binvar(self.N,1); %define sdpvar in vars struct
                
                varNames_d_pump{i} = varName_d_pump;
            end
            
            d = [d;varNames_d_pipe;varNames_d_pump];
        end
        
        function m = get_m(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            m = self.m;
            
            varNames_m_maxHexDH = cell(size(self.adjMatrix,1),1);
            varNames_m_maxPipeDH = cell(length(self.arcs),1);
            
            for i = 1:length(self.arcs)
                varName_m_maxPipeDH = [self.name,'_m_maxPipeDH',self.arcs{i}];         
                
                self.vars.(matlab.lang.makeValidName(varName_m_maxPipeDH)) = sdpvar(self.N,1);
                
                varNames_m_maxPipeDH{i} = varName_m_maxPipeDH;
            end
            
            for i = 1:size(self.adjMatrix,1)
                varName_m_maxHexDH = [self.name,'_m_maxHexDH',num2str(i)]; %define variable name
                
                self.vars.(matlab.lang.makeValidName(varName_m_maxHexDH)) = sdpvar(self.N,1); %define sdpvar in vars struct
                
                varNames_m_maxHexDH{i} = varName_m_maxHexDH;
            end
            
            m = [m;varNames_m_maxPipeDH;varNames_m_maxHexDH];
        end
        
        function C = get_C(self)
            %construct constraints for network model
            C = self.C; %constraints are added to existing lmi
            [r,c] = size(self.adjMatrix);
            
            %flow nodal balances
            for i = 1:r    %iterate through nodes
                varName_y_phi = [self.name,'_y_phi',num2str(i)]; %get name of node variable
                var_y_phi = self.vars.(matlab.lang.makeValidName(varName_y_phi)); %get node variable
                zVars = [];
                coeffs = [];
                for j = 1:c %iterate through each column of adjMatrix matrix
                    if self.adjMatrix(i,j)>0   %if element at index = (node,current column) is greater than 0
                        varName_z_Phi = [self.name,'_z_Phi',num2str(i),'_',num2str(j)]; %get name of adjacent arc variable
                        
                        var_z_Phi = self.vars.(matlab.lang.makeValidName(varName_z_Phi));
                        
                        zVars = [zVars, var_z_Phi]; %get adjacent arc variable
                        coeffs = [coeffs; 1]; %assign coefficient
                    end
                    if self.adjMatrix(j,i)>0
                        varName_z_Phi = [self.name,'_z_Phi',num2str(j),'_',num2str(i)];
                        
                        var_z_Phi = self.vars.(matlab.lang.makeValidName(varName_z_Phi));
                        
                        zVars = [zVars, var_z_Phi]; %get adjacent arc variable
                        coeffs = [coeffs; -1]; %assign arc conversion factor as coefficient
                    end
                end
                C = C + [(var_y_phi == zVars*coeffs):strcat(varName_y_phi,'_balance')]; %tagging is used to identify constraints
            end
            
%             %add reference pressure at node 1
%             varName_y_pr1 = [self.name,'_y_pr1'];
%             var_y_pr1 = self.vars.(matlab.lang.makeValidName(varName_y_pr1));
%             C = C + [(var_y_pr1 == self.pr1):strcat(varName_y_pr1,'_reference')];
                
            %add pressure drop relaxation constraints
            for i = 1:r
                for j = 1:c
                    if self.adjMatrix(i,j) > 0
                        pipeC = self.adjMatrix(i,j);
                        
                        varName_z_wHi = [self.name,'_z_wH',num2str(i)];
                        varName_z_wHj = [self.name,'_z_wH',num2str(j)];
                        varName_z_cHi = [self.name,'_z_cH',num2str(i)];
                        varName_z_cHj = [self.name,'_z_cH',num2str(j)];
                        varName_z_Phi = [self.name,'_z_Phi',num2str(i),'_',num2str(j)];
                        varName_z_pipeDH = [self.name,'_z_pipeDH',num2str(i),'_',num2str(j)];
                        varName_d_pipe = [self.name,'_d_pipe',num2str(i),'_',num2str(j)];
                        
                        var_z_wHi = self.vars.(matlab.lang.makeValidName(varName_z_wHi));
                        var_z_wHj = self.vars.(matlab.lang.makeValidName(varName_z_wHj));
                        var_z_cHi = self.vars.(matlab.lang.makeValidName(varName_z_cHi));
                        var_z_cHj = self.vars.(matlab.lang.makeValidName(varName_z_cHj));
                        var_z_Phi = self.vars.(matlab.lang.makeValidName(varName_z_Phi));
                        var_z_pipeDH = self.vars.(matlab.lang.makeValidName(varName_z_pipeDH));
                        var_d_pipe = self.vars.(matlab.lang.makeValidName(varName_d_pipe));
                        
                        %bilinear relaxation constraints
                        C = C + [(-self.bigM*(1 - var_d_pipe) <= var_z_Phi <= self.bigM*var_d_pipe):varName_d_pipe];
                        C = C + [(var_z_wHj - var_z_wHi - 2*self.bigM*var_d_pipe <= var_z_pipeDH):strcat(varName_z_pipeDH,'_relaxation_l_rev')];
                        C = C + [(var_z_wHi - var_z_wHj - 2*self.bigM*(1 - var_d_pipe) <= var_z_pipeDH):strcat(varName_z_pipeDH,'_relaxation_l_fwd')];
                        C = C + [(var_z_pipeDH <= var_z_wHj - var_z_wHi + 2*self.bigM*var_d_pipe):strcat(varName_z_pipeDH,'_relaxation_u_rev')];
                        C = C + [(var_z_pipeDH <= var_z_wHi - var_z_wHj + 2*self.bigM*(1 - var_d_pipe)):strcat(varName_z_pipeDH,'_relaxation_u_fwd')];
                        %second order cone constraint for pipe head loss
                        C = C + [(pipeC*var_z_Phi.^2 <= var_z_pipeDH):strcat(varName_z_pipeDH,'_SOCP')]; %suitable for gurobi
                        %other solvers may require the yalmip norm
                        %operator, implemented as follows for each sampling
                        %interval:
                        %for k = 1:self.N
%                         C = C + [(norm([2*sqrt(pipeC)*var_z_Phi(k); var_z_pipeDH(k) - 1],2) <= var_z_pipeDH(k) + 1):strcat(varName_z_pipeDH,'_SOCP')];
                        %end
                        %warm/cold side symmetry constraints
                        C = C + [(var_z_wHi - var_z_wHj == -(var_z_cHi - var_z_cHj)):strcat('pipe_',num2str(i),'_',num2str(j),'_symmetry')];
                    end
                end
            end
            
            for i = 1:r
                hexC = self.hexCs(i);
                
                varName_y_phi = [self.name,'_y_phi',num2str(i)];
                varName_z_hexDH = [self.name,'_z_hexDH',num2str(i)];
                varName_y_pumpDH = [self.name,'_y_pumpDH',num2str(i)];
                varName_z_wH = [self.name,'_z_wH',num2str(i)];
                varName_z_cH = [self.name,'_z_cH',num2str(i)];
                varName_d_pump = [self.name,'_d_pump',num2str(i)];
                
                var_y_phi = self.vars.(matlab.lang.makeValidName(varName_y_phi));
                var_z_hexDH = self.vars.(matlab.lang.makeValidName(varName_z_hexDH));
                var_y_pumpDH = self.vars.(matlab.lang.makeValidName(varName_y_pumpDH));
                var_z_wH = self.vars.(matlab.lang.makeValidName(varName_z_wH));
                var_z_cH = self.vars.(matlab.lang.makeValidName(varName_z_cH));
                var_d_pump = self.vars.(matlab.lang.makeValidName(varName_d_pump));
                
                %second order cone constraint for heat exchange head loss
                C = C + [(hexC*var_y_phi.^2 <= var_z_hexDH):strcat(varName_z_hexDH,'_SOCP')]; %suitable for gurobi
                %other solvers may require the yalmip norm
                %operator, implemented as follows for each sampling
                %interval:
                %for k = 1:self.N
%                 C = C + [(norm([2*sqrt(hexC)*var_y_phi(k); var_z_hexDH(k) - 1],2) <= var_z_hexDH(k) + 1):strcat(varName_z_hexDH,'_SOCP')];
                %end
                
                %minimum pump head gain constraints
                %bilinear relaxation constraints
                C = C + [(-self.bigM*(1 - var_d_pump) <= var_y_phi <= self.bigM*var_d_pump):varName_d_pump];
                C = C + [(var_z_cH - var_z_wH + var_z_hexDH - 2*self.bigM*var_d_pump <= var_y_pumpDH):strcat(varName_z_pipeDH,'_relaxation_l_rev')];
                C = C + [(var_z_wH - var_z_cH + var_z_hexDH - 2*self.bigM*(1 - var_d_pump) <= var_y_pumpDH):strcat(varName_z_pipeDH,'_relaxation_l_fwd')];
                C = C + [(var_y_pumpDH <= var_z_cH - var_z_wH + var_z_hexDH + 2*self.bigM*var_d_pump):strcat(varName_z_pipeDH,'_relaxation_u_rev')];
                C = C + [(var_y_pumpDH <= var_z_wH - var_z_cH + var_z_hexDH + 2*self.bigM*(1 - var_d_pump)):strcat(varName_z_pipeDH,'_relaxation_u_fwd')];
            end
            
            %add default bounds
            for i = 1:r
                varName_y_phi = [self.name,'_y_phi',num2str(i)];
                varName_y_pumpDH = [self.name,'_y_pumpDH',num2str(i)];
                varName_z_hexDH = [self.name,'_z_hexDH',num2str(i)];
                varName_z_wH = [self.name,'_z_wH',num2str(i)];
                varName_z_cH = [self.name,'_z_cH',num2str(i)];
                varName_m_maxHexDH = [self.name,'_m_maxHexDH',num2str(i)];
                
                var_y_phi = self.vars.(matlab.lang.makeValidName(varName_y_phi));
                var_y_pumpDH = self.vars.(matlab.lang.makeValidName(varName_y_pumpDH));
                var_z_hexDH = self.vars.(matlab.lang.makeValidName(varName_z_hexDH));
                var_z_wH = self.vars.(matlab.lang.makeValidName(varName_z_wH));
                var_z_cH = self.vars.(matlab.lang.makeValidName(varName_z_cH));
                var_m_maxHexDH = self.vars.(matlab.lang.makeValidName(varName_m_maxHexDH));
                
                C = C + [(-self.bigM <= var_y_phi <= self.bigM):strcat(varName_y_phi,'_bounds')];
                C = C + [(0 <= var_y_pumpDH <= self.bigM):strcat(varName_y_pumpDH,'_bounds')];
                C = C + [(zeros(self.N,1) <= var_z_hexDH <= var_m_maxHexDH):strcat(varName_z_hexDH,'_bounds')]; %if user does not supply m_maxHexDH...
                C = C + [(0 <= var_m_maxHexDH <= self.bigM):strcat(varName_m_maxHexDH,'_bounds')]; % ...default is bigM
                C = C + [(0 <= var_z_wH <= self.bigM):strcat(varName_z_wH,'_bounds')];
                C = C + [(0 <= var_z_cH <= self.bigM):strcat(varName_z_cH,'_bounds')];
            end
            for i = 1:r
                for j = 1:c
                    if self.adjMatrix(i,j) > 0
                        varName_z_Phi = [self.name,'_z_Phi',num2str(i),'_',num2str(j)];
                        varName_z_pipeDH = [self.name,'_z_pipeDH',num2str(i),'_',num2str(j)];
                        varName_m_maxPipeDH = [self.name,'_m_maxPipeDH',num2str(i),'_',num2str(j)];
                        
                        var_z_Phi = self.vars.(matlab.lang.makeValidName(varName_z_Phi));
                        var_z_pipeDH = self.vars.(matlab.lang.makeValidName(varName_z_pipeDH));
                        var_m_maxPipeDH = self.vars.(matlab.lang.makeValidName(varName_m_maxPipeDH));
                        
                        C = C + [(-self.bigM <= var_z_Phi <= self.bigM):strcat(varName_z_Phi,'_bounds')];
                        C = C + [(zeros(self.N,1) <= var_z_pipeDH <= var_m_maxPipeDH):strcat(varName_z_pipeDH,'_bounds')]; %if user does not supply m_maxPipeDH...
                        C = C + [(0 <= var_m_maxPipeDH <= self.bigM):strcat(varName_m_maxPipeDH,'_bounds')]; % ...default is bigM
                    end
                end
            end

        end
        
        function J = get_J(self)
            %add objective terms to minimise pipe head loss
            J = self.J;
            for i = 1:length(self.arcs)
                varName_z_pipeDH = [self.name,'_z_pipeDH',self.arcs{i}];
                var_z_pipeDH = self.vars.(matlab.lang.makeValidName(varName_z_pipeDH));
                
                J = J + self.penalty*ones(1,self.N)*var_z_pipeDH;
            end
            %add objective terms to minimise pump head loss
            for i = 1:size(self.adjMatrix,1)
                varName_y_pumpDH = [self.name,'_y_pumpDH',num2str(i)];
                var_y_pumpDH = self.vars.(matlab.lang.makeValidName(varName_y_pumpDH));
                
                J = J + self.penalty*ones(1,self.N)*var_y_pumpDH;
            end
        end
        
    end
    
end