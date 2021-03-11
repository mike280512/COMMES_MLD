classdef ElecNetworkModel < Component
    
    % ElecNetworkModel  A quadratically constrained network model
    % to connect hubs within a district. The model requires an adjacency 
    % matrix to be instantiated.  The adjacency matrix must be square. 
    
    properties (SetAccess = private, GetAccess = public)
        adjMatrix               %adjacency matrix (matrix)
        arcs                    %connecting arcs (cell array)
        v1                      %reference voltage (p.u.) (scalar)
        PF                      %power factor (vector)
        penalty                 %penalty to ensure exactness (scalar)
    end
    
    
    methods
        
        function self = ElecNetworkModel(name,N,bigM,adjMatrix,v1,PF,penalty)
            
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
            self.v1 = v1;
            if isempty(PF)
                self.PF = 1; %default value for power factor
            else
                self.PF = PF;
            end
                        
            %initialise variables
            self.y = self.get_y;
            self.z = self.get_z;
            self.m = self.get_m;
            
            %initialise constraints
            self.C = self.get_C;
            
            %add terms to objective function to ensure exact relaxation
            if isempty(penalty)
                self.penalty = []; %penalty term not added to objective
            else
                self.penalty = penalty;
                self.J = self.get_J;
            end
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
            
            varNames_y_p = cell(size(self.adjMatrix,1),1);
            varNames_y_q = cell(size(self.adjMatrix,1),1);
            varNames_y_v = cell(size(self.adjMatrix,1),1);
            
            for i = 1:length(varNames_y_p)
                varName_y_p = [self.name,'_y_p',num2str(i)]; %define variable name
                varName_y_q = [self.name,'_y_q',num2str(i)];
                varName_y_v = [self.name,'_y_v',num2str(i)];
                
                self.vars.(matlab.lang.makeValidName(varName_y_p)) = sdpvar(self.N,1); %define sdpvar in vars struct
                self.vars.(matlab.lang.makeValidName(varName_y_q)) = sdpvar(self.N,1);
                self.vars.(matlab.lang.makeValidName(varName_y_v)) = sdpvar(self.N,1);
                
                varNames_y_p{i} = varName_y_p;
                varNames_y_q{i} = varName_y_q;
                varNames_y_v{i} = varName_y_v;
            end
            
            y = [y;varNames_y_p;varNames_y_q;varNames_y_v];
            
            self.srcPorts = varNames_y_p;
        end
        
        function z = get_z(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            z = self.z;
            
            varNames_z_P = cell(length(self.arcs),1);
            varNames_z_Q = cell(length(self.arcs),1);
            varNames_z_I = cell(length(self.arcs),1);
            
            for i = 1:length(self.arcs)
                varName_z_P = [self.name,'_z_P',self.arcs{i}]; %define variable name
                varName_z_Q = [self.name,'_z_Q',self.arcs{i}];
                varName_z_I = [self.name,'_z_I',self.arcs{i}];             
                
                self.vars.(matlab.lang.makeValidName(varName_z_P)) = sdpvar(self.N,1); %define sdpvar in vars struct
                self.vars.(matlab.lang.makeValidName(varName_z_Q)) = sdpvar(self.N,1);
                self.vars.(matlab.lang.makeValidName(varName_z_I)) = sdpvar(self.N,1);
                
                varNames_z_P{i} = varName_z_P;
                varNames_z_Q{i} = varName_z_Q;
                varNames_z_I{i} = varName_z_I;
            end
            
            z = [z;varNames_z_P;varNames_z_Q;varNames_z_I];
        end
        
        function m = get_m(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            m = self.m;
            
            varNames_m_maxI = cell(length(self.arcs),1);
            
            for i = 1:length(self.arcs)
                varName_m_maxI = [self.name,'_m_maxI',self.arcs{i}]; %define variable name            
                
                self.vars.(matlab.lang.makeValidName(varName_m_maxI)) = sdpvar(self.N,1); %define sdpvar in vars struct
                
                varNames_m_maxI{i} = varName_m_maxI;
            end
            
            m = [m;varNames_m_maxI];
        end
        
        function C = get_C(self)
            %construct constraints for network model
            C = self.C; %constraints are added to existing lmi
            [r,c] = size(self.adjMatrix);
            
            %active power nodal balances
            for i = 1:r    %iterate through nodes
                varName_y_p = [self.name,'_y_p',num2str(i)]; %get name of node variable
                var_y_p = self.vars.(matlab.lang.makeValidName(varName_y_p)); %get node variable
                zVars = [];
                coeffs = [];
                for j = 1:c %iterate through each column of adjMatrix matrix
                    if self.adjMatrix(i,j)>0   %if element at index = (node,current column) is greater than 0
                        varName_z_P = [self.name,'_z_P',num2str(i),'_',num2str(j)]; %get name of adjacent arc variable
                        
                        var_z_P = self.vars.(matlab.lang.makeValidName(varName_z_P));
                        
                        zVars = [zVars, var_z_P]; %get adjacent arc variable
                        coeffs = [coeffs; 1]; %assign coefficient
                    end
                    if self.adjMatrix(j,i)>0
                        varName_z_P = [self.name,'_z_P',num2str(j),'_',num2str(i)];
                        varName_z_I = [self.name,'_z_I',num2str(j),'_',num2str(i)];
                        
                        var_z_P = self.vars.(matlab.lang.makeValidName(varName_z_P));
                        var_z_I = self.vars.(matlab.lang.makeValidName(varName_z_I));
                        
                        zVars = [zVars, var_z_P, var_z_I]; %get adjacent arc variable
                        coeffs = [coeffs; -1; real(self.adjMatrix(j,i))]; %assign arc conversion factor as coefficient
                    end
                end
                C = C + [(var_y_p == zVars*coeffs):strcat(varName_y_p,'_balance')]; %tagging is used to identify constraints
            end
            
            %add power factor relationship for non-reference nodes
            for i = 2:r
                varName_y_q = [self.name,'_y_q',num2str(i)]; %get name of node variable
                varName_y_p = [self.name,'_y_p',num2str(i)];
                
                var_y_q = self.vars.(matlab.lang.makeValidName(varName_y_q)); %get node variable
                var_y_p = self.vars.(matlab.lang.makeValidName(varName_y_p));
                
                pfConv = tan(acos(self.PF(i)));
                C = C + [(var_y_q == var_y_p*pfConv):strcat(varName_y_q)]; %tagging is used to identify constraints
                
            end
            %add reactive power nodal balances
            for i = 1:r    %iterate through nodes
                varName_y_q = [self.name,'_y_q',num2str(i)]; %get name of node variable
                
                var_y_q = self.vars.(matlab.lang.makeValidName(varName_y_q)); %get node variable
                
                zVars = [];
                coeffs = [];
                for j = 1:c %iterate through each column of adjMatrix matrix
                    if self.adjMatrix(i,j)>0   %if element at index = (node,current column) is greater than 0
                        varName_z_Q = [self.name,'_z_Q',num2str(i),'_',num2str(j)]; %get name of adjacent arc variable
                        var_z_Q = self.vars.(matlab.lang.makeValidName(varName_z_Q));
                        zVars = [zVars, var_z_Q]; %get adjacent arc variable
                        coeffs = [coeffs; 1]; %assign coefficient
                    end
                    if self.adjMatrix(j,i)>0
                        varName_z_Q = [self.name,'_z_Q',num2str(j),'_',num2str(i)];
                        varName_z_I = [self.name,'_z_I',num2str(j),'_',num2str(i)];
                        
                        var_z_Q = self.vars.(matlab.lang.makeValidName(varName_z_Q));
                        var_z_I = self.vars.(matlab.lang.makeValidName(varName_z_I));
                        
                        zVars = [zVars, var_z_Q, var_z_I]; %get adjacent arc variable
                        coeffs = [coeffs; -1; imag(self.adjMatrix(j,i))]; %assign arc conversion factor as coefficient
                    end
                end

                C = C + [(var_y_q == zVars*coeffs):strcat(varName_y_q,'_balance')]; %tagging is used to identify constraints
            end
            
            %add reference voltage magnitude squared at node 1
            varName_y_v1 = [self.name,'_y_v1'];
            var_y_v1 = self.vars.(matlab.lang.makeValidName(varName_y_v1));
            C = C + [(var_y_v1 == self.v1):strcat(varName_y_v1,'_reference')];
                
            %add voltage magnitude squared nodal balances
            for j = 2:c %don't add for reference node
                varName_y_v = [self.name,'_y_v',num2str(j)]; %get name of node variable
                var_y_v = self.vars.(matlab.lang.makeValidName(varName_y_v)); %get node variable
                zVars = [];
                coeffs = [];
                for i = 1:r
                    if self.adjMatrix(i,j)>0   %if element at index = (current row, node) is greater than 0
                        varName_y_vi = [self.name,'_y_v',num2str(i)];
                        varName_z_P = [self.name,'_z_P',num2str(i),'_',num2str(j)];
                        varName_z_Q = [self.name,'_z_Q',num2str(i),'_',num2str(j)];
                        varName_z_I = [self.name,'_z_I',num2str(i),'_',num2str(j)];
                        
                        var_y_vi = self.vars.(matlab.lang.makeValidName(varName_y_vi));
                        var_z_P = self.vars.(matlab.lang.makeValidName(varName_z_P));
                        var_z_Q = self.vars.(matlab.lang.makeValidName(varName_z_Q));
                        var_z_I = self.vars.(matlab.lang.makeValidName(varName_z_I));
                        
                        zVars = [zVars, var_y_vi, var_z_P, var_z_Q, var_z_I];
                        coeffs = [coeffs; 1; -2*real(self.adjMatrix(i,j)); -2*imag(self.adjMatrix(i,j));...
                            (real(self.adjMatrix(i,j))^2+imag(self.adjMatrix(i,j))^2)];
                    end
                end
                C = C + [(var_y_v == zVars*coeffs):strcat(varName_y_v,'_balance')]; %every node has at most one ancestor
            end
            
            %add SOCP constraints
            for i = 1:r
                for j = 1:c
                    if self.adjMatrix(i,j) > 0
                        varName_y_v = [self.name,'_y_v',num2str(i)];
                        varName_z_P = [self.name,'_z_P',num2str(i),'_',num2str(j)];
                        varName_z_Q = [self.name,'_z_Q',num2str(i),'_',num2str(j)];
                        varName_z_I = [self.name,'_z_I',num2str(i),'_',num2str(j)];
                        
                        var_y_v = self.vars.(matlab.lang.makeValidName(varName_y_v));
                        var_z_P = self.vars.(matlab.lang.makeValidName(varName_z_P));
                        var_z_Q = self.vars.(matlab.lang.makeValidName(varName_z_Q));
                        var_z_I = self.vars.(matlab.lang.makeValidName(varName_z_I));
                        
                        C = C + [(var_z_P.^2 + var_z_Q.^2 <= var_z_I.*var_y_v):strcat(varName_z_I,'_SOCP')]; %suitable for gurobi
                        %other solvers will require the yalmip norm
                        %operator, implemented as follows for each sampling
                        %interval:
                        %for k = 1:self.N
%                         C = C + [(norm([2*var_z_P(k); 2*var_z_Q(k); (var_z_I(k) - var_y_v(k))],2) <= (var_z_I(k) + var_y_v(k))):strcat(varName_z_I,'_SOCP')];
                        %end
                    end
                end
            end
            
            %add default bounds
            for i = 1:r
                varName_y_p = [self.name,'_y_p',num2str(i)];
                varName_y_q = [self.name,'_y_q',num2str(i)];
                varName_y_v = [self.name,'_y_v',num2str(i)];
                
                var_y_p = self.vars.(matlab.lang.makeValidName(varName_y_p));
                var_y_q = self.vars.(matlab.lang.makeValidName(varName_y_q));
                var_y_v = self.vars.(matlab.lang.makeValidName(varName_y_v));
                
                C = C + [(-self.bigM <= var_y_p <= self.bigM):strcat(varName_y_p,'_bounds')];
                C = C + [(-self.bigM <= var_y_q <= self.bigM):strcat(varName_y_q,'_bounds')];
                C = C + [(0 <= var_y_v <= self.bigM):strcat(varName_y_v,'_bounds')];
            end
            for i = 1:r
                for j = 1:c
                    if self.adjMatrix(i,j) > 0
                        varName_z_P = [self.name,'_z_P',num2str(i),'_',num2str(j)];
                        varName_z_Q = [self.name,'_z_Q',num2str(i),'_',num2str(j)];
                        varName_z_I = [self.name,'_z_I',num2str(i),'_',num2str(j)];
                        varName_m_maxI = [self.name,'_m_maxI',num2str(i),'_',num2str(j)];
                        
                        var_z_P = self.vars.(matlab.lang.makeValidName(varName_z_P));
                        var_z_Q = self.vars.(matlab.lang.makeValidName(varName_z_Q));
                        var_z_I = self.vars.(matlab.lang.makeValidName(varName_z_I));
                        var_m_maxI = self.vars.(matlab.lang.makeValidName(varName_m_maxI));
                        
                        C = C + [(-self.bigM <= var_z_P <= self.bigM):strcat(varName_z_P,'_bounds')];
                        C = C + [(-self.bigM <= var_z_Q <= self.bigM):strcat(varName_z_Q,'_bounds')];
                        C = C + [(zeros(self.N,1) <= var_z_I <= var_m_maxI):strcat(varName_z_I,'_bounds')]; %if user does not supply m_maxI...
                        C = C + [(0 <= var_m_maxI <= self.bigM):strcat(varName_m_maxI,'_bounds')]; % ...default is bigM
                    end
                end
            end

        end
        
        function J = get_J(self)
            %add objective terms to minimise line currents
            J = self.J;
            for i = 1:length(self.arcs)
                varName_z_I = [self.name,'_z_I',self.arcs{i}];
                var_z_I = self.vars.(matlab.lang.makeValidName(varName_z_I));
                
                J = J + self.penalty*ones(1,self.N)*var_z_I;
            end
        end
        
    end
    
end