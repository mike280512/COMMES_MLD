classdef NetworkBalanceModel < Component
    
    % NetworkBalanceModel  A linear, non-realistic simple network model
    % to connect hubs within a district. The model requires an adjacency 
    % matrix to be instantiated.  The adjacency matrix must be square. 
    
    properties (SetAccess = private, GetAccess = public)
        adjMatrix               %adjacency matrix
        arcs                    %connecting arcs
    end
    
    
    methods
        
        function self = NetworkBalanceModel(name,N,bigM,adjMatrix)
            
            %call parent class constructor
            self@Component(name,N)
            
            [r,c] = size(adjMatrix);
            
            %validate input params
            assert(r == c,'Node connection matrix must be square')
            assert(~any(diag(adjMatrix)),'Diagonal of adjMatrix matrix must be zero.  Nodes cannot be connected to themselves.')
            
            %initialise and set object properties
            self.adjMatrix = adjMatrix;
            self.bigM = bigM;
            self.arcs = self.get_Arcs;
            
            %initialise variables
            self.y = self.get_y;
            self.z = self.get_z;
            
            %initialise constraints
            self.C = self.get_C;
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
            
            varNames_y = cell(size(self.adjMatrix,1),1);

            for i = 1:length(varNames_y)
                varName_y = [self.name,'_y_',num2str(i)]; %define variable name
                self.vars.(matlab.lang.makeValidName(varName_y)) = sdpvar(self.N,1); %define sdpvar in vars struct
                varNames_y{i} = varName_y;
            end
            
            y = [y;varNames_y];
            
            self.srcPorts = varNames_y;
        end
        
        function z = get_z(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            z = self.z;
            
            varNames_z = cell(length(self.arcs),1);
            
            for i = 1:length(self.arcs)
                varName_z = [self.name,'_z_',self.arcs{i}]; %define variable name
                self.vars.(matlab.lang.makeValidName(varName_z)) = sdpvar(self.N,1); %define sdpvar in vars struct
                varNames_z{i} = varName_z;
            end
            
            z = [z;varNames_z];
        end
        
        function C = get_C(self)
            %construct constraints for network model
            C = self.C; %constraints are added to existing lmi
            
            for i = 1:size(self.adjMatrix,1)    %iterate through nodes
                varName_y = [self.name,'_y_',num2str(i)]; %get name of node variable
                var_y = self.vars.(matlab.lang.makeValidName(varName_y)); %get node variable
                zVars = [];
                coeffs = [];
                for j = 1:size(self.adjMatrix,1) %iterate through each column of adjMatrix matrix
                    if self.adjMatrix(i,j)>0   %if element at index = (node,current column) is greater than 0
                        varName_z = [self.name,'_z_',num2str(i),'_',num2str(j)]; %get name of adjacent arc variable
                        zVars = [zVars self.vars.(matlab.lang.makeValidName(varName_z))]; %get adjacent arc variable
                        coeffs = [coeffs; 1]; %assign positive coefficient
                    end
                    if self.adjMatrix(j,i)>0
                        varName_z = [self.name,'_z_',num2str(j),'_',num2str(i)];
                        zVars = [zVars self.vars.(matlab.lang.makeValidName(varName_z))]; %get adjacent arc variable
                        coeffs = [coeffs; -self.adjMatrix(j,i)]; %assign arc conversion factor as coefficient
                    end
                end
                C = C + [(var_y == zVars*coeffs):strcat(varName_y,'_balance')]; %tagging is used to identify constraints
            end
            
            for i = 1:length(self.y)
                varName_y = self.y{i};
                
                var_y = self.vars.(matlab.lang.makeValidName(varName_y));
                
                %bound constraints
                C = C + [(-self.bigM <= var_y <= self.bigM):strcat(varName_y,'_bounds')];
            end
            
            for i = 1:length(self.z)
                varName_z = self.z{i};
                
                var_z = self.vars.(matlab.lang.makeValidName(varName_z));
                
                %bound constraints
                C = C + [(-self.bigM <= var_z <= self.bigM):strcat(varName_z,'_bounds')];
            end
        end
        
    end
    
end