classdef PowerBalanceModel < Component
    
    % PowerBalanceModel Static MLD system model to provide a power balance
    % of multiple connections
    
    properties (SetAccess = private, GetAccess = public)
        Nsrc      %number of power balance source ports
        Nsnk      %1990s boyband
    end
    
    
    methods
        
        function self = PowerBalanceModel(name,N,bigM,Nsnk,Nsrc)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.Nsnk = Nsnk;
            self.Nsrc = Nsrc;
            
            %initialise variables
            self.y = self.get_y;
            self.z = self.get_z;
            
            %initialise constraints
            self.C = self.get_C;
        end%constructor
        
        
        %% OBJECT PROPERTY GET METHODS
        function y = get_y(self)
            %initialises MLD system output variables and returns cell array
            y = self.y;
            
            %pre-allocate variable name lists
            varNames_y_snk = cell(self.Nsnk,1);
            varNames_y_src = cell(self.Nsrc,1);
            
            %intantiate variables and add to name list
            for i = 1:self.Nsnk
                varName_y_snk = [self.name,'_y_snk',num2str(i)];
                
                self.vars.(matlab.lang.makeValidName(varName_y_snk)) = sdpvar(self.N,1); %define sdpvar in vars struct
                
                varNames_y_snk{i} = varName_y_snk;
            end
            
            for i = 1:self.Nsrc
                varName_y_src = [self.name,'_y_src',num2str(i)];
                
                self.vars.(matlab.lang.makeValidName(varName_y_src)) = sdpvar(self.N,1); %define sdpvar in vars struct
                
                varNames_y_src{i} = varName_y_src;
            end
            
            y = [y;varNames_y_snk;varNames_y_src];
            
            self.snkPorts = varNames_y_snk;
            self.srcPorts = varNames_y_src;
        end
        
        function z = get_z(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            z = self.z;
            
            varName_z = [self.name,'_z'];
                
            self.vars.(matlab.lang.makeValidName(varName_z)) = sdpvar(self.N,1);
                        
            z = [z;varName_z];
        end
        
        function C = get_C(self)
            C = self.C;
            %construct constraints for PowerBalanceModel
            
            %set bounds for system output variables and create vector of
            %variables for summation
            tmp_snk = [];
            tmp_src = [];
            
            for i = 1:self.Nsnk
                varName_y_snk = [self.name,'_y_snk',num2str(i)];
                
                var_y_snk = self.vars.(matlab.lang.makeValidName(varName_y_snk)); %get variable
                
                tmp_snk = [tmp_snk var_y_snk];
                
                %bound constraints
                C = C + [(-self.bigM <= var_y_snk <= self.bigM):strcat(varName_y_snk,'_bounds')];
            end
            
            for i = 1:self.Nsrc
                varName_y_src = [self.name,'_y_src',num2str(i)];
                
                var_y_src = self.vars.(matlab.lang.makeValidName(varName_y_src)); %get variable
                
                tmp_src = [tmp_src var_y_src];
                
                %bound constraints
                C = C + [(-self.bigM <= var_y_src <= self.bigM):strcat(varName_y_src,'_bounds')];
            end
            
            %get auxiliary variable for summation
            varName_z = [self.name,'_z'];
                
            var_z = self.vars.(matlab.lang.makeValidName(varName_z));
            
            %overall power balance constraints
            C = C + [(var_z == tmp_snk*ones(self.Nsnk,1) - tmp_src*ones(self.Nsrc,1)):varName_z];   
            C = C + [(var_z == 0):strcat(self.name,'_power_balance')];
        end
        
    end
    
end