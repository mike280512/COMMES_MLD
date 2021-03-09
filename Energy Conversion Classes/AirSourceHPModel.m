classdef AirSourceHPModel < Component
    
    % AirSourceHPModel Static MLD system model of an irreversible
    % air-source heat pump
    
    properties (SetAccess = private, GetAccess = public)
        COP   % Coefficient of Performance of heat pump
    end
    
    
    methods
        
        function self = AirSourceHPModel(name,N,bigM,COP)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.COP = COP;
            
            %initialise variables
            self.y = self.get_y;
            self.uc = self.get_uc;
            
            %initialise constraints
            self.C = self.get_C;
        end%constructor
        
        
        %% OBJECT PROPERTY GET METHODS
        function y = get_y(self)
            %initialies MLD system output variables and returns cell array
            y = self.y;
            
            varName_y_e = [self.name,'_y_e'];
            varName_y_h = [self.name,'_y_h'];
            
            self.vars.(matlab.lang.makeValidName(varName_y_e)) = sdpvar(self.N,1); %define sdpvar in vars struct
            self.vars.(matlab.lang.makeValidName(varName_y_h)) = sdpvar(self.N,1);
            
            y = [y;varName_y_e;varName_y_h];
            
            self.srcPorts = {varName_y_h};
            self.snkPorts = {varName_y_e};
        end
        
        function uc = get_uc(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            uc = self.uc;
            
            varName_uc = [self.name,'_uc'];
            
            self.vars.(matlab.lang.makeValidName(varName_uc)) = sdpvar(self.N,1);
            
            uc = [uc;varName_uc];
        end
        
        function C = get_C(self)
            C = self.C;
            %construct constraints for AirSourceHPModel
                        
            varName_y_e = [self.name,'_y_e']; %sink port
            varName_y_h = [self.name,'_y_h']; %source port
            varName_uc = [self.name,'_uc'];
            
            var_y_e = self.vars.(matlab.lang.makeValidName(varName_y_e)); %get variable
            var_y_h = self.vars.(matlab.lang.makeValidName(varName_y_h));
            var_uc = self.vars.(matlab.lang.makeValidName(varName_uc));
            
            %MLD constraints
            C = C + [(var_y_e == var_uc):varName_y_e]; %tagging is used to identify constraints
            C = C + [(var_y_h == (self.COP)*var_uc):varName_y_h];
            
            %bound constraints
            C = C + [(0 <= var_y_e <= self.bigM):strcat(varName_y_e,'_bounds')];
            C = C + [(0 <= var_y_h <= self.bigM):strcat(varName_y_h,'_bounds')];
            C = C + [(0 <= var_uc <= self.bigM):strcat(varName_uc,'_bounds')];
        end
        
    end
    
end