classdef ChillerModel < Component
    
    % ChillerModel Static MLD system model of an irreversible
    % chiller
    
    properties (SetAccess = private, GetAccess = public)
        EER   % Coefficient of Performance of heat pump
    end
    
    
    methods
        
        function self = ChillerModel(name,N,bigM,EER)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.EER = EER;
            
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
            varName_y_c = [self.name,'_y_c'];
            
            self.vars.(matlab.lang.makeValidName(varName_y_e)) = sdpvar(self.N,1); %define sdpvar in vars struct
            self.vars.(matlab.lang.makeValidName(varName_y_c)) = sdpvar(self.N,1);
            
            y = [y;varName_y_e;varName_y_c];
            
            self.snkPorts = {varName_y_e;varName_y_c};
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
            %construct constraints for ChillerModel
                        
            varName_y_e = [self.name,'_y_e']; %sink port
            varName_y_c = [self.name,'_y_c']; %sink port
            varName_uc = [self.name,'_uc'];
            
            var_y_e = self.vars.(matlab.lang.makeValidName(varName_y_e)); %get variable
            var_y_c = self.vars.(matlab.lang.makeValidName(varName_y_c));
            var_uc = self.vars.(matlab.lang.makeValidName(varName_uc));
            
            %MLD constraints
            C = C + [(var_y_e == var_uc):varName_y_e]; %tagging is used to identify constraints
            C = C + [(var_y_c == (self.EER)*var_uc):varName_y_c];
            
            %bound constraints
            C = C + [(0 <= var_y_e <= self.bigM):strcat(varName_y_e,'_bounds')];
            C = C + [(0 <= var_y_c <= self.bigM):strcat(varName_y_c,'_bounds')];
            C = C + [(0 <= var_uc <= self.bigM):strcat(varName_uc,'_bounds')];
        end
        
    end
    
end