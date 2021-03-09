classdef HotTankModel < Component
    
    % HotTankModel MLD system model of a hot water tank
    
    properties (SetAccess = private, GetAccess = public)
        alpha       %standby loss for sampling interval
        dT          %sampling interval length
    end
    
    
    methods
        
        function self = HotTankModel(name,N,bigM,alpha,dT)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.alpha = alpha;
            self.dT = dT;
            
            %initialise variables
            self.x = self.get_x;
            self.uc = self.get_uc;
            self.y = self.get_y;
            self.m = self.get_m;
            
            %initialise constraints
            self.C = self.get_C;
        end%constructor
        
        
        %% OBJECT PROPERTY GET METHODS
        function x = get_x(self)
            %initialies MLD system state variables and returns cell array
            x = self.x;
            
            varName_x = [self.name,'_x'];
            
            self.vars.(matlab.lang.makeValidName(varName_x)) = sdpvar(self.N,1); %define sdpvar in vars struct
            
            x = [x;varName_x];
        end
        
        function uc = get_uc(self)
            %initialies MLD system input variables and returns cell array
            uc = self.uc;
            
            varName_uc_chg = [self.name,'_uc_chg'];
            
            self.vars.(matlab.lang.makeValidName(varName_uc_chg)) = sdpvar(self.N,1);
            
            uc = [uc;varName_uc_chg];
        end
        
        function y = get_y(self)
            %initialies MLD system output variables and returns cell array
            y = self.y;
            
            varName_y_chg = [self.name,'_y_chg'];
            
            self.vars.(matlab.lang.makeValidName(varName_y_chg)) = sdpvar(self.N,1);
            
            y = [y;varName_y_chg];
            
            self.snkPorts = {varName_y_chg};
        end
        
        function m = get_m(self)
            %initialies MLD system measurement variables and returns cell array
            m = self.m;
            
            varName_m = [self.name,'_m_x'];
            
            self.vars.(matlab.lang.makeValidName(varName_m)) = sdpvar(1,1);
            
            m = [m;varName_m];
        end
        
        function C = get_C(self)
            C = self.C;
            %construct constraints for HotTankModel
                        
            varName_x = [self.name,'_x'];
            varName_uc_chg = [self.name,'_uc_chg'];
            varName_y_chg = [self.name,'_y_chg']; %sink port
            varName_m_x = [self.name,'_m_x'];
            
            var_x = self.vars.(matlab.lang.makeValidName(varName_x)); %get variable
            var_uc_chg = self.vars.(matlab.lang.makeValidName(varName_uc_chg));
            var_y_chg = self.vars.(matlab.lang.makeValidName(varName_y_chg));
            var_m_x = self.vars.(matlab.lang.makeValidName(varName_m_x));
            
            
            %state-space constraints
            C = C + [(var_x(1) == self.alpha*var_m_x + self.dT*var_uc_chg(1)):strcat(varName_x,'_0')];
            C = C + [(var_x(2:self.N) == self.alpha*var_x(1:self.N-1) + self.dT*var_uc_chg(2:self.N)):strcat(varName_x,'_1:N-1')];
            
            %MLD constraints
            C = C + [(var_y_chg == var_uc_chg):varName_y_chg];
            
            %bound constraints
            C = C + [(0 <= var_x <= self.bigM):strcat(varName_x,'_bounds')];
            C = C + [(-self.bigM <= var_uc_chg <= self.bigM):strcat(varName_uc_chg,'_bounds')];
            C = C + [(-self.bigM <= var_y_chg <= self.bigM):strcat(varName_y_chg,'_bounds')];

        end
        
    end
    
end