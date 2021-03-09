classdef RevAirSourceHPModel < Component
    
    % RevAirSourceHPModel Static MLD system model of a reversible
    % air-source heat pump
    
    properties (SetAccess = private, GetAccess = public)
        COP     % Coefficient of Performance of heat pump
        EER     % Energy Efficiency Ratio
    end
    
    
    methods
        
        function self = RevAirSourceHPModel(name,N,bigM,COP,EER)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.COP = COP;
            self.EER = EER;
            
            %initialise variables
            self.y = self.get_y;
            self.uc = self.get_uc;
            self.z = self.get_z;
            self.d = self.get_d;
            
            %initialise constraints
            self.C = self.get_C;
        end%constructor
        
        
        %% OBJECT PROPERTY GET METHODS
        function y = get_y(self)
            %initialies MLD system output variables and returns cell array
            y = self.y;
            
            varName_y_e = [self.name,'_y_e'];
            varName_y_c = [self.name,'_y_c'];
            varName_y_h = [self.name,'_y_h'];
            
            self.vars.(matlab.lang.makeValidName(varName_y_e)) = sdpvar(self.N,1); %define sdpvar in vars struct
            self.vars.(matlab.lang.makeValidName(varName_y_c)) = sdpvar(self.N,1);
            self.vars.(matlab.lang.makeValidName(varName_y_h)) = sdpvar(self.N,1);
            
            y = [y;varName_y_e;varName_y_c;varName_y_h];
            
            self.srcPorts = {varName_y_h};
            self.snkPorts = {varName_y_e;varName_y_c};
        end
        
        function uc = get_uc(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            uc = self.uc;
            
            varName_uc = [self.name,'_uc'];
            
            self.vars.(matlab.lang.makeValidName(varName_uc)) = sdpvar(self.N,1);
            
            uc = [uc;varName_uc];
        end
        
        function z = get_z(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            z = self.z;
            
            varName_z_fwd = [self.name,'_z_fwd'];
            
            self.vars.(matlab.lang.makeValidName(varName_z_fwd)) = sdpvar(self.N,1);
            
            z = [z;varName_z_fwd];
        end
        
        function d = get_d(self)
            %initialies MLD system integer auxiliary variables and returns cell array
            d = self.d;
            
            varName_d_fwd = [self.name,'_d_fwd'];
            
            self.vars.(matlab.lang.makeValidName(varName_d_fwd)) = binvar(self.N,1);
            
            d = [d;varName_d_fwd];
        end
        
        function C = get_C(self)
            C = self.C;
            %construct constraints for RevAirSourceHPModel
                        
            varName_y_e = [self.name,'_y_e']; %sink port
            varName_y_c = [self.name,'_y_c']; %sink port
            varName_y_h = [self.name,'_y_h']; %source port
            varName_uc = [self.name,'_uc'];
            varName_z_fwd = [self.name,'_z_fwd'];
            varName_d_fwd = [self.name,'_d_fwd'];
            
            var_y_e = self.vars.(matlab.lang.makeValidName(varName_y_e)); %get variable
            var_y_c = self.vars.(matlab.lang.makeValidName(varName_y_c));
            var_y_h = self.vars.(matlab.lang.makeValidName(varName_y_h));
            var_uc = self.vars.(matlab.lang.makeValidName(varName_uc));
            var_z_fwd = self.vars.(matlab.lang.makeValidName(varName_z_fwd));
            var_d_fwd = self.vars.(matlab.lang.makeValidName(varName_d_fwd));
            
            %MLD constraints
            C = C + [(var_y_e == var_uc):varName_y_e]; %tagging is used to identify constraints
            C = C + [(var_y_c == self.EER*var_uc - self.EER*var_z_fwd):varName_y_c];
            C = C + [(var_y_h == self.COP*var_z_fwd):varName_y_h];
            C = C + [(-self.bigM*var_d_fwd <= -var_y_h):strcat(varName_y_h,'_bigM_upper')];
            C = C + [(eps*var_d_fwd <= var_y_h):strcat(varName_y_h,'_bigM_lower')];
            C = C + [(self.bigM*var_d_fwd + var_z_fwd <= var_uc + self.bigM):strcat(varName_z_fwd,'=d*uc_1')];
            C = C + [(self.bigM*var_d_fwd - var_z_fwd <= -var_uc + self.bigM):strcat(varName_z_fwd,'=d*uc_2')];
            C = C + [(-self.bigM*var_d_fwd + var_z_fwd <= 0):strcat(varName_z_fwd,'_bigM_upper')];
            C = C + [(eps*var_d_fwd - var_z_fwd <= 0):strcat(varName_z_fwd,'_bigM_lower')];
            
            %bound constraints
            C = C + [(0 <= var_y_e <= self.bigM):strcat(varName_y_e,'_bounds')];
            C = C + [(0 <= var_y_c <= self.bigM):strcat(varName_y_c,'_bounds')];
            C = C + [(0 <= var_y_h <= self.bigM):strcat(varName_y_h,'_bounds')];
            C = C + [(0 <= var_uc <= self.bigM):strcat(varName_uc,'_bounds')];
        end
        
    end
    
end