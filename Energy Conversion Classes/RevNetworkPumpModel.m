classdef RevNetworkPumpModel < Component
    
    % RevNetworkPumpModel Static MLD system model of a reversible network
    % water pump at fixed speed.
    
    properties (SetAccess = private, GetAccess = public)
        onPwr   % fixed electrical power consumption when running
        qPwr    % coefficient of variable electrical power consumption
        qConv   % factor to convert thermal power into equivalent water flow
    end
    
    
    methods
        
        function self = RevNetworkPumpModel(name,N,bigM,onPwr,qPwr,qConv)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.onPwr = onPwr;
            self.qPwr = qPwr;
            self.qConv = qConv;
            
            %initialise variables
            self.y = self.get_y;
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
            varName_y_in = [self.name,'_y_in'];
            varName_y_out = [self.name,'_y_out'];
            
            self.vars.(matlab.lang.makeValidName(varName_y_e)) = sdpvar(self.N,1); %define sdpvar in vars struct
            self.vars.(matlab.lang.makeValidName(varName_y_in)) = sdpvar(self.N,1);
            self.vars.(matlab.lang.makeValidName(varName_y_out)) = sdpvar(self.N,1);
            
            y = [y;varName_y_e;varName_y_in;varName_y_out];
            
            self.srcPorts = {varName_y_out};
            self.snkPorts = {varName_y_e;varName_y_in};
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
            %construct constraints for ThermalNetworkPumpModel
                        
            varName_y_e = [self.name,'_y_e']; %sink port
            varName_y_in = [self.name,'_y_in']; %sink port
            varName_y_out = [self.name,'_y_out']; %source port
            varName_z_fwd = [self.name,'_z_fwd'];
            varName_d_fwd = [self.name,'_d_fwd'];
            
            var_y_e = self.vars.(matlab.lang.makeValidName(varName_y_e)); %get variable
            var_y_in = self.vars.(matlab.lang.makeValidName(varName_y_in));
            var_y_out = self.vars.(matlab.lang.makeValidName(varName_y_out));
            var_z_fwd = self.vars.(matlab.lang.makeValidName(varName_z_fwd));
            var_d_fwd = self.vars.(matlab.lang.makeValidName(varName_d_fwd));
            
            %MLD constraints
            coef = self.onPwr*self.qPwr;
            
            C = C + [(var_y_e == self.onPwr + (coef + coef)*var_z_fwd - coef*var_y_in):varName_y_e]; %tagging is used to identify constraints
            C = C + [(var_y_out == var_y_in):varName_y_out];
            C = C + [(self.bigM*var_d_fwd <= var_y_in + self.bigM):strcat(varName_y_in,'_bigM_lower')];
            C = C + [(-(self.bigM + eps)*var_d_fwd <= -var_y_in - eps):strcat(varName_y_in,'_bigM_upper')];
            C = C + [(self.bigM*var_d_fwd + var_z_fwd <= var_y_in + self.bigM):strcat(varName_z_fwd,'=d*y_in1')];
            C = C + [(self.bigM*var_d_fwd - var_z_fwd <= -var_y_in + self.bigM):strcat(varName_z_fwd,'=d*y_in2')];
            C = C + [(-self.bigM*var_d_fwd + var_z_fwd <= 0):strcat(varName_z_fwd,'_bigM_upper')];
            C = C + [(-self.bigM*var_d_fwd - var_z_fwd <= 0):strcat(varName_z_fwd,'_bigM_lower')];
            
            %bound constraints
            C = C + [(0 <= var_y_e <= self.bigM):strcat(varName_y_e,'_bounds')];
            C = C + [(-self.bigM <= var_y_in <= self.bigM):strcat(varName_y_in,'_bounds')];
            C = C + [(-self.bigM <= var_y_out <= self.bigM):strcat(varName_y_out,'_bounds')];
        end
        
    end
    
end