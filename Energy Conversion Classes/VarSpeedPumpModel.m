classdef VarSpeedPumpModel < Component
    
    % VarSpeedPumpModel Static MLD system model of an irreversible network
    % water pump at fixed speed.
    
    properties (SetAccess = private, GetAccess = public)
        etaMotor   % fixed motor efficiency
        etaHydra   % fixed hydraulic efficiency
    end
    
    
    methods
        
        function self = VarSpeedPumpModel(name,N,bigM,etaMotor,etaHydra)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.etaMotor = etaMotor;
            self.etaHydra = etaHydra;
            
            %initialise variables
            self.y = self.get_y;
            self.z = self.get_z;
            self.d = self.get_d;
            self.m = self.get_m;
            
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
            varName_y_DH = [self.name,'_y_DH'];
            
            self.vars.(matlab.lang.makeValidName(varName_y_e)) = sdpvar(self.N,1); %define sdpvar in vars struct
            self.vars.(matlab.lang.makeValidName(varName_y_in)) = sdpvar(self.N,1);
            self.vars.(matlab.lang.makeValidName(varName_y_out)) = sdpvar(self.N,1);
            self.vars.(matlab.lang.makeValidName(varName_y_DH)) = sdpvar(self.N,1);
            
            y = [y;varName_y_e;varName_y_in;varName_y_out;varName_y_DH];
            
            self.srcPorts = {varName_y_out};
            self.snkPorts = {varName_y_e;varName_y_in;varName_y_DH};
        end
        
        function z = get_z(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            z = self.z;
            
            varName_z_fwd = [self.name,'_z_fwd'];
%             varName_z_bilin = [self.name,'_z_bilin'];
            
            self.vars.(matlab.lang.makeValidName(varName_z_fwd)) = sdpvar(self.N,1);
%             self.vars.(matlab.lang.makeValidName(varName_z_bilin)) = sdpvar(self.N,1);
            
            z = [z;varName_z_fwd];%;varName_z_bilin];
        end
        
        function d = get_d(self)
            %initialies MLD system integer auxiliary variables and returns cell array
            d = self.d;
            
            varName_d_fwd = [self.name,'_d_fwd'];
            
            self.vars.(matlab.lang.makeValidName(varName_d_fwd)) = binvar(self.N,1);
            
            d = [d;varName_d_fwd];
        end
        
        function m = get_m(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            m = self.m;
            
            varName_m_minDH = [self.name,'_m_minDH']; %define variable name            
                
            self.vars.(matlab.lang.makeValidName(varName_m_minDH)) = sdpvar(self.N,1); %define sdpvar in vars struct
            
            m = [m;varName_m_minDH];
        end
        
        function C = get_C(self)
            C = self.C;
            %construct constraints
                        
            varName_y_e = [self.name,'_y_e']; %sink port
            varName_y_in = [self.name,'_y_in']; %sink port
            varName_y_out = [self.name,'_y_out']; %source port
            varName_y_DH = [self.name,'_y_DH']; %sink port
            varName_m_minH = [self.name,'_m_minDH'];
            varName_z_fwd = [self.name,'_z_fwd'];
%             varName_z_bilin = [self.name,'_z_bilin'];
            varName_d_fwd = [self.name,'_d_fwd'];
            
            var_y_e = self.vars.(matlab.lang.makeValidName(varName_y_e)); %get variable
            var_y_in = self.vars.(matlab.lang.makeValidName(varName_y_in));
            var_y_out = self.vars.(matlab.lang.makeValidName(varName_y_out));
            var_y_DH = self.vars.(matlab.lang.makeValidName(varName_y_DH));
            var_m_minDH = self.vars.(matlab.lang.makeValidName(varName_m_minH));
            var_z_fwd = self.vars.(matlab.lang.makeValidName(varName_z_fwd));
%             var_z_bilin = self.vars.(matlab.lang.makeValidName(varName_z_bilin));
            var_d_fwd = self.vars.(matlab.lang.makeValidName(varName_d_fwd));
            
            %MLD constraints
            coef = 9.81/self.etaMotor/self.etaHydra/1000; %P[W] = m[kg/s]*g[m/s^2]*H[m]/eff => P[kW] = m[kg/s]*g[m/s^2]*H[m]/1000/eff
            
            C = C + [(var_y_out == var_y_in):varName_y_out]; %tagging is used to identify constraints
            C = C + [(var_y_e == 2*coef*var_z_fwd.*var_y_DH - coef*var_y_in.*var_y_DH):varName_y_e];
%             C = C + [(var_z_bilin == var_z_fwd.*var_y_DH):varName_z_bilin]; %YALMIP doesnt seem to play nice when bilinear terms used in constraints involving other variables (bug?) - hidden with auxiliary variable
            C = C + [(self.bigM*var_d_fwd <= var_y_in + self.bigM):strcat(varName_y_in,'_bigM_lower')];
            C = C + [(-(self.bigM + eps)*var_d_fwd <= -var_y_in - eps):strcat(varName_y_in,'_bigM_upper')];
            C = C + [(self.bigM*var_d_fwd + var_z_fwd <= var_y_in + self.bigM):strcat(varName_z_fwd,'=d*y_in1')];
            C = C + [(self.bigM*var_d_fwd - var_z_fwd <= -var_y_in + self.bigM):strcat(varName_z_fwd,'=d*y_in2')];
            C = C + [(-self.bigM*var_d_fwd + var_z_fwd <= 0):strcat(varName_z_fwd,'_bigM_upper')];
            C = C + [(-self.bigM*var_d_fwd - var_z_fwd <= 0):strcat(varName_z_fwd,'_bigM_lower')];
            
            %bound constraints
            C = C + [(0 <= var_y_e <= self.bigM):strcat(varName_y_e,'_bounds')];
            C = C + [(-self.bigM <= var_y_in <= self.bigM):strcat(varName_y_in,'_bounds')];
            C = C + [(var_m_minDH <= var_y_DH <= self.bigM):strcat(varName_y_DH,'_bounds')]; %if user does not supply m_minDH...
            C = C + [(0 <= var_m_minDH <= self.bigM):strcat(varName_m_minH,'_bounds')]; % ...default is 0
        end
        
    end
    
end