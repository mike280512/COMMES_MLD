classdef TransformerModel < Component
    
    % TransformerModel Static MLD system model to account for forward and
    % reverse efficiencies through a lossy device.
    
    properties (SetAccess = private, GetAccess = public)
        etaTx   % conversion efficiency through transformer
    end
    
    
    methods
        
        function self = TransformerModel(name,N,bigM,etaTx)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.etaTx = etaTx;
            
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
            
            varName_y_in = [self.name,'_y_in'];
            varName_y_out = [self.name,'_y_out'];
            
            self.vars.(matlab.lang.makeValidName(varName_y_in)) = sdpvar(self.N,1); %define sdpvar in vars struct
            self.vars.(matlab.lang.makeValidName(varName_y_out)) = sdpvar(self.N,1);
            
            y = [y;varName_y_in;varName_y_out];
            
            self.srcPorts = {varName_y_out};
            self.snkPorts = {varName_y_in};
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
            %construct constraints for TransformerModel
                        
            varName_y_in = [self.name,'_y_in']; %sink port
            varName_y_out = [self.name,'_y_out']; %source port
            varName_z_fwd = [self.name,'_z_fwd'];
            varName_d_fwd = [self.name,'_d_fwd'];
            
            var_y_in = self.vars.(matlab.lang.makeValidName(varName_y_in)); %get variable
            var_y_out = self.vars.(matlab.lang.makeValidName(varName_y_out));
            var_z_fwd = self.vars.(matlab.lang.makeValidName(varName_z_fwd));
            var_d_fwd = self.vars.(matlab.lang.makeValidName(varName_d_fwd));
            
            %MLD constraints
            C = C + [(var_y_out == (self.etaTx - 1/self.etaTx)*var_z_fwd + 1/self.etaTx*var_y_in):varName_y_out]; %tagging is used to identify constraints
            C = C + [(self.bigM*var_d_fwd <= var_y_in + self.bigM):strcat(varName_y_in,'_bigM_lower')];
            C = C + [(-(self.bigM + eps)*var_d_fwd <= -var_y_in - eps):strcat(varName_y_in,'_bigM_upper')];
            C = C + [(self.bigM*var_d_fwd + var_z_fwd <= var_y_in + self.bigM):strcat(varName_z_fwd,'=d*y_in_1')];
            C = C + [(self.bigM*var_d_fwd - var_z_fwd <= -var_y_in + self.bigM):strcat(varName_z_fwd,'=d*y_in_2')];
            C = C + [(-self.bigM*var_d_fwd + var_z_fwd <= 0):strcat(varName_z_fwd,'_bigM_upper')];
            C = C + [(-self.bigM*var_d_fwd - var_z_fwd <= 0):strcat(varName_z_fwd,'_bigM_lower')];
        end
        
    end
    
end