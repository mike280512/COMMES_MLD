classdef CHPModel < Component
    
    % CHPModel Static MLD system model of a combined heat and power device
    % with one input and two outputs.
    
    properties (SetAccess = private, GetAccess = public)
        etaE    % conversion efficiency for electricity production
        etaH    % conversion efficiency for heat production
    end
    
    
    methods
        
        function self = CHPModel(name,N,bigM,etaE,etaH)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.etaE = etaE;
            self.etaH = etaH;
            
            %initialise variables
            self.uc = self.get_uc;
            self.y = self.get_y;
            
            %initialise constraints
            self.C = self.get_C;
        end%constructor
        
        
        %% OBJECT PROPERTY GET METHODS
        
        function uc = get_uc(self)
            %initialies MLD system continuous auxiliary variables and returns cell array
            uc = self.uc;
            
            varName_uc_in = [self.name,'_uc_in'];
            
            self.vars.(matlab.lang.makeValidName(varName_uc_in)) = sdpvar(self.N,1);
            
            uc = [uc;varName_uc_in];
        end
        
        function y = get_y(self)
            %initialies MLD system output variables and returns cell array
            y = self.y;
            
            varName_y_in = [self.name,'_y_in']; %sink port
            varName_y_e = [self.name,'_y_e']; %source port
            varName_y_h = [self.name,'_y_h']; %source port
            
            self.vars.(matlab.lang.makeValidName(varName_y_in)) = sdpvar(self.N,1); %define sdpvar in vars struct
            self.vars.(matlab.lang.makeValidName(varName_y_e)) = sdpvar(self.N,1);
            self.vars.(matlab.lang.makeValidName(varName_y_h)) = sdpvar(self.N,1);
            
            y = [y;varName_y_in;varName_y_e;varName_y_h];
            
            self.srcPorts = {varName_y_e;varName_y_h};
            self.snkPorts = {varName_y_in};
        end
        
        function C = get_C(self)
            C = self.C;
            %construct constraints for CHPModel
                      
            varName_y_in = [self.name,'_y_in'];
            varName_y_e = [self.name,'_y_e'];
            varName_y_h = [self.name,'_y_h'];
            varName_uc_in = [self.name,'_uc_in'];
            
            var_y_in = self.vars.(matlab.lang.makeValidName(varName_y_in));
            var_y_e = self.vars.(matlab.lang.makeValidName(varName_y_e)); %get variable
            var_y_h = self.vars.(matlab.lang.makeValidName(varName_y_h));
            var_uc_in = self.vars.(matlab.lang.makeValidName(varName_uc_in));
            
            %MLD constraints
            C = C + [(var_y_in == var_uc_in):varName_y_in]; %tagging is used to identify constraints
            C = C + [(var_y_e == self.etaE*var_uc_in):varName_y_e];
            C = C + [(var_y_h == self.etaH*var_uc_in):varName_y_h];
            
            %bound constraints
            C = C + [(0 <= var_y_in <= self.bigM):strcat(varName_y_in,'_bounds')]; %tagging is used to identify constraints
            C = C + [(0 <= var_y_e <= self.bigM):strcat(varName_y_e,'_bounds')];
            C = C + [(0 <= var_y_h <= self.bigM):strcat(varName_y_h,'_bounds')];
            C = C + [(0 <= var_uc_in <= self.bigM):strcat(varName_uc_in,'_bounds')];
        end
        
    end
    
end