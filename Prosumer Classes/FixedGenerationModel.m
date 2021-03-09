classdef FixedGenerationModel < Component
    
    % FixedGenerationModel Static MLD system model to allow input of known
    % disturbance forecast
    
    % properties inherited from Component class
    
    methods
        
        function self = FixedGenerationModel(name,N)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = [];
            
            %initialise variables
            self.y = self.get_y;
            self.w = self.get_w;
            
            %initialise constraints
            self.C = self.get_C;
        end%constructor
        
        
        %% OBJECT PROPERTY GET METHODS
        function y = get_y(self)
            %initialies MLD system output variables and returns cell array
            y = self.y;
            
            varName_y = [self.name,'_y'];
            
            self.vars.(matlab.lang.makeValidName(varName_y)) = sdpvar(self.N,1); %define sdpvar in vars struct
            
            y = [y;varName_y];
            
            self.srcPorts = {varName_y};
        end
        
        function w = get_w(self)
            %initialies MLD system known disturbance variables and returns cell array
            w = self.w;
            
            varName_w = [self.name,'_w'];
            
            self.vars.(matlab.lang.makeValidName(varName_w)) = sdpvar(self.N,1); %define sdpvar in vars struct
            
            w = [w;varName_w];
        end
        
        function C = get_C(self)
            C = self.C;
            %construct constraints for FixedGenerationModel
                        
            varName_y = [self.name,'_y']; %sink port
            varName_w = [self.name,'_w'];
            
            var_y = self.vars.(matlab.lang.makeValidName(varName_y)); %get variable
            var_w = self.vars.(matlab.lang.makeValidName(varName_w));
            
            %MLD constraints
            C = C + [(var_y == var_w):varName_y];
        end
        
    end
    
end