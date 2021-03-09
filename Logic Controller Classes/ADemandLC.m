classdef ADemandLC < matlab.System
    %ADemandLC Determines the inputs to be fed into eMPC controller
    %(YALMIP optimizer class) based on system status updates.
    
    properties (SetAccess = private, GetAccess = public)
        N               %controller prediction horizon length (scalar)
        Nn              %number of segments per energy consumption cycle (scalar)
        Nc              %number of sampling periods per energy consumption cycle (scalar)
    end    
    
    properties (SetAccess = public, GetAccess = public)
        %user-specified tunable initial parameters
        E               %segment energy requirement profile
        EStart          %initial energy requirement profile for controller reset
    end

    properties(DiscreteState)
        online          %log of time online in current cycle
        %parameters updated at each scheduling period:
        m_E             %updated segment energy requirement profile          
    end

    methods
        function self = ADemandLC(FlexDemandObj,E)
            %values inherited from demand component
            self.N = FlexDemandObj.N;
            self.Nn = FlexDemandObj.Nn;
            self.Nc = 1; %determines nature of componenent reset. Nc is equal to periods in one segment, i.e 1
            
            %assign provided baseline energy requirement
            assert(size(E,2)>=self.Nn,'Energy consumption cycle baseline energy requirement must have dimensions greater or equal to the number of segments.');
            self.EStart = E;
            self.E = E;
            
        end            
    end

    methods(Access = protected)
        function eMPCInputArray = stepImpl(self,uc_l,m_proc,m_comp)
            %returns an array of input parameters to be passed to eMPC
            %controller. Inputs are decision variables calculated by eMPC
            %controller in previous time step.

            %on first use self.resetImpl is automatically called - return
            %initial values without a second call to resetImpl
            if self.online == 0
                m_proc = zeros(1,self.Nn);
                m_comp = zeros(1,self.Nn);
                
                eMPCInputArray = {self.m_E,m_proc,m_comp};
                
                %increment online state
                self.online = self.online + 1;
                return
            end
            
            %reset discrete state properties if reached end of cycle time
            if self.online == self.Nc
                self.resetImpl;
                m_proc = zeros(1,self.Nn);
                m_comp = zeros(1,self.Nn);
                
                eMPCInputArray = {self.m_E,m_proc,m_comp};
                
                %increment online state
                self.online = self.online + 1;
                return
            end
            
            %during cycle discrete state properties are updated as normal
            %** should not ever be executed due to above if statements -
            %only adjustable demand has Nc == 1 **
            self.m_E = self.m_E - uc_l; %update baseline energy requirements for each segment
            
            eMPCInputArray = {self.m_E,m_proc,m_comp};
            
            %increment online state
            self.online = self.online + 1;
        end

        function resetImpl(self)
            % Initialize / reset discrete-state properties
            self.online = 0;
            self.m_E = self.E(1:self.Nn);
            self.E = circshift(self.E,-1);
        end

        function releaseImpl(self)
            % Release resources
            self.E = self.EStart; %to fully reset controller to initial state on next call
        end
    end
end

