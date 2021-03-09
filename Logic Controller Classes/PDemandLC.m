classdef PDemandLC < matlab.System
    %PDemandLC Determines the inputs to be fed into eMPC controller
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
        procMin         %minimum number of periods for processing segments
        procMax         %maximum number of periods for processing segments
    end

    properties(DiscreteState)
        online          %log of time online in current cycle
        %parameters updated at each scheduling period:
        m_E             %updated segment energy requirement profile
        m_procMin       %updated minimum number of periods for processing segments
        m_procMax       %updated maximum number of periods for processing segments
    end

    methods
        function self = PDemandLC(FlexDemandObj,E,Nc,varargin)
            %values inherited from demand component
            self.N = FlexDemandObj.N;
            self.Nn = FlexDemandObj.Nn;
            self.Nc = Nc; %determines nature of componenent reset. Nc is equal to number of periods in one segment
            
            %assign provided baseline energy requirement
            assert(size(E,2)>=self.Nn,'Energy consumption cycle baseline energy requirement must have dimensions greater or equal to the number of segments.');
            self.EStart = E;
            self.E = E;
            
            %default values
            self.procMin = ones(1,self.Nn);
            self.procMax = Nc*ones(1,self.Nn);
            
            %enable user inputs through named properties on instantiation
            if ~isempty(varargin)
                setProperties(self,length(varargin),varargin{:});
            end
            
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
                
                eMPCInputArray = {self.m_E,m_proc,m_comp,self.m_procMin,self.m_procMax};
                
                %increment online state
                self.online = self.online + 1;
                return
            end
            
            %reset discrete state properties if reached end of cycle time
            if self.online == self.Nc
                self.resetImpl;
                m_proc = zeros(1,self.Nn);
                m_comp = zeros(1,self.Nn);
                
                eMPCInputArray = {self.m_E,m_proc,m_comp,self.m_procMin,self.m_procMax};
                
                %increment online state
                self.online = self.online + 1;
                return
            end
            
            %during cycle discrete state properties are updated as normal
            self.m_E = self.m_E - uc_l; %update baseline energy requirements for each segment
            
            for i = 1:self.Nn
                if m_proc(i) == 1
                    if self.m_procMin(i) > 0
                        self.m_procMin(i) = self.m_procMin(i) - 1;
                    end
                    if self.m_procMax(i) > 0
                        self.m_procMax(i) = self.m_procMax(i) - 1;
                    end
                    if self.m_E(i) < 1e-6
                        m_comp(i) = 1;
                    end
                end
            end
            
            %if energy consumption is met for first segment then discrete 
            %state properties are reset to begin new iterated energy 
            %consumption cycle ***may lead to infeasibility due to change 
            %in expected schedule***
            if m_comp(1) == 1
                self.resetImpl;
                m_proc = zeros(1,self.Nn);
                m_comp = zeros(1,self.Nn);
                
                eMPCInputArray = {self.m_E,m_proc,m_comp,self.m_procMin,self.m_procMax};
                
                %increment online state
                self.online = self.online + 1;
                return
            end
            
            eMPCInputArray = {self.m_E,m_proc,m_comp,self.m_procMin,self.m_procMax};
            
            %increment online state
            self.online = self.online + 1;
        end

        function resetImpl(self)
            % Initialize / reset discrete-state properties
            self.online = 0;
            self.m_E = self.E(1:self.Nn);
            self.E = circshift(self.E,-1);

            self.m_procMin = self.procMin;
            self.m_procMax = self.procMax;
        end

        function releaseImpl(self)
            % Release resources
            self.E = self.EStart; %to fully reset controller to initial state on next call
        end
    end
end

