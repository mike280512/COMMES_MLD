classdef SDemandLC < matlab.System
    %SDemandLC Determines the inputs to be fed into eMPC controller
    %(YALMIP optimizer class) based on system status updates.
    
    properties (SetAccess = private, GetAccess = public)
        N               %controller prediction horizon length (scalar)
        Nn              %number of segments per energy consumption cycle (scalar)
        Nc              %number of sampling periods per energy consumption cycle (scalar)
    end    
    
    properties (SetAccess = public, GetAccess = public)
        %user-specified tunable initial parameters
        E               %initial segment energy requirement profile
        timePref        %binary vector for periods when segments cannot be processed
    end

    properties(DiscreteState)
        online          %log of time online in current cycle
        %parameters updated at each scheduling period:
        m_E             %updated segment energy requirement profile 
        m_pref          %updated time preferences
        m_rem           %updated binary vector to indicate remaining periods in which segments can be processed
    end

    methods
        function self = SDemandLC(FlexDemandObj,E,Nc,varargin)
            %values inherited from demand component
            self.N = FlexDemandObj.N;
            self.Nn = FlexDemandObj.Nn;
            self.Nc = Nc; %determines nature of componenent reset. Nc must be long enough to process all segments.
            
            %assign provided baseline energy requirement
            assert(size(E,2)>=self.Nn,'Energy consumption cycle baseline energy requirement must have dimensions equal to the number of segments.');
            self.E = E;
            
            %default values
            self.timePref = zeros(self.N,1); %may be scheduled in any period
            
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
                
                eMPCInputArray = {self.m_E,m_proc,m_comp,self.m_rem,self.m_pref};
                
                %increment online state
                self.online = self.online + 1;
                return
            end
            
            %reset discrete state properties if reached end of cycle time
            if self.online == self.Nc
                self.resetImpl;
                m_proc = zeros(1,self.Nn);
                m_comp = zeros(1,self.Nn);
                
                eMPCInputArray = {self.m_E,m_proc,m_comp,self.m_rem,self.m_pref};
                
                %increment online state
                self.online = self.online + 1;
                return
            end
            
            %during cycle discrete state properties are updated as normal
            self.m_E = self.m_E - uc_l; %update baseline energy requirements for each segment
                        
            %shift time indicators along one and replace last element with
            %0
            self.m_pref = circshift(self.m_pref,-1); self.m_pref(end) = 0;
            self.m_rem = circshift(self.m_rem,-1); self.m_rem(end) = 0;
            
            for i = 1:self.Nn
                if self.m_E(i) < 1e-5 && m_proc(i) == 1
                    self.m_E(i) = 0;
                    m_comp(i) = 1;
                end
            end
            
            eMPCInputArray = {self.m_E,m_proc,m_comp,self.m_rem,self.m_pref};
            
            %increment online state
            self.online = self.online + 1;
        end

        function resetImpl(self)
            % Initialize / reset discrete-state properties
            self.online = 0;
            self.m_E = self.E;
            self.m_rem = zeros(self.N,1);
            self.m_rem(1:self.Nc) = 1;
            self.m_pref = self.timePref;
        end

        function releaseImpl(self)
            % Release resources
        end
    end
end

