classdef Aggregation < Component
    
    % Aggregation An aggregation class used to combine components into a
    % single model by merging object properties.
    % Requires use of @Component.connect() to connect sub-component 
    % variables using equality constraints.
    
    properties (SetAccess = private, GetAccess = public)
        %inherits from component class        
    end
    
    methods
        %constructor
        function self = Aggregation(name,N,varargin)
            
            for i = 1:length(varargin)
                assert(isa(varargin{i},'Component'),'Sub-components must be objects inherited from the Componenet class.')
            end
            
            %call parent class constructor
            self@Component(name,N,varargin{:})
            
            self.vars = self.get_allVars;
            self.C = self.get_C;
            self.J = self.get_J;
            self.get_varLists;

            self.bigM = [];
        end
        
        function Vars = get_allVars(self)
            %returns merged vars struct from subComponents.
            Vars = self.subComponents{1}.vars;
            
            for i = 2:length(self.subComponents)
                subVars = self.subComponents{i}.vars;
                Vars = self.merge_vars(Vars,subVars);                
            end
        end

        function C = get_C(self)
            %returns merged YALMIP LMI object from subComponents.
            C = self.C;
                        
            for i = 1:length(self.subComponents)
                C = C + self.subComponents{i}.C;       
            end
        end
        
        function J = get_J(self)
            %returns merged YALMIP cost objective from subComponents.
            J = self.subComponents{1}.J;
                        
            for i = 2:length(self.subComponents)
                J = J + self.subComponents{i}.J;       
            end
        end
        
        function get_varLists(self)
            %returns merged arrays of variables from subComponents.
            sub_len = length(self.subComponents);
            
            %pre-allocate cell arrays with excess dimensions
            x = cell(sub_len,1);
            uc = cell(sub_len,1);
            ud = cell(sub_len,1);
            z = cell(sub_len,1);
            d = cell(sub_len,1);
            y = cell(sub_len,1);
            w = cell(sub_len,1);
            l = cell(sub_len,1);
            m = cell(sub_len,1);
            srcPorts = cell(sub_len,1);
            snkPorts = cell(sub_len,1);
                                   
            %assign values from subComponents
            for i = 1:sub_len
                x{i,1} = self.subComponents{i}.x;
                uc{i,1} = self.subComponents{i}.uc;
                ud{i,1} = self.subComponents{i}.ud;
                z{i,1} = self.subComponents{i}.z;
                d{i,1} = self.subComponents{i}.d;
                y{i,1} = self.subComponents{i}.y;
                w{i,1} = self.subComponents{i}.w;
                l{i,1} = self.subComponents{i}.l;
                m{i,1} = self.subComponents{i}.m;
                srcPorts{i,1} = self.subComponents{i}.srcPorts;
                snkPorts{i,1} = self.subComponents{i}.snkPorts;
            end            
            
            %assign temporary cell arrays to object properties
            self.x = vertcat(x{:});
            self.uc = vertcat(uc{:});
            self.ud = vertcat(ud{:});
            self.z = vertcat(z{:});
            self.d = vertcat(d{:});
            self.y = vertcat(y{:});
            self.w = vertcat(w{:});
            self.l = vertcat(l{:});
            self.m = vertcat(m{:});
            self.srcPorts = vertcat(srcPorts{:});
            self.snkPorts = vertcat(snkPorts{:});
        end
    end
end

