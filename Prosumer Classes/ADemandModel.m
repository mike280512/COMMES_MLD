classdef ADemandModel < Component
    
    % ADemandModel Static MLD system model to allow input of nominal energy
    % demand which may be adjusted up or down. Assumed to be a continuous
    % energy demand.
    
    properties (SetAccess = private, GetAccess = public)
        %inherits from Component class
        Nn              %number of segments per energy consumption cycle, n
        plusMax         %maximum increase in nominal demand per segment
        minusMax        %maximum decrease in nominal demand per segment
        
    end
    
    methods
        
        function self = ADemandModel(name,N,bigM,Nn,plusMax,minusMax)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.Nn = Nn;
            
            %check for scalar or vector inputs of adjustable limits
            [r,c] = size(plusMax);
            if r == 1 && c == 1
                self.plusMax = plusMax*ones(self.N,self.Nn);
            elseif c == 1
                self.plusMax = plusMax*ones(1,self.Nn);
            else
                self.plusMax = plusMax;
            end
            [r,c] = size(minusMax);
            if r == 1 && c == 1
                self.minusMax = minusMax*ones(self.N,self.Nn);
            elseif c == 1
                self.minusMax = minusMax*ones(1,self.Nn);
            else
                self.minusMax = minusMax;
            end
            
            %initialise variables
            self.y = self.get_y;
            self.uc = self.get_uc;
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
            
            varName_y_L = [self.name,'_y_L'];
            
            self.vars.(matlab.lang.makeValidName(varName_y_L)) = sdpvar(self.N,1); %define sdpvar in vars struct
            
            y = [y;varName_y_L];
            
            self.snkPorts = {varName_y_L};
        end
        
        function uc = get_uc(self)
            %initialies MLD system input variables and returns cell array
            uc = self.uc;
            
            varName_uc_l = [self.name,'_uc_l'];
            
            self.vars.(matlab.lang.makeValidName(varName_uc_l)) = sdpvar(self.N,self.Nn,'full'); %define sdpvar in vars struct
            
            uc = [uc;varName_uc_l];
        end
        
        function z = get_z(self)
            %initialies MLD system input variables and returns cell array
            z = self.z;
            
            varName_z_l = [self.name,'_z_l'];
            varName_z_plus = [self.name,'_z_plus'];
            varName_z_minus = [self.name,'_z_minus'];
            
            self.vars.(matlab.lang.makeValidName(varName_z_l)) = sdpvar(self.N,self.Nn,'full'); %define sdpvar in vars struct
            self.vars.(matlab.lang.makeValidName(varName_z_plus)) = sdpvar(self.N,self.Nn,'full');
            self.vars.(matlab.lang.makeValidName(varName_z_minus)) = sdpvar(self.N,self.Nn,'full');
            
            z = [z;varName_z_l,varName_z_plus;varName_z_minus];
        end
        
        function d = get_d(self)
            %initialies MLD system integer auxiliary variables and returns cell array
            d = self.d;
            
            varName_d_proc = [self.name,'_d_proc'];
            varName_d_comp = [self.name,'_d_comp'];
            varName_d_plus = [self.name,'_d_plus'];
            varName_d_minus = [self.name,'_d_minus'];
            
            self.vars.(matlab.lang.makeValidName(varName_d_proc)) = binvar(self.N,self.Nn,'full');
            self.vars.(matlab.lang.makeValidName(varName_d_comp)) = binvar(self.N,self.Nn,'full');
            self.vars.(matlab.lang.makeValidName(varName_d_plus)) = binvar(self.N,self.Nn,'full');
            self.vars.(matlab.lang.makeValidName(varName_d_minus)) = binvar(self.N,self.Nn,'full');
            
            d = [d;varName_d_proc;varName_d_comp;varName_d_plus;varName_d_minus];
        end
        
        function m = get_m(self)
            %initialies MLD system measured variables and returns cell array
            m= self.m;
            
            varName_m_E = [self.name,'_m_E'];
            varName_m_proc = [self.name,'_m_proc'];
            varName_m_comp = [self.name,'_m_comp'];
            
            self.vars.(matlab.lang.makeValidName(varName_m_E)) = sdpvar(1,self.Nn);
            self.vars.(matlab.lang.makeValidName(varName_m_proc)) = sdpvar(1,self.Nn);
            self.vars.(matlab.lang.makeValidName(varName_m_comp)) = sdpvar(1,self.Nn);
            
            m = [m;varName_m_E;varName_m_proc;varName_m_comp];
        end
        
        function C = get_C(self)
            C = self.C;
            %construct constraints for ADemandModel
                        
            varName_y_L = [self.name,'_y_L']; %sink port
            varName_uc_l = [self.name,'_uc_l'];
            varName_z_l = [self.name,'_z_l'];
            varName_z_plus = [self.name,'_z_plus'];
            varName_z_minus = [self.name,'_z_minus'];
            varName_d_proc = [self.name,'_d_proc'];
            varName_d_comp = [self.name,'_d_comp'];
            varName_d_plus = [self.name,'_d_plus'];
            varName_d_minus = [self.name,'_d_minus'];
            varName_m_E = [self.name,'_m_E'];
            varName_m_proc = [self.name,'_m_proc'];
            varName_m_comp = [self.name,'_m_comp'];
            
            var_y_L = self.vars.(matlab.lang.makeValidName(varName_y_L)); %get variable
            var_uc_l = self.vars.(matlab.lang.makeValidName(varName_uc_l));
            var_z_l = self.vars.(matlab.lang.makeValidName(varName_z_l));
            var_z_plus = self.vars.(matlab.lang.makeValidName(varName_z_plus));
            var_z_minus = self.vars.(matlab.lang.makeValidName(varName_z_minus));
            var_d_proc = self.vars.(matlab.lang.makeValidName(varName_d_proc));
            var_d_comp = self.vars.(matlab.lang.makeValidName(varName_d_comp));
            var_d_plus = self.vars.(matlab.lang.makeValidName(varName_d_plus));
            var_d_minus = self.vars.(matlab.lang.makeValidName(varName_d_minus));
            var_m_E = self.vars.(matlab.lang.makeValidName(varName_m_E));
            var_m_proc = self.vars.(matlab.lang.makeValidName(varName_m_proc));
            var_m_comp = self.vars.(matlab.lang.makeValidName(varName_m_comp));
            
            %MLD constraints
            C = C + [(var_y_L == var_uc_l*ones(self.Nn,1)):varName_y_L];
            C = C + [(var_uc_l == var_z_l + var_z_plus - var_z_minus):varName_uc_l];
            C = C + [(ones(1,self.N)*var_z_l == var_m_E):strcat(varName_z_l,'_schedule')];
            C = C + [(var_d_plus + var_d_minus <= ones(self.N,self.Nn)):strcat(varName_d_plus,'_',varName_d_minus,'_excl')];
            C = C + [(eps*var_d_plus <= var_z_plus <= self.plusMax.*var_d_plus):strcat(varName_z_plus,'_limits')];
            C = C + [(eps*var_d_minus <= var_z_minus <= self.minusMax.*var_d_minus):strcat(varName_z_minus,'_limits')];
            C = C + [(0 <= var_z_plus <= self.plusMax.*var_d_proc):strcat(varName_z_plus,'_processing')];
            C = C + [(0 <= var_z_minus <= self.minusMax.*var_d_proc):strcat(varName_z_minus,'_processing')];
            C = C + [(eps*var_d_proc <= var_uc_l <= self.bigM*var_d_proc):strcat(varName_uc_l,'_processing')];
            C = C + [(ones(1,self.Nn) <= ones(1,self.N)*var_d_proc <= ones(1,self.Nn)):strcat(varName_d_proc,'_limits')];
            C = C + [(var_d_proc + var_d_comp <= ones(self.N,self.Nn)):strcat(varName_d_proc,'_',varName_d_comp,'_excl')];
            C = C + [(var_m_proc - var_d_proc(1,:) <= var_d_comp(1,:)):strcat(varName_d_comp,'_0')];
            C = C + [(var_d_proc(1:self.N-1,:) - var_d_proc(2:self.N,:) <= var_d_comp(2:self.N,:)):strcat(varName_d_comp,'_1:N-1')];
            C = C + [(var_m_comp <= var_d_comp(1,:)):strcat(varName_d_comp,'_hold_0')];
            C = C + [(var_d_comp(1:self.N-1,:) <= var_d_comp(2:self.N,:)):strcat(varName_d_comp,'_hold_1:N')];
            C = C + [(var_d_proc(:,2:self.Nn) <= var_d_comp(:,1:self.Nn-1)):strcat(self.name,'_segment_sequence')];
            C = C + [(var_d_comp(:,1:self.Nn-1) == var_d_proc(:,2:self.Nn) + var_d_comp(:,2:self.Nn)):varName_d_comp];
            C = C + [(var_d_proc(1,1) == 1):strcat(varName_d_proc,'_start')];
            
        end
        
    end
    
end