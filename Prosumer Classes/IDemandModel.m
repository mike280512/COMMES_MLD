classdef IDemandModel < Component
    
    % IDemandModel Static MLD system model to allow input of nominal energy
    % demand with interruptions. Assumed to take place once within
    % a given period, e.g. once per day.
    
    properties (SetAccess = private, GetAccess = public)
        %inherits from Component class
        Nn              %number of segments per energy consumption cycle, n

    end
    
    methods
        
        function self = IDemandModel(name,N,bigM,Nn)
            
            %call parent class constructor
            self@Component(name,N)
            
            %initialise and set object properties
            self.bigM = bigM;
            self.Nn = Nn;
            
            %initialise variables
            self.y = self.get_y;
            self.uc = self.get_uc;
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
        
        function d = get_d(self)
            %initialies MLD system integer auxiliary variables and returns cell array
            d = self.d;
            
            varName_d_proc = [self.name,'_d_proc'];
            varName_d_comp = [self.name,'_d_comp'];
            varName_d_wait = [self.name,'_d_wait'];
            
            self.vars.(matlab.lang.makeValidName(varName_d_proc)) = binvar(self.N,self.Nn,'full');
            self.vars.(matlab.lang.makeValidName(varName_d_comp)) = binvar(self.N,self.Nn,'full');
            self.vars.(matlab.lang.makeValidName(varName_d_wait)) = binvar(self.N,self.Nn-1,'full');
            
            d = [d;varName_d_proc;varName_d_comp;varName_d_wait];
        end
        
        function m = get_m(self)
            %initialies MLD system measured variables and returns cell array
            m = self.m;
            
            varName_m_E = [self.name,'_m_E'];
            varName_m_proc = [self.name,'_m_proc'];
            varName_m_comp = [self.name,'_m_comp'];
            varName_m_waitMin = [self.name,'_m_waitMin'];
            varName_m_waitMax = [self.name,'_m_waitMax'];
            varName_m_pref = [self.name,'_m_pref'];
            varName_m_start = [self.name,'_m_start'];
            
            self.vars.(matlab.lang.makeValidName(varName_m_E)) = sdpvar(1,self.Nn);
            self.vars.(matlab.lang.makeValidName(varName_m_proc)) = sdpvar(1,self.Nn);
            self.vars.(matlab.lang.makeValidName(varName_m_comp)) = sdpvar(1,self.Nn);
            self.vars.(matlab.lang.makeValidName(varName_m_waitMin)) = sdpvar(1,self.Nn-1);
            self.vars.(matlab.lang.makeValidName(varName_m_waitMax)) = sdpvar(1,self.Nn-1);
            self.vars.(matlab.lang.makeValidName(varName_m_pref)) = sdpvar(self.N,1);
            self.vars.(matlab.lang.makeValidName(varName_m_start)) = sdpvar(self.N,1);
            
            m = [m;varName_m_E;varName_m_proc;varName_m_comp;...
                varName_m_waitMin;varName_m_waitMax;varName_m_pref;varName_m_start];
        end
        
        function C = get_C(self)
            C = self.C;
            %construct constraints for IDemandModel
                        
            varName_y_L = [self.name,'_y_L']; %sink port
            varName_uc_l = [self.name,'_uc_l'];
            varName_d_proc = [self.name,'_d_proc'];
            varName_d_comp = [self.name,'_d_comp'];
            varName_d_wait = [self.name,'_d_wait'];
            varName_m_E = [self.name,'_m_E'];
            varName_m_proc = [self.name,'_m_proc'];
            varName_m_comp = [self.name,'_m_comp'];
            varName_m_waitMin = [self.name,'_m_waitMin'];
            varName_m_waitMax = [self.name,'_m_waitMax'];
            varName_m_pref = [self.name,'_m_pref'];
            varName_m_start = [self.name,'_m_start'];

            var_y_L = self.vars.(matlab.lang.makeValidName(varName_y_L)); %get variable
            var_uc_l = self.vars.(matlab.lang.makeValidName(varName_uc_l));
            var_d_proc = self.vars.(matlab.lang.makeValidName(varName_d_proc));
            var_d_comp = self.vars.(matlab.lang.makeValidName(varName_d_comp));
            var_d_wait = self.vars.(matlab.lang.makeValidName(varName_d_wait));
            var_m_E = self.vars.(matlab.lang.makeValidName(varName_m_E));
            var_m_proc = self.vars.(matlab.lang.makeValidName(varName_m_proc));
            var_m_comp = self.vars.(matlab.lang.makeValidName(varName_m_comp));
            var_m_waitMin = self.vars.(matlab.lang.makeValidName(varName_m_waitMin));
            var_m_waitMax = self.vars.(matlab.lang.makeValidName(varName_m_waitMax));
            var_m_pref = self.vars.(matlab.lang.makeValidName(varName_m_pref));
            var_m_start = self.vars.(matlab.lang.makeValidName(varName_m_start));
            
            %MLD constraints
            C = C + [(var_y_L == var_uc_l*ones(self.Nn,1)):varName_y_L];
            C = C + [(ones(1,self.N)*var_uc_l == var_m_E):strcat(varName_uc_l,'_schedule')];
            C = C + [(eps*var_d_proc <= var_uc_l <= self.bigM*var_d_proc):strcat(varName_uc_l,'_processing')];
            C = C + [(zeros(1,self.Nn) <= ones(1,self.N)*var_d_proc <= ones(1,self.Nn)):strcat(varName_d_proc,'_limits')];
            C = C + [(var_d_proc + var_d_comp <= ones(self.N,self.Nn)):strcat(varName_d_proc,'_',varName_d_comp,'_excl')];
            C = C + [(var_m_proc - var_d_proc(1,:) <= var_d_comp(1,:)):strcat(varName_d_comp,'_0')];
            C = C + [(var_d_proc(1:self.N-1,:) - var_d_proc(2:self.N,:) <= var_d_comp(2:self.N,:)):strcat(varName_d_comp,'_1:N-1')];
            C = C + [(var_m_comp <= var_d_comp(1,:)):strcat(varName_d_comp,'_hold_0')];
            C = C + [(var_d_comp(1:self.N-1,:) <= var_d_comp(2:self.N,:)):strcat(varName_d_comp,'_hold_1:N-1')];
            C = C + [(var_d_proc(:,2:self.Nn) <= var_d_comp(:,1:self.Nn-1)):strcat(self.name,'_segment_sequence')];
            C = C + [(var_d_wait(:,1:self.Nn-1) == var_d_comp(:,1:self.Nn-1) - var_d_proc(:,2:self.Nn) - var_d_comp(:,2:self.Nn)):varName_d_wait];
            C = C + [(var_m_waitMin <= ones(1,self.N)*var_d_wait <= var_m_waitMax):strcat(varName_d_wait,'_limits')];
            C = C + [(var_m_pref'*var_d_proc == zeros(1,self.Nn)):strcat(varName_d_proc,'_preference')];
            C = C + [(var_m_start.*var_d_proc(:,1) == var_m_start):strcat(varName_d_proc,'_start')];
            
        end
        
    end
    
end