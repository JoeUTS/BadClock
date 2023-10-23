classdef DobotCR16 < RobotBaseClass
    %% DobotCR16 

    properties(Access = public)
         plyFileNameStem = 'DobotCR16';
    end
    
    methods
%% Define robot Function 
        function self = DobotCR16(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;
            
            self.PlotAndColourRobot();
        end

%% Create the robot model
        function CreateModel(self)
            % Create the Dobot CR16 model
            link(1) = Link('d',0.1765,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.512,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.363,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(4) = Link('d',0.125,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset',0);
            link(5) = Link('d',0.125,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(6) = Link('d',0.1084,'a',0,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);
            
            % Joint limits
            link(1).qlim = [-360 360]*pi/180;
            link(2).qlim = [-90 90]*pi/180;
            link(3).qlim = [-170 170]*pi/180;
            link(4).qlim = [-360 360]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end
