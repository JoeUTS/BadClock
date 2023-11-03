classdef SeggyDs < handle
    
    properties (Constant)
        originRotation = trotx(0, 'deg');

        % Drawing
        segmentLength = 0.01; % 1cm
        steps = 20; % THIS IS LOW FOR TESTING. MAKE ME HIGHER!!!!!
        drawingOffset = 0.055;
        deltaT = 0.05; % Time step for RMRC

        % Robots
        robot2EndEffectorOffset = 0.05;
        deliveryZone = [0.5,-0.3,0.025];

        % Poses
        robot1ReadyPose = [-90, 5, 85, 50, 0];
        robot2ReadyPose = [90, -90, 90, -90, -90, 0];
        
        % Debug
        debug = 1; % This is to display drawn lines in the simulation and post progress to the command window
        collisionDetection = 1; % turn collision detection off
    end

    properties
        % Robots
        robot1;
        robot2;
        penOffset = 0.090;
        penRaisedOffset = 0.1;
        robot1Links;
        robot2Links;

        % Box
        objectBox;

        % Drawing
        originTranslation = [0.3,0,0.03]; % Set at roughly the top of the base unit
        minutesArray = [9,9]; % initialise as impossible time for bug checking
        hoursArray = [9,9]; % initialise as impossible time for bug checking
        penRaised = 1; % Used to signal if pen is raised
        % Used for placing the time
        digit1Origin = [0.3,0,0.1];
        digit2Origin = [0.3,0,0.1];
        colonOrigin = [0.3,0,0.1];
        digit3Origin = [0.3,0,0.1];
        digit4Origin = [0.3,0,0.1];

        % Operation Recovery
        operationRunning = 0;
        digit1Status = 0;
        digit2Status = 0;
        colonStatus = 0;
        digit3Status = 0;
        digit4Status = 0;
        

        % Safety
        estopFlag = false; % Add an e-stop flag
        lightCurtainSize = [0.5, 0.01, 0.4];  % Default size
        lightCurtainPosition = [0.25, -0.5, 0];  % Default position
        lightCurtainSafe = true;

        % Collision detection
        % Faces/Vertices/FaceNormals
        robot1Faces;
        robot1Vertices;
        robot1FaceNormals;
        robot2Faces;
        robot2Vertices;
        robot2FaceNormals;
        lightCurtainVertices;
        lightCurtainFaceNormals;
    end

    methods 
		function self = SeggyDs()
            %% To do list
            % - visual servoing function to set origin
            % - make number of steps variable based on distance

			clf;
			clc;
            axis([-1 2.25 -1 1 -0.5 2]);
            view(-90,90);
            hold on;

            % Main robot (Dobot Magician)
            % NOTE: Base turned off in RobotBaseClass.InitiliseRobotPlot() function.
            %       Add 'notiles' to plot3D in function to hide.
            self.robot1 = DobotMagician(transl(0,0,0));
            %self.robot1.model.teach('noname', 'noarrow', 'notiles', 'nojoints');
            robot1ReadyPose = deg2rad(self.robot1ReadyPose);
            self.robot1.model.animate(robot1ReadyPose);

            % Secondary robot (Dobot CR16)
            self.robot2 = DobotCR16(transl(0.9,0.2,0));
            robot2ReadyPose = deg2rad(self.robot2ReadyPose);
            self.robot2.model.animate(robot2ReadyPose);

            self.enviroment();

            % Light curtain load in
            self.InitializeLightCurtain(self.lightCurtainSize, self.lightCurtainPosition);
            self.VisualizeLightCurtain();

            % Add debug message if enabled
            if self.debug == 1
                disp('initialization complete');
            end
            
            % Deliverables
            %input('Press enter to begin')
            %self.OperationRun();
            %self.ColisionSimulation();
            self.LightScreenTripSimulation();
            %self.operationRecovery();
		end
	end
    
    methods
        function MoveRobot1(self, targetCartesian, RMRC)
            if nargin < 3
                RMRC = 0;
            end
            % get current pose
            currentPose = self.robot1.model.getpos();
                        
            % Find new pose
            newPose = self.robot1.model.ikine(targetCartesian, 'q0', currentPose, 'mask', [1 1 1 0 1 1], 'forceSoln');
                        
            % Generate trajectory
            if RMRC == 0
                trajectory = jtraj(currentPose, newPose, self.steps);
            else
                %% Use RMRC for drawing
                % Set up variables
                trajectoryCartesian = nan(6,self.steps);
                s = lspb(0,1,self.steps);
                Jacobian = zeros(6);
                manipulability = zeros(self.steps, 1);
                xdot = zeros(6,1);
                trajectory = nan(self.steps,5);
                threshDLS = 1*10^-10;

                % Set first step as current pose
                trajectory(1,:) = currentPose;

                currentCartesian = self.robot1.model.fkine(currentPose);
                
                % Generate trajectory
                for i = 1:self.steps
                    % RPY not needed for this design
                    trajectoryCartesian(4:6,i) = [0,0,0]';
                    trajectoryCartesian(1:3,i) = (1-s(i))*currentCartesian.t + s(i)*targetCartesian(1:3,4);
                end

                for i = 1:self.steps-1
                    % Calculate velocity of end effector
                    xdot = (trajectoryCartesian(:,i+1) - trajectoryCartesian(:,i))/self.deltaT;

                    % Get Jacobian of current pose
                    Jacobian = self.robot1.model.jacob0(trajectory(i,:));

                    % Jacobian isnt square, pseudoinverse used instead.
                    %invJ = inv(Jacobian * Jacobian');
                    %JPlus = Jacobian' * invJ;

                    % Calculate joint velocities
                    %qdot = JPlus * xdot;

                    %% Damped Least Squares
                    manipulability(i) = sqrt(det(Jacobian*Jacobian'));
                    
                    if manipulability(i) < threshDLS
                        lambda = (1 - (manipulability(i)/threshDLS)^2)*10^-5;
                    else
                        lambda = 0;
                    end

                    % Using pseudoinverse as Jacobian isnt square
                    invJ = Jacobian' * inv((Jacobian*Jacobian') + (lambda * eye(6)));
                    qdot = invJ * xdot;

                    % Update qMatrix
                    trajectory(i+1,:) =  trajectory(i,:) + self.deltaT * qdot';
                end
            end

            % Animate
            for i = 1:self.steps
                % Safety
                self.SafetyCheck();

                % Animate
                animateStep = trajectory(i,:);
                self.robot1.model.animate(animateStep);
                drawnow();
            end
        end

        function MoveRobot2(self, targetCartesian, trapezoidal)
            if nargin < 3
                trapezoidal = 0;
            end
            % get current pose
            currentPose = self.robot2.model.getpos();
                        
            % Find new pose
            newPose = self.robot2.model.ikine(targetCartesian, 'q0', currentPose, 'forceSoln');
                        
            % Generate trajectory
            if trapezoidal == 0
                trajectory = jtraj(currentPose, newPose, self.steps);
            else
                % Nil RMRC required for this arm. Trapezoidal will work.
                trajectory = nan(self.steps,5);
                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end
            end

            % Animate
            for i = 1:self.steps
                % Safety
                self.SafetyCheck();

                % Animate
                animateStep = trajectory(i,:);
                self.robot2.model.animate(animateStep);
                drawnow();
            end
        end

        function MoveToOrigin(self)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('moving to origin');
            end

            self.MoveRobot1(transl(self.originTranslation + [0,0,self.penRaisedOffset]) * self.originRotation);       
        end

        function RaisePen(self)
            % If pen already raised then ignore
            if self.penRaised == 1
                % Add debug message if enabled
                if self.debug == 1
                    disp('Pen already in lifted position');
                    input('press enter to continue');
                end
                
                return
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Lifting Pen');
            end

            % Get current position
            currentPose = self.robot1.model.getpos();
            currentCartesian = self.robot1.model.fkine(currentPose);

            self.MoveRobot1(transl([currentCartesian.t(1), currentCartesian.t(2), (self.originTranslation(3) + self.penRaisedOffset)])* self.originRotation);

            self.penRaised = 1;

        end

        function LowerPen(self)
            % If pen already raised then ignore
            if self.penRaised == 0
                % Add debug message if enabled
                if self.debug == 1
                    disp('Pen already in lowered position');
                    input('press enter to continue');
                end
                
                return
            end

            % Get current position
            currentPose = self.robot1.model.getpos();
            currentCartesian = self.robot1.model.fkine(currentPose);

            self.MoveRobot1(transl([currentCartesian.t(1), currentCartesian.t(2), (self.originTranslation(3) + self.penOffset)])* self.originRotation);

            self.penRaised = 0;

        end

        function JogX(self, distance)
            % Get current pose
            currentPose = self.robot1.model.getpos();
            currentCartesian = self.robot1.model.fkine(currentPose);

            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('Jogging in X axis. Distance(m): %d', distance);
                disp(message);
            end

            % Uses RMRC to keep drawing speed mostly constant
            self.MoveRobot1(transl(currentCartesian.t' + [distance, 0, 0])* self.originRotation, 1);

            % Used to plot drawn lines within simulation
            if (self.debug == 1) && (self.penRaised == 0)
                endPoint = self.robot1.model.fkine(self.robot1.model.getpos());
                plot3([currentCartesian.t(1),endPoint.t(1)], [currentCartesian.t(2), endPoint.t(2)], [currentCartesian.t(3) - self.drawingOffset, endPoint.t(3) - self.drawingOffset]);
            end
        end

        function JogY(self, distance)
            % Get current pose
            currentPose = self.robot1.model.getpos();
            currentCartesian = self.robot1.model.fkine(currentPose);  % is this needed?

            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('Jogging in Y axis. Distance(m): %d', distance);
                disp(message);
            end

            % Uses RMRC to keep drawing speed mostly constant
            self.MoveRobot1(transl(currentCartesian.t' + [0, distance, 0]) * self.originRotation, 1);

            % Used to plot drawn lines within simulation
            if (self.debug == 1) && (self.penRaised == 0)
                endPoint = self.robot1.model.fkine(self.robot1.model.getpos());
                plot3([currentCartesian.t(1),endPoint.t(1)], [currentCartesian.t(2), endPoint.t(2)], [currentCartesian.t(3) - self.drawingOffset, endPoint.t(3) - self.drawingOffset]);
            end
        end
        
        function Number0(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end
            
            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);
            
            % Lower pen
            self.LowerPen();
            
            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 0');
            end

            % Draw digit
            self.JogY(-self.segmentLength);
            self.JogX(-2*self.segmentLength);
            self.JogY(self.segmentLength);
            self.JogX(2*self.segmentLength);
            
            % Raise pen
            self.RaisePen();
        end

        function Number1(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);
            
            % Move one segment to the right and lower pen
            self.JogY(-self.segmentLength)
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 1');
            end

            % Draw digit
            self.JogX(-2*self.segmentLength);

            % Raise pen
            self.RaisePen();
        end

        function Number2(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);

            % Lower pen
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 2');
            end

            % Draw digit
            self.JogY(-self.segmentLength);
            self.JogX(-self.segmentLength);
            self.JogY(self.segmentLength);
            self.JogX(-self.segmentLength);
            self.JogY(-self.segmentLength);

            % Raise pen
            self.RaisePen();
        end

        function Number3(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);

            % Lower pen
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 3');
            end

            % Draw digit
            self.JogY(-self.segmentLength);
            self.JogX(-2*self.segmentLength);
            self.JogY(self.segmentLength);
            self.RaisePen();
            self.JogX(self.segmentLength);
            self.LowerPen();
            self.JogY(-self.segmentLength);

            % Raise pen
            self.RaisePen();
        end

        function Number4(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);

            % Lower pen
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 4');
            end

            % Draw digit
            self.JogX(-self.segmentLength);
            self.JogY(-self.segmentLength);
            self.RaisePen();
            self.JogX(self.segmentLength);
            self.LowerPen();
            self.JogX(-2*self.segmentLength);

            % Raise pen
            self.RaisePen();
        end

        function Number5(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);

            % Move one segment to the right then lower pen
            self.JogY(-self.segmentLength)
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 5');
            end

            % Draw digit
            self.JogY(self.segmentLength);
            self.JogX(-self.segmentLength);
            self.JogY(-self.segmentLength);
            self.JogX(-self.segmentLength);
            self.JogY(self.segmentLength);

            % Raise pen
            self.RaisePen();
        end

        function Number6(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);

            % Move one segment to the right then lower pen
            self.JogY(-self.segmentLength)
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 6');
            end

            % Draw digit
            self.JogX(-2*self.segmentLength);
            self.JogY(self.segmentLength);
            self.JogX(self.segmentLength);
            self.JogY(-self.segmentLength);

            % Raise pen
            self.RaisePen();
        end

        function Number7(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);

            % Lower pen
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 7');
            end

            % Draw digit
            self.JogY(-self.segmentLength);
            self.JogX(-2*self.segmentLength);

            % Raise pen
            self.RaisePen();
        end

        function Number8(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);

            % Lower pen
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 8');
            end

            % Draw digit
            self.JogY(-self.segmentLength);
            self.JogX(-2*self.segmentLength);
            self.JogY(self.segmentLength);
            self.JogX(2*self.segmentLength);
            self.RaisePen();
            self.JogX(-self.segmentLength);
            self.LowerPen();
            self.JogY(-self.segmentLength);

            % Raise pen
            self.RaisePen();
        end

        function Number9(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);

            % Move right and down then Lower pen
            self.JogY(-self.segmentLength);
            self.JogX(-self.segmentLength);
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 9');
            end

            % Draw digit
            self.JogY(self.segmentLength);
            self.JogX(self.segmentLength);
            self.JogY(-self.segmentLength);
            self.JogX(-2*self.segmentLength);

            % Raise pen
            self.RaisePen();
        end

        function DrawColon(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Add debug message if enabled
            if self.debug == 1
                disp('Moving to digit origin');
            end

            % Move to digit Origin
            self.MoveRobot1(transl(digitOrigin + [0, 0, self.penRaisedOffset]) * self.originRotation);
            

            % Move right and down then Lower pen
            self.JogY(-self.segmentLength/3);
            self.JogX(-self.segmentLength/3);
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing colon');
            end

            % Draw digit
            self.JogY(-self.segmentLength/3);
            self.JogX(-self.segmentLength/3);
            self.JogY(self.segmentLength/3);
            self.JogX(self.segmentLength/3);
            self.RaisePen();
            self.JogX(-self.segmentLength/2);
            self.LowerPen();
            self.JogY(-self.segmentLength/3);
            self.JogX(-self.segmentLength/3);
            self.JogY(self.segmentLength/3);
            self.JogX(self.segmentLength/3);

            % Raise pen
            self.RaisePen();
        end

        function UpdateTime(self)
            % get current time
            [hour,minute] = hms(datetime('now','Format','HH:mm'));

            %Convert hour and minute values to individual digits
            hourDigits=str2double(regexp(num2str(hour),'\d','match'));
            minuteDigits=str2double(regexp(num2str(minute),'\d','match'));

            % Add a 0 to the start of the hours array if only 1 digit
            hourArraySize = size(hourDigits);
            minuteArraySize = size(minuteDigits);

            if hourArraySize < 2
                hourIntermediary = [0,hourDigits];
            else
                hourIntermediary = hourDigits;
            end

            if minuteArraySize < 2
                minuteIntermediary = [0,minuteDigits];
            else
                minuteIntermediary = minuteDigits;
            end
            
            % Update global variables
            self.hoursArray = hourIntermediary;
            self.minutesArray = minuteIntermediary;
        end

        function WriteNumber(self, number, location)
            % Check what the number is and write it
            if number == 0
                self.Number0(location);
            elseif number == 1
                self.Number1(location);
            elseif number == 2
                self.Number2(location);
            elseif number == 3
                self.Number3(location);
            elseif number == 4
                self.Number4(location);
            elseif number == 5
                self.Number5(location);
            elseif number == 6
                self.Number6(location);
            elseif number == 7
                self.Number7(location);
            elseif number == 8
                self.Number8(location);
            elseif number == 9
                self.Number9(location);
            else
                % add something to log for being out of range
            end
        end

        function WriteTime(self, timeOrigin)
            % Set digit origins
            self.digit1Origin = timeOrigin;
            self.digit2Origin = timeOrigin - [0, 2*(self.segmentLength + (0.5 * self.segmentLength)), 0];
            self.colonOrigin = timeOrigin - [0, 3*(self.segmentLength + (0.5 * self.segmentLength)), 0];
            self.digit3Origin = timeOrigin - [0, 4*(self.segmentLength + (0.5 * self.segmentLength)), 0];
            self.digit4Origin = timeOrigin - [0, 5*(self.segmentLength + (0.5 * self.segmentLength)), 0];

            % Get new time, move to origin and write time
            self.UpdateTime();
            self.MoveToOrigin();
            self.WriteNumber(self.hoursArray(1), self.digit1Origin);
            self.digit1Status = 1;
            self.WriteNumber(self.hoursArray(2), self.digit2Origin);
            self.digit2Status = 1;
            self.DrawColon(self.colonOrigin);
            self.colonStatus = 1;
            self.WriteNumber(self.minutesArray(1), self.digit3Origin);
            self.digit3Status = 1;
            self.WriteNumber(self.minutesArray(2), self.digit4Origin);
            self.digit4Status = 1;

            % Clear flag to signal operation complete
            self.operationRunning = 0;
        end

        function operationRecovery(self)
            % make the program continue after E-Stop
            % Check if program running and cease function if not
            if self.operationRunning == 0
                return
            end

            self.MoveToReady();

            % write each digit sequencially which lacks a completion flag
            self.MoveToOrigin();

            if self.digit1Status == 0
                % resume digit 1
                self.WriteNumber(self.hoursArray(1), self.digit1Origin);
                self.digit1Status = 1;
            end
                
            if self.digit2Status == 0
                % resume digit 2
                self.WriteNumber(self.hoursArray(2), self.digit2Origin);
                self.digit2Status = 1;
            end
                
            if self.colonStatus == 0
                % resume colon
                self.DrawColon(self.colonOrigin);
                self.colonStatus = 1;
            end

            if self.digit3Status == 0
                % resume digit 3
                self.WriteNumber(self.minutesArray(1), self.digit3Origin);
                self.digit3Status = 1;
            end

            if self.digit4Status == 0
                % resume form digit 4
                self.WriteNumber(self.minutesArray(2), self.digit4Origin);
                self.digit4Status = 1;
            end
            self.operationRunning = 0;

            self.MoveToReady();
            self.DeliverBox();
        end

        function enviroment(self)
            %% Load in and position objects
            % Box (This reporesents the box to be writen on)
            self.objectBox = PlaceObject('brick.ply',[0,0,0]);
            verts = [get(self.objectBox,'Vertices'), ones(size(get(self.objectBox,'Vertices'),1),1)] * trotz(pi/2) * trotx(pi);
            verts(:,:) = verts(:,:);
            set(self.objectBox,'Vertices',verts(:,1:3));
            TransformMesh(self.objectBox, transl([0.5,-0.3,0.025]));

            % Table
            objectTable = PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0.75,0,-0.5]);

            % Printer (Represents PC)
            objectPrinter = PlaceObject('printer.ply',[1.5,-0.5,0]);

            % Chair
            objectchair = PlaceObject('chair.ply',[0,0,0]);
            verts = [get(objectchair,'Vertices'), ones(size(get(objectchair,'Vertices'),1),1)] * trotx(-pi/2) * trotz(pi);
            verts(:,:) = verts(:,:);
            set(objectchair,'Vertices',verts(:,1:3));
            TransformMesh(objectchair, transl([1.5,-0.75, -0.35]));

            % E-Stop
            objecEmergencyButton = PlaceObject('emergencyStopButton.ply',[0,0,0]);
            verts = [get(objecEmergencyButton,'Vertices'), ones(size(get(objecEmergencyButton,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,:) = verts(:,:) * 0.25;
            set(objecEmergencyButton,'Vertices',verts(:,1:3));
            TransformMesh(objecEmergencyButton, transl([1.75,-0.6,0.05]));

            % Fire Extinguisher
            objecFireExtinguisher = PlaceObject('fireExtinguisher.ply',[2,-0.5,-0.5]);

            % Barriers
            objectbarrier1 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            verts = [get(objectbarrier1,'Vertices'), ones(size(get(objectbarrier1,'Vertices'),1),1)] * trotz(-pi/2);
            verts(:,:) = verts(:,:) / 3;
            set(objectbarrier1,'Vertices',verts(:,1:3));
            TransformMesh(objectbarrier1, transl([-0.25,-0.25,0.15]));

            objectbarrier2 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            verts = [get(objectbarrier2,'Vertices'), ones(size(get(objectbarrier2,'Vertices'),1),1)] * trotz(-pi/2);
            verts(:,:) = verts(:,:) / 3;
            set(objectbarrier2,'Vertices',verts(:,1:3));
            TransformMesh(objectbarrier2, transl([-0.25,0.25,0.15]));

            objectbarrier3 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            verts = [get(objectbarrier3,'Vertices'), ones(size(get(objectbarrier3,'Vertices'),1),1)];
            verts(:,:) = verts(:,:) / 3;
            set(objectbarrier3,'Vertices',verts(:,1:3));
            TransformMesh(objectbarrier3, transl([0,0.5,0.15]));

            objectbarrier4 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            verts = [get(objectbarrier4,'Vertices'), ones(size(get(objectbarrier4,'Vertices'),1),1)];
            verts(:,:) = verts(:,:) / 3;
            set(objectbarrier4,'Vertices',verts(:,1:3));
            TransformMesh(objectbarrier4, transl([0.5,0.5,0.15]));

            objectbarrier5 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            verts = [get(objectbarrier5,'Vertices'), ones(size(get(objectbarrier5,'Vertices'),1),1)];
            verts(:,:) = verts(:,:) / 3;
            set(objectbarrier5,'Vertices',verts(:,1:3));
            TransformMesh(objectbarrier5, transl([1,0.5,0.15]));

            objectbarrier7 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            verts = [get(objectbarrier7,'Vertices'), ones(size(get(objectbarrier7,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,:) = verts(:,:) / 3;
            set(objectbarrier7,'Vertices',verts(:,1:3));
            TransformMesh(objectbarrier7, transl([1.25,-0.25,0.15]));

            objectbarrier8 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            verts = [get(objectbarrier8,'Vertices'), ones(size(get(objectbarrier8,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,:) = verts(:,:) / 3;
            set(objectbarrier8,'Vertices',verts(:,1:3));
            TransformMesh(objectbarrier8, transl([1.25,0.25,0.15]));

            objectbarrier9 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            verts = [get(objectbarrier9,'Vertices'), ones(size(get(objectbarrier9,'Vertices'),1),1)] * trotz(pi);
            verts(:,:) = verts(:,:) / 3;
            set(objectbarrier9,'Vertices',verts(:,1:3));
            TransformMesh(objectbarrier9, transl([0,-0.5,0.15]));

            objectbarrier10 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            verts = [get(objectbarrier10,'Vertices'), ones(size(get(objectbarrier10,'Vertices'),1),1)] * trotz(pi);
            verts(:,:) = verts(:,:) / 3;
            set(objectbarrier10,'Vertices',verts(:,1:3));
            TransformMesh(objectbarrier10, transl([1,-0.5,0.15]));
        end

        function MoveToReady(self)
            % Move to ready position
            % get current pose
            currentPose1 = self.robot1.model.getpos();
            currentPose2 = self.robot2.model.getpos();
            
            % Dont move arm if already in position
            if currentPose1 == self.robot1ReadyPose
                moveRobot1 = 0;
            else
                moveRobot1 = 1;
            end
            
            if currentPose2 == self.robot2ReadyPose
                moveRobot2 = 0;
            else
                moveRobot2 = 1;
            end
                        
            % Generate trajectory
            if moveRobot1 == 1
                trajectory1 = jtraj(currentPose1, deg2rad(self.robot1ReadyPose), self.steps);
            end

            if moveRobot2 == 1
                trajectory2 = jtraj(currentPose2, deg2rad(self.robot2ReadyPose), self.steps);
            end
            
            % Add debug message if enabled
            if self.debug == 1
                disp('moving to ready position');
            end

            % Animate
            for i = 1:self.steps
                % Safety
                self.SafetyCheck();

                % Animate
                if moveRobot1 == 1
                    animateStep1 = trajectory1(i,:);
                    self.robot1.model.animate(animateStep1);
                end

                if moveRobot2 == 1
                    animateStep2 = trajectory2(i,:);
                    self.robot2.model.animate(animateStep2);
                end

                drawnow();
            end
        end

        function DeliverBox(self, boxLocation)
            %% Move arm 2 to box
            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('Moving to box');
                disp(message);
            end

            self.MoveRobot2(transl([boxLocation(1),boxLocation(2),boxLocation(3) + self.robot2EndEffectorOffset]) * troty(-180, 'deg'));

            %% Move box to delivery zone
            % Get current pose
            currentPose2 = self.robot2.model.getpos();

            % Move to delivery zone
            newCartesian2 = transl([self.deliveryZone(1),self.deliveryZone(2),self.deliveryZone(3) + self.robot2EndEffectorOffset]) * troty(-180, 'deg');
            newPose2 = self.robot2.model.ikine(newCartesian2, 'q0', currentPose2, 'forceSoln');
            
            % Generate trajectory
            trajectory2 = jtraj(currentPose2, newPose2, self.steps);
            
            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('Moving to box');
                disp(message);
            end

            % Animate
            for i = 1:size(trajectory2)
                % Safety
                self.SafetyCheck();

                % Animate
                animateStep = trajectory2(i,:);
                TransformMesh(self.objectBox, transl(self.robot2.model.fkine(animateStep).t'-[0,0,self.robot2EndEffectorOffset]));
                self.robot2.model.animate(animateStep);
                drawnow();
            end

            self.MoveToReady();
        end

        function GetBox(self, boxLocation)
            %% Move arm 2 to box
            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('Moving to box');
                disp(message);
            end

            self.MoveRobot2(transl([boxLocation(1),boxLocation(2),boxLocation(3) + self.robot2EndEffectorOffset]) * troty(-180, 'deg'));


            %% Move box to origin
            % Get current pose
            currentPose2 = self.robot2.model.getpos();

            % Move to origin
            newCartesian2 = transl([self.originTranslation(1),self.originTranslation(2),boxLocation(3) + self.robot2EndEffectorOffset]) * troty(-180, 'deg');
            newPose2 = self.robot2.model.ikine(newCartesian2, 'q0', currentPose2, 'forceSoln');
            
            % Generate trajectory
            trajectory2 = jtraj(currentPose2, newPose2, self.steps);
            
            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('moving to origin');
                disp(message);
            end

            % Animate
            for i = 1:size(trajectory2)
                % Safety
                self.SafetyCheck();

                % Animate
                animateStep = trajectory2(i,:);
                self.robot2.model.animate(animateStep);
                TransformMesh(self.objectBox, transl(self.robot2.model.fkine(animateStep).t'-[0,0,self.robot2EndEffectorOffset]));
                drawnow();
            end

            self.MoveToReady();
        end

        function OperationRun(self)
            % Set recovery flags
            self.operationRunning = 1;
            self.digit1Status = 0;
            self.digit2Status = 0;
            self.digit3Status = 0;
            self.digit4Status = 0;
            self.colonStatus = 0;

            self.MoveToReady();
            
            % Visual Servoing to find box
            boxCartesian = self.BoxPosition();
            
            % Move box to origin point
            self.GetBox(boxCartesian);
            self.MoveToReady();

            % Visual servoing to set new origin
            boxCartesian = self.OriginPosition();
            self.originTranslation = boxCartesian;

            self.MoveToOrigin();
            self.WriteTime(self.originTranslation + [0, 3*(self.segmentLength + (0.5 * self.segmentLength)), 0]);
            self.MoveToReady();
            self.DeliverBox(boxCartesian);
        end

        function ColisionSimulation(self)
            % Set recovery flags
            self.operationRunning = 1;
            self.digit1Status = 0;
            self.digit2Status = 0;
            self.digit3Status = 0;
            self.digit4Status = 0;
            self.colonStatus = 0;

            self.MoveToReady();
            
            %% Position arm 2 for colision
            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('moving');
                disp(message);
            end

            self.MoveRobot2(transl([self.originTranslation(1), self.originTranslation(2), self.originTranslation(3) + self.robot2EndEffectorOffset]) * troty(-180, 'deg'));

            self.WriteTime(self.originTranslation + [0, 3*(self.segmentLength + (0.5 * self.segmentLength)), 0]);
            self.MoveToReady();
        end

        function LightScreenTripSimulation(self)

            self.MoveToReady();

            % Get current pose
            currentPose1 = self.robot1.model.getpos();
            currentPose2 = self.robot2.model.getpos();

            % Move to lightscreen
            newCartesian2 = transl([0.5,-0.5,0.2]) * troty(-180, 'deg');
            newPose2 = self.robot2.model.ikine(newCartesian2, 'q0', currentPose2, 'forceSoln');
            
            % Generate trajectory
            trajectory1 = jtraj(currentPose1, deg2rad([135, 5, 85, 50, 0]), self.steps);
            trajectory2 = jtraj(currentPose2, newPose2, self.steps);
            
            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('moving');
                disp(message);
            end

            % Animate
            for i = 1:self.steps
                % Safety
                self.SafetyCheck();

                % Animate
                animateStep1 = trajectory1(i,:);
                self.robot1.model.animate(animateStep1);
                animateStep2 = trajectory2(i,:);
                self.robot2.model.animate(animateStep2);
                drawnow();
            end

            self.MoveToReady();
        end

        function location = BoxPosition(self)
            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('Finding Box');
                disp(message);
            end

            % INSERT CODE

            location = self.deliveryZone; % placeholder for visual servoing
        end

        function location = OriginPosition(self)
            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('Finding Origin');
                disp(message);
            end

            % INSERT CODE

            location = self.originTranslation; % placeholder for visual servoing
        end

        function SafetyCheck(self)
            if self.collisionDetection == 1
                % Check if there's an obstacle detected in the light curtain area
                self.checkLightCurtainInterception()
            end

            % Check for EStop and cease operation if engaged
            if self.estopFlag
                if self.debug == 1
                    disp('ERROR: E-STOP ENGAGED');
                end
                return
            end
        end
        
        function engageEStop(obj)
            if ~obj.estopFlag
                obj.estopFlag = true;
                disp('Emergency Stop Engaged');
            else
                disp('Emergency Stop is already engaged');
            end
        end

        function disengageEStop(obj)
            if obj.estopFlag
                obj.estopFlag = false;
                disp('Emergency Stop Disengaged');
            else
                disp('Emergency Stop is not engaged');
            end
        end

        function canResume = canResumeOperations(obj)
            canResume = ~obj.estopFlag;
            self.operationRecovery();
        end

        function read_ply_data(self)
            % Robot 1 PLY file names
            robot1_ply_files = {
                'DobotMagiciansLink0.ply',
                'DobotMagiciansLink1.ply',
                'DobotMagiciansLink2.ply',
                'DobotMagiciansLink3.ply',
                'DobotMagiciansLink4.ply',
                'DobotMagiciansLink5.ply'
                };

            % Robot 2 PLY file names
            robot2_ply_files = {
                'DobotCR16sLink0.ply',
                'DobotCR16sLink1.ply',
                'DobotCR16sLink2.ply',
                'DobotCR16sLink3.ply',
                'DobotCR16sLink4.ply',
                'DobotCR16sLink5.ply',
                'DobotCR16sLink6.ply'
                };

            % Read and process PLY files for Robot 1
            robot1_data = struct;
            for link = 1:numel(robot1_ply_files)
                file_name = robot1_ply_files{link};
                [self.robot1Vertices, self.robot1Faces] = self.read_ply_file(file_name);
                link_name = sprintf('DobotMagiciansLink%d', link - 1);
                robot1_data.(link_name).vertices = self.robot1Vertices;
                robot1_data.(link_name).faces = self.robot1Faces;
            end

            % Read and process PLY files for Robot 2
            robot2_data = struct;
            for link = 1:numel(robot2_ply_files)
                file_name = robot2_ply_files{link};
                [self.robot2Vertices, self.robot2Faces] = self.read_ply_file(file_name);
                link_name = sprintf('DobotCR16sLink%d', link - 1);
                robot2_data.(link_name).vertices = self.robot2Vertices;
                robot2_data.(link_name).faces = self.robot2Faces;
            end


            % % Display the data for Robot 1
            % fprintf('Robot 1 Data:\n');
            % self.display_robot_data(robot1_data);
            %
            % % Display the data for Robot 2
            % fprintf('Robot 2 Data:\n');
            % self.display_robot_data(robot2_data);
        end

        function display_robot_data(~, robot_data)
            field_names = fieldnames(robot_data);
            for link = 1:numel(field_names)
                link_name = field_names{link};
                if isfield(robot_data, link_name)
                    vertices = robot_data.(link_name).vertices;
                    faces = robot_data.(link_name).faces;
                    disp(['Link Name: ' link_name]);
                    disp('Vertices:');
                    disp(vertices);
                    disp('Faces:');
                    disp(faces);
                end
            end
        end

        function [vertices, faces] = read_ply_file(~, file_name)
            try
                ply_data = plyread(file_name);
                vertices = cell2mat(num2cell([ply_data.vertex.x, ply_data.vertex.y, ply_data.vertex.z], 2));
                faces = cell2mat(ply_data.face.vertex_indices);
            catch
                error(['Error reading ' file_name]);
                vertices = {};
                faces = {};
            end
        end

        function calculateFaceNormals(self)
            self.robot1FaceNormals = self.calculateFaceNormalsForRobot(self.robot1Vertices, self.robot1Faces);
            self.robot2FaceNormals = self.calculateFaceNormalsForRobot(self.robot2Vertices, self.robot2Faces);
        end
        
        function faceNormals = calculateFaceNormalsForRobot(~, vertices, faces)
            numFaces = size(faces, 1);
            faceNormals = zeros(numFaces, 3); % Initialize a matrix to store the face normals
    
            for i = 1:numFaces
                face = faces(i);
                if length(face) == 3
                    % Calculate face normal using cross product
                    v1 = vertices(face(2), :) - vertices(face(1), :);
                    v2 = vertices(face(3), :) - vertices(face(1), :);
                    normal = cross(v1, v2);
                    normal = normal / norm(normal);
                    faceNormals(i, :) = normal;  % Store the face normal in the matrix
                else
                    faceNormals(i, :) = zeros(1, 3);  % Handle the case when face is not a triangle
                end
            end
        end

        function InitializeLightCurtain(self, size, position)

            % Initialize the light curtain with custom parameters
            self.lightCurtainPosition = position;
            self.lightCurtainSize = size;
    
            % Set the light curtain to the initial safe state
            self.lightCurtainSafe = true;
        end

        function InitializeLightCurtainVertices(self)
            % Calculate the vertices of the light curtain bounding box
            halfWidth = self.lightCurtainSize(1) / 2;
            halfLength = self.lightCurtainSize(2) / 2;
            halfHeight = self.lightCurtainSize(3) / 2;
        
            self.lightCurtainVertices = [
                self.lightCurtainPosition + [-halfWidth, -halfLength, -halfHeight];
                self.lightCurtainPosition + [halfWidth, -halfLength, -halfHeight];
                self.lightCurtainPosition + [halfWidth, halfLength, -halfHeight];
                self.lightCurtainPosition + [-halfWidth, halfLength, -halfHeight];
                self.lightCurtainPosition + [-halfWidth, -halfLength, halfHeight];
                self.lightCurtainPosition + [halfWidth, -halfLength, halfHeight];
                self.lightCurtainPosition + [halfWidth, halfLength, halfHeight];
                self.lightCurtainPosition + [-halfWidth, halfLength, halfHeight];
            ];
        end

        function UpdateLightCurtainStatus(self, obstacleDetected)
            % Check if the provided 'obstacleDetected' parameter is valid
            if islogical(obstacleDetected)
                if obstacleDetected
                    % An obstacle has been detected in the light curtain area
                    % Call collision detection function
                    % isCollision = CheckLightCurtainCollision(self.robot1.model, self.robot2.model, self.lightCurtainSize, self.lightCurtainPosition, point);
                    isCollision = IsCollision(self.robot1, self.robot2, self.robot1.model.getpos(), self.robot2.model.getpos(), self.faces, self.vertex, self.faceNormals);

                    if isCollision
                        % Collision detected in the light curtain
                        % Engage the emergency stop
                        self.engageEStop();
                        self.lightCurtainSafe = false;
                        disp('Collision detected in the light curtain area. Emergency Stop Engaged.');
                    else
                        % No collision detected
                        self.lightCurtainSafe = true;
                        disp('Obstacle detected in the light curtain area. Light curtain is safe.');
                    end
                else
                    % No obstacle detected.
                    self.lightCurtainSafe = true;
                    disp('No obstacle detected in the light curtain area. Light curtain is safe.');
                end
            else
                % Invalid parameter, display an error message
                error('Invalid parameter for obstacle detection.');
            end
        end

        function VisualizeLightCurtain(self)

            x = [self.lightCurtainPosition(1), self.lightCurtainPosition(1) + self.lightCurtainSize(1);
                self.lightCurtainPosition(1), self.lightCurtainPosition(1) + self.lightCurtainSize(1)];
            y = [self.lightCurtainPosition(2), self.lightCurtainPosition(2);
                self.lightCurtainPosition(2) + self.lightCurtainSize(2), self.lightCurtainPosition(2) + self.lightCurtainSize(2)];
            z = [self.lightCurtainPosition(3), self.lightCurtainPosition(3) ;
                self.lightCurtainPosition(3) + self.lightCurtainSize(3), self.lightCurtainPosition(3) + self.lightCurtainSize(3)];

            % Create the filled plane using surf
            surf(x, y, z, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            % Create the bounding box using patch
            patch(x, y, z, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'r', 'FaceColor', 'none');
         
            % Show the plot
            drawnow;
        end

        function CalculateLightCurtainFaceNormals(self)
            % Initialize an array to store the face normals
            self.lightCurtainFaceNormals = zeros(6, 3);       
        
            % Define the order of vertices that form the faces
            faceVertexOrder = [1, 2, 3, 4; 5, 6, 7, 8; 1, 2, 6, 5; 2, 3, 7, 6; 3, 4, 8, 7; 4, 1, 5, 8];
        
            % Calculate face normals for each face
            for i = 1:6
                % Get the vertices that form the current face
                currentFaceVertices = self.lightCurtainVertices(faceVertexOrder(i, :), :);
        
                % Calculate the face normal using the cross product of the vectors
                faceNormal = cross(currentFaceVertices(2, :) - currentFaceVertices(1, :), currentFaceVertices(4, :) - currentFaceVertices(1, :));
                faceNormal = faceNormal / norm(faceNormal);  % Normalize the face normal
        
                % Store the calculated face normal
                self.lightCurtainFaceNormals(i, :) = faceNormal;
            end
        end
        
        function checkLightCurtainInterception(self)
            % Get the current poses of both robots
            currentPose1 = self.robot1.model.getpos();
            currentPose2 = self.robot2.model.getpos();
        
            % Iterate over the faces of the light curtain to check interceptions
            for i = 1:size(self.lightCurtainFaceNormals, 1)
                faceNormal = self.lightCurtainFaceNormals(i, :);
        
                % Calculate the position of a point on the face
                facePosition = self.lightCurtainVertices(i, :);
        
                % Check if the point on the face is in front of the face (dot product > 0)
                % for both robots
                if dot(facePosition - currentPose1(1:3), faceNormal) > 0 || ...
                   dot(facePosition - currentPose2(1:3), faceNormal) > 0
                    % Engage the emergency stop
                    self.engageEStop();
                    return; % Exit the loop as soon as interception is detected
                end
            end
        end


        function result = IsCollisionBetweenRobots(self, qMatrix1, qMatrix2, returnOnceFound)
            if nargin < 3
                returnOnceFound = true;
            end
            result = false;
        
            for qIndex = 1:size(qMatrix1, 1)
                % Get the transform of every joint for both robots at this configuration
                tr1 = self.GetLinkPoses(qMatrix1(qIndex, :), self.robot1.model);
                tr2 = self.GetLinkPoses(qMatrix2(qIndex, :), self.robot2.model);
        
                % Check for collisions between each robot and the environment obstacles
                for i = 1:size(tr1, 3) - 1
                    for j = 1:size(tr2, 3) - 1
                        for faceIndex1 = 1:size(self.robot1Faces, 1)
                            for faceIndex2 = 1:size(self.robot2Faces,1)

                                % Display the values of faceIndex1 and faceIndex2
                                disp(['faceIndex1 = ' num2str(faceIndex1) ', faceIndex2 = ' num2str(faceIndex2)]);

                                vertOnPlane1 = self.robot1Vertices(self.robot1Faces(faceIndex1,1)',:);
                                vertOnPlane2 = self.robot2Vertices(self.robot2Faces(faceIndex2,1)',:);
                                
                                % Check for collisions with robot1
                                [intersectP1, check1] = LinePlaneIntersection(self.robot1FaceNormals(faceIndex1, :), vertOnPlane1, tr1(1:3, 4, i)', tr1(1:3, 4, i + 1)');
                                if check1 == 1 && IsIntersectionPointInsideTriangle(intersectP1, vertOnPlane1)
                                    result = true;
                                    if returnOnceFound
                                        return;
                                    end
                                end
        
                                % Check for collisions with robot2
                                [intersectP2, check2] = LinePlaneIntersection(self.robot2FaceNormals(faceIndex2, :), vertOnPlane2, tr2(1:3, 4, j)', tr2(1:3, 4, j + 1)');
                                if check2 == 1 && IsIntersectionPointInsideTriangle(intersectP2, vertOnPlane2)
                                    result = true;
                                    if returnOnceFound
                                        return;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end

        function isInside = IsIntersectionPointInsideTriangle(intersectP, triangleVertices)
            % Calculate barycentric coordinates of the intersection point
            u = ((triangleVertices(2,2) - triangleVertices(3,2)) * (intersectP(1) - triangleVertices(3,1)) + (triangleVertices(3,1) - triangleVertices(2,1)) * (intersectP(2) - triangleVertices(3,2))) / ...
                ((triangleVertices(2,2) - triangleVertices(3,2)) * (triangleVertices(1,1) - triangleVertices(3,1)) + (triangleVertices(3,1) - triangleVertices(2,1)) * (triangleVertices(1,2) - triangleVertices(3,2)));
            v = ((triangleVertices(3,2) - triangleVertices(1,2)) * (intersectP(1) - triangleVertices(3,1)) + (triangleVertices(1,1) - triangleVertices(3,1)) * (intersectP(2) - triangleVertices(3,2))) / ...
                ((triangleVertices(2,2) - triangleVertices(3,2)) * (triangleVertices(1,1) - triangleVertices(3,1)) + (triangleVertices(3,1) - triangleVertices(2,1)) * (triangleVertices(1,2) - triangleVertices(3,2)));
        
            % Check if the point is inside the triangle
            isInside = u >= 0 && v >= 0 && u + v <= 1;
        end


        function [linkPoses] = GetLinkPoses(~, q, robot)
            % Get the links for the given robot
            links = robot.links;
        
            % Initialize an array to store the link poses
            linkPoses = zeros(4, 4, length(links) + 1);
            linkPoses(:, :, 1) = robot.base;
        
            for i = 1:length(links)
                link = links(1,i);
        
                % Get the transformation matrix for the current link
                T = trotz(q(1,i) + link.offset) * transl(0, 0, link.d) * transl(link.a, 0, 0) * trotx(link.alpha);
        
                % Update the link poses with the current transformation
                linkPoses(:, :, i + 1) = linkPoses(:, :, i) * T;
            end
        end

        function updateRobot1(self, Robot1JointAngles)
            % Update the robot model based on provided joint angles
            self.robot1.model.animate(Robot1JointAngles);
        end

        function updateRobot2(self, Robot2JointAngles)
            % Update the robot model based on provided joint angles
            self.robot2.model.animate(Robot2JointAngles);
        end
        
        function flipEStop(obj)
            if ~obj.estopFlag
                obj.estopFlag = true;
                disp('Emergency Stop Engaged');
                return;
            end
            if obj.estopFlag
                obj.estopFlag = false;
                disp('Emergency Stop Disengaged');
            end
        end

        function moveTo1(self, x1, y1, z1)
            % Desired end effector pose
            T_desired = transl(x1, y1, z1);

            % Compute inverse kinematics for the desired pose
            q_sol = self.robot1.model.ikcon(T_desired);

            % Current joint configuration
            q_current = self.robot1.model.getpos();

            % Interpolate between current and target joint angles
            q_traj = jtraj(q_current, q_sol, self.steps);

            % Animate the movement
            for k = 1:self.steps
                self.robot1.model.animate(q_traj(k,:));
                pause(0);  % Pause for smooth animation. Adjust as needed.
            end
        end

        function moveTo2(self, x2, y2, z2)
            % Desired end effector pose
            T_desired = transl(x2, y2, z2);

            % Compute inverse kinematics for the desired pose
            q_sol = self.robot2.model.ikcon(T_desired);

            % Current joint configuration
            q_current = self.robot2.model.getpos();

            % Interpolate between current and target joint angles
            q_traj = jtraj(q_current, q_sol, self.steps);

            % Animate the movement
            for k = 1:self.steps
                self.robot2.model.animate(q_traj(k,:));
                pause(0);  % Pause for smooth animation. Adjust as needed.
            end
        end

    end
end
