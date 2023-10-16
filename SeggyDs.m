classdef SeggyDs < handle
    
    properties (Constant)
        originRotation = trotx(90, 'deg');

        % Drawing
        segmentLength = 0.01; % 1cm
        penOffset = 0.05;
        penRaisedOffset = 0.055;
        steps = 10; % THIS IS LOW FOR TESTING. MAKE ME HIGHER!!!!!
        errorTollerance = 0.005; %5mm can be lowered later 
        
        % Debug
        debug = 1; % This is to display drawn lines in the simulation
    end

    properties
        robot;

        % Drawing
        originTranslation = [0.3,0,0.1]; % Set at roughly the top of the base unit
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
        operationRunning = 1;
        digit1Status = 0;
        digit2Status = 0;
        colonStatus = 0;
        digit3Status = 0;
        digit4Status = 0;
        

        % Safety
        estopFlag = false; % Add an e-stop flag
        lightCurtainSize = [0.01, 0.8, 0.4];  % Default size
        lightCurtainPosition = [0.4, -0.4, 0];  % Default position
        lightCurtainSafe = true;
    end

    methods 
		function self = SeggyDs()
            %% To do list
            % - bugfix colon polacement in writing time

            %% Questions
            % - do we ask if wanting to recover when clearing E-Stop or do we just do it?

			clf
			clc
            % Load robot
            self.robot = DobotMagician(transl(0,0,0));
            hold on

            %% GUI
            % teach
            self.robot.model.teach('noname', 'noarrow', 'notiles', 'nojoints');

            % Light curtain load in
            self.InitializeLightCurtain(self.lightCurtainSize, self.lightCurtainPosition);
            self.VisualizeLightCurtain();

            self.MoveToOrigin()

			input('Press enter to begin')
            self.operationRecovery();
            %self.WriteTime([0.2,0.1,0.2])
			
		end
	end
    
    methods
        function MoveToOrigin(self)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % move to origin
            % get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);
                        
            % move to origin
            newCartesian = transl(self.originTranslation + [0,0,self.penRaisedOffset]);   % Nil rotation data entered seems to work the same as rotx90 deg. why?
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');
                        
            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);
            
            % Add debug message if enabled
            if self.debug == 1
                disp('moving to origin');
            end

            % Animate
            for i = 1:size(trajectory)
                % Check for EStop and cease operation if engaged
                if self.estopFlag
                    if self.debug == 1
                        disp('ERROR: E-STOP ENGAGED');
                        input('press enter to continue');
                    end
                    return
                end
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % Log theoretical location vs acutal location
            % Log currentCartesian
            % INSERT CODE HERE
            % Log actualTransform
            actualTransform = self.robot.model.fkine(self.robot.model.getpos());
            % INSERT LOG CODE
            % Calculate delta
            errorTransform = actualTransform.t' - currentCartesian.t';
            errorAmplitude = sqrt(errorTransform(1)^2 + errorTransform(2)^2 + errorTransform(3)^2);
            % INSERT LOG CODE
            if errorAmplitude > 0.0005  % 0.5mm error
                % INSERT LOG CODE TO DISPLAY AN ERROR
            end
            
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

            % Get current position
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

             % Lift Pen
            newCartesian = transl([currentCartesian.t(1), currentCartesian.t(2), (self.originTranslation(3) + self.penRaisedOffset)]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');
            
            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);
            
            % Add debug message if enabled
            if self.debug == 1
                disp('Lifting Pen');
            end

            % Animate
            for i = 1:size(trajectory)
                % Check for EStop and cease operation if engaged
                if self.estopFlag
                    if self.debug == 1
                        disp('ERROR: E-STOP ENGAGED');
                    end
                    return
                end
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

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
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

             % Lift Pen
            newCartesian = transl([currentCartesian.t(1), currentCartesian.t(2), (self.originTranslation(3) + self.penOffset)]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');
            
            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);
            
            % Add debug message if enabled
            if self.debug == 1
                disp('Lowering Pen');
            end

            % Animate
            for i = 1:size(trajectory)
                % Check for EStop and cease operation if engaged
                if self.estopFlag
                    if self.debug == 1
                        disp('ERROR: E-STOP ENGAGED');
                    end
                    return
                end
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            self.penRaised = 0;

        end

        function JogX(self, distance)
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % Move one segment to the right
            newCartesian = transl(currentCartesian.t' + [distance, 0, 0]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');
            
            % Generate trajectory
            % Uses trapezoidal trajectory to keep drawing speed mostly constant
            trajectory = nan(self.steps,5);
            s = lspb(0,1,self.steps);
            
            for i = 1:self.steps
                trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
            end
            
            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('Jogging in X axis. Distance: %d', distance);
                disp(message);
            end

            % Animate
            for i = 1:size(trajectory)
                % Check for EStop and cease operation if engaged
                if self.estopFlag
                    if self.debug == 1
                        disp('ERROR: E-STOP ENGAGED');
                    end
                    return
                end
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % Used to plot drawn lines within simulation
            if (self.debug == 1) && (self.penRaised == 0)
                endPoint = self.robot.model.fkine(self.robot.model.getpos());
                plot3([currentCartesian.t(1),endPoint.t(1)], [currentCartesian.t(2), endPoint.t(2)], [currentCartesian.t(3), endPoint.t(3)]);
            end

            % INSERT LOG CODE
            % Compare where should be in theory to where is currently
        end

        function JogY(self, distance)
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Move one segment to the right
            newCartesian = transl(currentCartesian.t' + [0, distance, 0]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');
            
            % Generate trajectory
            % Uses trapezoidal trajectory to keep drawing speed mostly constant
            trajectory = nan(self.steps,5);
            s = lspb(0,1,self.steps);
            
            for i = 1:self.steps
                trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
            end

            % Add debug message if enabled
            if self.debug == 1
                message = sprintf('Jogging in Y axis. Distance: %d', distance);
                disp(message);
            end
            
            % Animate
            for i = 1:size(trajectory)
                % Check for EStop and cease operation if engaged
                if self.estopFlag
                    if self.debug == 1
                        disp('ERROR: E-STOP ENGAGED');
                    end
                    return
                end
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % Used to plot drawn lines within simulation
            if (self.debug == 1) && (self.penRaised == 0)
                endPoint = self.robot.model.fkine(self.robot.model.getpos());
                plot3([currentCartesian.t(1),endPoint.t(1)], [currentCartesian.t(2), endPoint.t(2)], [currentCartesian.t(3), endPoint.t(3)]);
            end

            % INSERT LOG CODE
            % Compare where should be in theory to where is currently
        end
        
        function Number0(self, digitOrigin)
            % Raise pen if lowered
            if self.penRaised == 0
                self.RaisePen();
            end

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);
            
            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);
                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end
            
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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end
            
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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end

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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end

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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end

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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end

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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end

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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end

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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end

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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end

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

            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % If not close to digit origin point then move there
            if (abs(currentCartesian.t(1)) - abs(digitOrigin(1)) > self.errorTollerance) || (abs(currentCartesian.t(2)) - abs(digitOrigin(2)) > self.errorTollerance)
                % Move to digit origin
                newCartesian = transl(digitOrigin + [0, 0, self.penRaisedOffset]);
                newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

                % Generate trajectory
                % Uses trapezoidal trajectory to keep drawing speed mostly constant
                trajectory = nan(self.steps,5);

                s = lspb(0,1,self.steps);

                for i = 1:self.steps
                    trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
                end

                % Add debug message if enabled
                if self.debug == 1
                    disp('Moving to digit origin');
                end

                % Animate
                for i = 1:size(trajectory)
                    % Check for EStop and cease operation if engaged
                    if self.estopFlag
                        if self.debug == 1
                            disp('ERROR: E-STOP ENGAGED');
                        end
                        return
                    end
                    animateStep = trajectory(i,:);
                    self.robot.model.animate(animateStep);
                    drawnow();
                end
            end

            % Move right and down then Lower pen
            self.JogY(-self.segmentLength/3);
            self.JogX(-self.segmentLength/3);
            self.LowerPen();

            % Add debug message if enabled
            if self.debug == 1
                disp('Drawing digit: 9');
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

            if hourArraySize < 2
                hourIntermediary = [0,hourDigits];
            else
                hourIntermediary = hourDigits;
            end
            
            % Update global variables
            self.hoursArray = hourIntermediary;
            self.minutesArray = minuteDigits;
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
            % Set recovery flags
            self.operationRunning = 1;
            self.digit1Status = 0;
            self.digit2Status = 0;
            self.digit3Status = 0;
            self.digit4Status = 0;
            self.colonStatus = 0;

            % Set digit origins
            self.digit1Origin = timeOrigin;
            self.digit2Origin = self.digit1Origin - [0, (1.5*self.segmentLength), 0];
            self.colonOrigin = self.digit2Origin - [0, (1.5*self.segmentLength), 0];
            self.digit3Origin = self.colonOrigin - [0, (1.5*self.segmentLength), 0];
            self.digit4Origin = self.digit3Origin - [0, (1.5*self.segmentLength), 0];

            % Get new time, move to origin and write time
            self.UpdateTime();
            self.MoveToOrigin();
            %self.WriteNumber(self.hoursArray(1), self.digit1Origin);
            self.WriteNumber(1, self.digit1Origin);
            self.digit1Status = 1;
            %self.WriteNumber(self.hoursArray(2), self.digit2Origin);
            self.WriteNumber(2, self.digit2Origin);
            self.digit2Status = 1;
            self.DrawColon(self.colonOrigin);
            self.colonStatus = 1;
            %self.WriteNumber(self.minutesArray(1), self.digit3Origin);
            self.WriteNumber(3, self.digit3Origin);
            self.digit3Status = 1;
            %self.WriteNumber(self.minutesArray(2), self.digit4Origin);
            self.WriteNumber(4, self.digit4Origin);
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

            % ask if wanting to recover?

            % Add debug message if enabled
            if self.debug == 1
                disp('Resuming operation');
            end

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

        function InitializeLightCurtain(self, size, position)

            % Initialize the light curtain with custom parameters
            self.lightCurtainPosition = position;
            self.lightCurtainSize = size;
    
            % Implement collision detecting
    
            % Set the light curtain to the initial safe state
            self.lightCurtainSafe = true;
        end

        function UpdateLightCurtainStatus(self, obstacleDetected)

            % Check if the provided 'obstacleDetected' parameter is valid
            if islogical(obstacleDetected)
                if obstacleDetected

                    % An obstacle has been detected in the light curtain
                    % area (need collision detection)

                    % Update the status of the light curtain to be unsafe
                    self.lightCurtainSafe = false;
                    % Engage the emergency stop
                    self.engageEStop();
                    disp('Obstacle detected in the light curtain area. Emergency Stop Engaged.');
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
            z = [self.lightCurtainPosition(3), self.lightCurtainPosition(3) + self.lightCurtainSize(3);
                self.lightCurtainPosition(3), self.lightCurtainPosition(3) + self.lightCurtainSize(3)];

            % Create the filled plane using surf
            surf(x, y, z, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
         
            % Show the plot
            drawnow;
        end

    end
end
