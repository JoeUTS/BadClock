classdef SeggyDs < handle
    
    properties (Constant)
        originRotation = trotx(90, 'deg');

        % Drawing
        segmentLength = 0.01; % 1cm
        penOffset = 0.05;
        penRaisedOffset = 0.055;
        steps = 25;
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
        penLifted = 1; %

        % Safety
        estopFlag = false; % Add an e-stop flag
        lightCurtainSafe = true;
    end

    methods 
		function self = SeggyDs()
            %% To do list
            % - finish new format for drawing numbers

            %% Questions
            % Nil

			clf
			clc
            % Load robot
            self.robot = DobotMagician(transl(0,0,0));
            hold on

            %% GUI
            % teach
            loadPose = [0,pi/4,pi/4,pi/2,0];
            %robot.model.plot(loadPose, 'noname', 'noarrow', 'notiles', 'nojoints');
            self.robot.model.teach(loadPose, 'noname', 'noarrow', 'notiles', 'nojoints');

            self.MoveToOrigin()

			input('Press enter to begin')
            self.Number0([0.20,0.05,0.1])
            self.Number1([0.20,0,0.1])

            %self.UpdateTime();

            %display(self.hoursArray);
            %display(self.minutesArray);
            %WriteNumber(self, self.minutesArray(1));
			
		end
	end
    
    methods
        function MoveToOrigin(self)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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

        function LiftPen(self)
            % If pen already raised then ignore
            if self.penLifted == 1
                % Add debug message if enabled
                if self.debug == 1
                    disp('Pen already in lifted position');
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

            self.penLifted = 1;

        end

        function LowerPen(self)
            % If pen already raised then ignore
            if self.penLifted == 0
                % Add debug message if enabled
                if self.debug == 1
                    disp('Pen already in lowered position');
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

            self.penLifted = 0;

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
            if (self.debug == 1) && (self.penLifted == 0)
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
            if (self.debug == 1) && (self.penLifted == 0)
                endPoint = self.robot.model.fkine(self.robot.model.getpos());
                plot3([currentCartesian.t(1),endPoint.t(1)], [currentCartesian.t(2), endPoint.t(2)], [currentCartesian.t(3), endPoint.t(3)]);
            end

            % INSERT LOG CODE
            % Compare where should be in theory to where is currently
        end
        
        function Number0(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
        end

        function Number1(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
        end

        function Number2(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
        end

        function Number3(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
            self.JogX(self.segmentLength);
            self.LowerPen();
            self.JogY(-self.segmentLength);

            % Raise pen
            self.LiftPen();
        end

        function Number4(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
            self.JogX(self.segmentLength);
            self.LowerPen();
            self.JogX(-2*self.segmentLength);

            % Raise pen
            self.LiftPen();
        end

        function Number5(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
        end

        function Number6(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
        end

        function Number7(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
        end

        function Number8(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
            self.JogX(-self.segmentLength);
            self.LowerPen();
            self.JogY(-self.segmentLength);

            % Raise pen
            self.LiftPen();
        end

        function Number9(self, digitOrigin)
            % Raise pen if lowered
            if self.penLifted == 0
                self.LiftPen();
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
            self.LiftPen();
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

        function WriteNumber(self, number)
            % Check what the number is and write it
            if number == 0
                self.Number0;
            elseif number == 1
                self.Number1;
            elseif number == 2
                self.Number2;
            elseif number == 3
                self.Number3;
            elseif number == 4
                self.Number4;
            elseif number == 5
                self.Number5;
            elseif number == 6
                self.Number6;
            elseif number == 7
                self.Number7;
            elseif number == 8
                self.Number8;
            elseif number == 9
                self.Number9;
            else
                % add something to log for being out of range
            end
        end

        function WriteTime(self, XYZ)
            % Set origin to XYZ
            % DO THE CODE

            % Get new time, move to origin and write digit 1
            self.UpdateTime();
            self.MoveToOrigin();
            self.WriteNumber(self, self.hoursArray(1));

            % Move origin to next digit
            % DO THE CODE
            self.WriteNumber(self, self.hoursArray(2));

            % Dots in the middle (:) <-- that thing
            % move the origin here
            % DO THE CODE

            % Move origin to next digit
            % DO THE CODE
            self.WriteNumber(self, self.minutesArray(1));

            % Move origin to next digit
            % DO THE CODE
            self.WriteNumber(self, self.minutesArray(2));
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
        end

        function WriteNumberWithSafety(self, number)
            % Check if the light curtain is safe before moving
            if self.lightCurtainSafe
                % Check what the number is and write it
                if number == 0
                    self.Segment0();
                elseif number == 1
                    self.Segment1();
                % Add cases for other numbers
                else
                    % add something to log for being out of range
                end
            else
                % Light curtain is unsafe, stop the robot
                self.engageEStop();
                disp('Light curtain is unsafe. Emergency Stop Engaged.');
            end
        end

        % Add a method to update the light curtain status
        function updateLightCurtainStatus(self, safe)
            self.lightCurtainSafe = safe;
            if safe
                disp('Light curtain is now safe.');
            else
                % If unsafe, immediately engage emergency stop
                self.engageEStop();
                disp('Light curtain is now unsafe. Emergency Stop Engaged.');
            end
        end

    end
end

