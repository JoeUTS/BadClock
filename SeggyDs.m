classdef SeggyDs < handle
    
    properties (Constant)
        originRotation = trotx(90, 'deg');
        segmentLength = 0.01; % 1cm
        penOffset = 0.05;
        penRaisedOffset = 0.055;
        steps = 25;

        testLine = 1; % This is to display drawn lines in the simulation
    end

    properties
        robot;
        originTranslation = [0.3,0,0.05]; % Set at roughly the top of the base unit and at max reach
        minutesArray = [9,9]; % initialise as impossible time for bug checking
        hoursArray = [9,9]; % initialise as impossible time for bug checking
        estopFlag = false; % Add an e-stop flag
        lightCurtainSafe = true;
    end

    methods 
		function self = SeggyDs()
            %% To do list
            % - test number functions
            % - Make number functions accept a custom origin

            %% Questions
            % - Do I make another function to raise and lower the pen?
            % - Do I make the segment functions skip raising/lowering if at
            %   the start point of the next segment?

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
            
			input('Press enter to begin')
            self.UpdateTime();

            display(self.hoursArray);
            display(self.minutesArray);
            WriteNumber(self, self.minutesArray(1));
			
		end
	end
    
    methods
        function MoveToOrigin(self)
            % move to origin
            % get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);
                        
            % move to origin
            newCartesian = transl(self.originTranslation + [0,0,self.penRaisedOffset]);   % Nil rotation data entered seems to work the same as rotx90 deg. why?
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');
                        
            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);
                        
            % Animate
            for i = 1:size(trajectory)
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
        
        function SegmentA(self)
            %% Move to Start (origin)
            self.MoveToOrigin();

            %% Lower Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?
            
            % Lower Pen
            newCartesian = transl(self.originTranslation + [0, 0, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');
            
            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);
            
            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end
            
            % INSERT LOG CODE
            
            %% Move one segment to the right
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?
            
            % Move one segment to the right
            newCartesian = transl(self.originTranslation + [0, -self.segmentLength, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');
            
            % Generate trajectory
            % Uses trapezoidal trajectory to keep drawing speed mostly constant
            trajectory = nan(self.steps,5);
            s = lspb(0,1,self.steps);
            
            for i = 1:self.steps
                trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
            end
            
            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % Used to plot drawn lines within simulation
            if self.testLine == 1
                plot3([currentCartesian.t(1),newCartesian(1,4)], [currentCartesian.t(2), newCartesian(2,4)], [currentCartesian.t(3), newCartesian(3,4)]);
            end

            % INSERT LOG CODE
            
            %% Lift Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?
            
            % Lift Pen
            newCartesian = transl(self.originTranslation + [0, -self.segmentLength, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');
            
            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);
            
            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end
            
            % INSERT LOG CODE


        end

        function SegmentB(self)
            %% move to start
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % Is this needed?

            % Move to start (pen raised)
            newCartesian = transl(self.originTranslation + [0, -self.segmentLength, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Lower Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lower Pen
            newCartesian = transl(self.originTranslation + [0, -self.segmentLength, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Move one segment down
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % Move one segment down
            newCartesian = transl(self.originTranslation + [-self.segmentLength, -self.segmentLength, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            % Uses trapezoidal trajectory to keep drawing speed mostly constant
            trajectory = nan(self.steps,5);
            s = lspb(0,1,self.steps);

            for i = 1:self.steps
                trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
            end

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end
            
            % Used to plot drawn lines within simulation
            if self.testLine == 1
                plot3([currentCartesian.t(1),newCartesian(1,4)], [currentCartesian.t(2), newCartesian(2,4)], [currentCartesian.t(3), newCartesian(3,4)]);
            end

            % INSERT LOG CODE

            %% Lift Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lift Pen
            newCartesian = transl(self.originTranslation + [-self.segmentLength, -self.segmentLength, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

        end

        function SegmentC(self)
            %% move to start
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % Is this needed?

            % Move to start (pen raised)
            newCartesian = transl(self.originTranslation + [-self.segmentLength, -self.segmentLength, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Lower Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lower Pen
            newCartesian = transl(self.originTranslation + [-self.segmentLength, -self.segmentLength, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Move one segment down
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % Move one segment down
            newCartesian = transl(self.originTranslation + [-2*self.segmentLength, -self.segmentLength, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            % Uses trapezoidal trajectory to keep drawing speed mostly constant
            trajectory = nan(self.steps,5);
            s = lspb(0,1,self.steps);

            for i = 1:self.steps
                trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
            end

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % Used to plot drawn lines within simulation
            if self.testLine == 1
                plot3([currentCartesian.t(1),newCartesian(1,4)], [currentCartesian.t(2), newCartesian(2,4)], [currentCartesian.t(3), newCartesian(3,4)]);
            end

            % INSERT LOG CODE

            %% Lift Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lift Pen
            newCartesian = transl(self.originTranslation + [-2*self.segmentLength, -self.segmentLength, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

        end

        function SegmentD(self)
            %% move to start
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % Is this needed?

            % Move to start (pen raised)
            newCartesian = transl(self.originTranslation + [-2*self.segmentLength, 0, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Lower Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lower Pen
            newCartesian = transl(self.originTranslation + [-2*self.segmentLength, 0, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Move one segment to the right
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);
            % Move one segment to the right
            newCartesian = transl(self.originTranslation + [-2*self.segmentLength, -self.segmentLength, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            % Uses trapezoidal trajectory to keep drawing speed mostly constant
            trajectory = nan(self.steps,5);
            s = lspb(0,1,self.steps);

            for i = 1:self.steps
                trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
            end

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % Used to plot drawn lines within simulation
            if self.testLine == 1
                plot3([currentCartesian.t(1),newCartesian(1,4)], [currentCartesian.t(2), newCartesian(2,4)], [currentCartesian.t(3), newCartesian(3,4)]);
            end

            % INSERT LOG CODE

            %% Lift Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lift Pen
            newCartesian = transl(self.originTranslation + [-2*self.segmentLength, -self.segmentLength, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

        end

        function SegmentE(self)
            %% move to start
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % Is this needed?

            % Move to start (pen raised)
            newCartesian = transl(self.originTranslation + [-self.segmentLength, 0, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Lower Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lower Pen
            newCartesian = transl(self.originTranslation + [-self.segmentLength, 0, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Move one segment down
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % Move one segment down
            newCartesian = transl(self.originTranslation + [-2*self.segmentLength, 0, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            % Uses trapezoidal trajectory to keep drawing speed mostly constant
            trajectory = nan(self.steps,5);
            s = lspb(0,1,self.steps);

            for i = 1:self.steps
                trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
            end

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % Used to plot drawn lines within simulation
            if self.testLine == 1
                plot3([currentCartesian.t(1),newCartesian(1,4)], [currentCartesian.t(2), newCartesian(2,4)], [currentCartesian.t(3), newCartesian(3,4)]);
            end

            % INSERT LOG CODE

            %% Lift Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lift Pen
            newCartesian = transl(self.originTranslation + [-2*self.segmentLength, 0, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

        end

        function SegmentF(self)
            %% Move to Start (origin)
            self.MoveToOrigin();

            %% Lower Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lower Pen
            newCartesian = transl(self.originTranslation + [0, 0, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Move one segment down
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % Move one segment down
            newCartesian = transl(self.originTranslation + [-self.segmentLength, 0, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            % Uses trapezoidal trajectory to keep drawing speed mostly constant
            trajectory = nan(self.steps,5);
            s = lspb(0,1,self.steps);

            for i = 1:self.steps
                trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
            end

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % Used to plot drawn lines within simulation
            if self.testLine == 1
                plot3([currentCartesian.t(1),newCartesian(1,4)], [currentCartesian.t(2), newCartesian(2,4)], [currentCartesian.t(3), newCartesian(3,4)]);
            end

            % INSERT LOG CODE

            %% Lift Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lift Pen
            newCartesian = transl(self.originTranslation + [-self.segmentLength, 0, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

        end

        function SegmentG(self)
            %% move to start
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % Is this needed?

            % Move to origin (pen raised)
            newCartesian = transl(self.originTranslation + [-self.segmentLength, 0, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Lower Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lower Pen
            newCartesian = transl(self.originTranslation + [-self.segmentLength, 0, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

            %% Move one segment to the right
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);

            % Move one segment to the right
            newCartesian = transl(self.originTranslation + [-self.segmentLength, -self.segmentLength, self.penOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            % Uses trapezoidal trajectory to keep drawing speed mostly constant
            trajectory = nan(self.steps,5);
            s = lspb(0,1,self.steps);

            for i = 1:self.steps
                trajectory(i,:) = (1-s(i))*currentPose + s(i)*newPose;
            end

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % Used to plot drawn lines within simulation
            if self.testLine == 1
                plot3([currentCartesian.t(1),newCartesian(1,4)], [currentCartesian.t(2), newCartesian(2,4)], [currentCartesian.t(3), newCartesian(3,4)]);
            end

            % INSERT LOG CODE

            %% Lift Pen
            % Get current pose
            currentPose = self.robot.model.getpos();
            currentCartesian = self.robot.model.fkine(currentPose);  % is this needed?

            % Lift Pen
            newCartesian = transl(self.originTranslation + [-self.segmentLength, -self.segmentLength, self.penRaisedOffset]);
            newPose = self.robot.model.ikine(newCartesian, 'q0', currentPose, 'mask', [1 1 1 1 1 0], 'forceSoln');

            % Generate trajectory
            trajectory = jtraj(currentPose, newPose, self.steps);

            % Animate
            for i = 1:size(trajectory)
                animateStep = trajectory(i,:);
                self.robot.model.animate(animateStep);
                drawnow();
            end

            % INSERT LOG CODE

        end
        
        function Number0(self)
            self.SegmentA();
            self.SegmentB();
            self.SegmentC();
            self.SegmentD();
            self.SegmentE();
            self.SegmentF();
        end

        function Number1(self)
            self.SegmentB();
            self.SegmentC();
        end

        function Number2(self)
            self.SegmentA();
            self.SegmentB();
            self.SegmentG();
            self.SegmentE();
            self.SegmentD();
        end

        function Number3(self)
            self.SegmentA();
            self.Number1();
            self.SegmentD();
            self.SegmentG();
        end

        function Number4(self)
            self.SegmentF();
            self.SegmentG();
            self.Number1();
        end

        function Number5(self)
            self.SegmentA();
            self.SegmentF();
            self.SegmentG();
            self.SegmentC();
            self.SegmentD();
        end

        function Number6(self)
            self.SegmentF();
            self.SegmentE();
            self.SegmentD();
            self.SegmentG();
            self.SegmentC();
        end

        function Number7(self)
            self.SegmentA();
            self.Number1();
        end

        function Number8(self)
            self.Number0();
            self.SegmentG();
        end

        function Number9(self)
            self.SegmentA();
            self.Number1();
            self.SegmentF();
            self.SegmentG();
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
            self.minutesArray = minuteDigits
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

