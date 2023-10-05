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
    end

    methods 
		function self = SeggyDs()
            %% To do list
            % - test number functions
            % - Make number functions accept a custom origin
            % - convert DateTime string to int array
            % - build function to take current DateTime and make robot move

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
            self.Number8();
			
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

    end
end

