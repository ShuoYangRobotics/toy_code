%parameters for ground contract
ground.stiff = 5e4;
ground.damp = 1000;

%body size
body.x_length = 0.6;
body.y_length = 0.3;
body.z_length = 0.15;
body.shoulder_size = 0.07;
body.upper_length = 0.2;
body.lower_length = 0.2;
body.foot_radius = 0.035;
body.shoulder_distance = 0.2;

% parameters for leg control 
ctrl.k_p = 100;
ctrl.k_d = 2.8;

% parameters for high level plan
planner.touch_down_height = 0.239; % from foot center to ground
planner.stand_height = 0.55; % 0.2*cos(30/180*pi)*2+0.2345 = 0.5809 
                             % due to joint torque, the exact height is
                             % 0.5796
planner.flight_height = 0.35;% when leg is flying in the air
planner.stand_s = 0;
planner.stand_u = 30*pi/180;
planner.stand_k = -60*pi/180;

