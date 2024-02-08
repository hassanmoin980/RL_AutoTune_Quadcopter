clear
close
clc

addpath(genpath('geometry'))
addpath(genpath('aerodynamics'))
run drone.m
run bemt.m

%% DATA
%                               Geometry

L       = robot.geom.L;
maxRPM  = robot.rotor.rpm;
Wmax    = maxRPM*2*pi/60;                          % [rad/s]

%                               Environment
rho     = robot.flow.rho;
g       = robot.env.g;

%                               Aerodynamics
Kf      = robot.aero.Kthrust;
Km      = robot.aero.Ktorque;

% From Introduction to Multicopter Design and Control by Quan Quan
if robot.geom.config == 0

    % [+] Config Transformation Matrix
    T   =  [Kf    Kf     Kf     Kf;
        0     -L*Kf   0      L*Kf;
        L*Kf   0      -L*Kf  0;
        -Km     Km     -Km    Km];

elseif robot.geom.config == 1

    % [x] Config Transformation Matrix
    % CW is positive, CCW is negative
    % Pitch up is positive, Pitch down is negative
    % Propeller 1: CW & Propeller 2: CCW

    T   = [Kf                              Kf                             Kf                             Kf;
        -L*Kf*sind(robot.geom.phi1)     -L*Kf*sind(robot.geom.phi2)    -L*Kf*sind(robot.geom.phi3)    -L*Kf*sind(robot.geom.phi4);
        L*Kf*cosd(robot.geom.phi1)      L*Kf*cosd(robot.geom.phi2)     L*Kf*cosd(robot.geom.phi3)     L*Kf*cosd(robot.geom.phi4);
        -Km                              Km                            -Km                             Km];

end

Tinv    = inv(T);

%% INITIAL CONDITIONS
pos_I_init      = [0 0 0];      % Xe Ye Ze
vel_B_init      = [0 0 0];      % U v w
euler_I_init    = [0 0 0];      % Roll Pitch Yaw
angvel_B_init   = [0 0 0];      % p q r

%% TRAJECTORY GENERATOR
r = 5;
w = 0.125; %rad/s
t = 0:1:50;
x = r*cos(w*t);
y = r*sin(w*t);

filename = "Drone_PID_rev5_Tuned_Manual_PID.slx"

mass    = robot.geom.mass;
J       = robot.geom.J;
out0 = sim(filename, 60);
rng(0)
for i = 1:100
    mass    = robot.geom.mass*(0.40*rand()-0.20+1);
    J       = robot.geom.J*diag(0.40*rand(3,1)-0.20+1);
    out(i) = sim(filename, 60);
    mass_vector(i) = mass;
    J_vector(:,:,i) = J;
    disp(i)
end

save robustness_vector_m mass_vector J_vector out out0 mass J