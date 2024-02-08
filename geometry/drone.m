%% DRONE GEOMETRY & MOIs
geom.config = 1;                        % 0 for [+], 1 for [x]
geom.mass = 80/1000;                    % Drone weight: 80 grams [kg]
diagonal_length = 11.9/100;             % Diagonal length C2C: 11.9 cm [m]

Ixx = 7.152e-5; Ixy = 0; Ixz = 0;
Iyx = 0; Iyy = 6.537e-5; Iyz = 0;       % MOIs [kg m^2]
Izx = 0; Izy = 0; Izz = 1.492e-4;

geom.J =    [Ixx -Ixy -Ixz;
            -Iyx Iyy -Iyz;              % Inertia tensor [kg m^2]
            -Izx -Izy Izz];

geom.L = diagonal_length/2;             % Length of arm: CG to mid of rotor [m]

geom.phi1 = 313.305;                    % Angle in degrees from x-axis
geom.phi2 = 46.695;
geom.phi3 = 133.305;
geom.phi4 = 226.695;

%% ENVIRONMENT
env.g = 9.81;                           % Gravitaional constant [m/s^2]

%% FLOW FIELD
flow.rho = 1.225;                       % Air density [kg/m^3]
flow.mu = 1.8*10^(-5);                  % Dynamic viscosity [kg/m s]
flow.V = 0;                             % Freestream velocity of drone [m/s]
flow.rotorAoA = 0;                      % Rotor AoA [deg]. Angle between freestream and rotor plane (0 to +/- 90). 
                                        % (Edgewise flight = 0 [deg], Propeller mode = 90 [deg]) - pg. 22 Carroll, Tim

%% ROTOR GEOMETRY & OPERATING CONDITIONS
rotor.oper = 'forwardflight';           % Robot is in forwardflight or hover
rotor.rpm = 25500;                      % Rotor rotational speed [RPM]
rotor.psi = 8;                          % Number of blade azimuth positions [Int]. First is at 0 or 2pi rad.
rotor.cd = 0;                           % Viscous effects
rotor.alpha0 = 0;                       % If viscous effects are off (Cd = 0), include also a zero lift angle estimate [rad]
rotor.cl_alpha = 2*pi;                  % Lift curve slope [1/rad] (2*pi is estimated from Thin Airfoil Theory)
rotor.blades = 2;                       % Number of rotor blades [Int]
rotor.R_tip = 0.0381;                   % Tip radius [m]
rotor.r_R = [0.005 0.0085 0.012...
             0.016 0.0195 0.0225...
             0.0265 0.0295 0.0325...
             0.036 0.0381]/rotor.R_tip; % Non-dimensional radial position, normalized from 0 to 1 by tip radius
rotor.c_R = [0.0075	0.0095 0.01...
            0.009 0.008 0.007...
            0.005 0.0045 0.0042...
            0.004 0.004]/rotor.R_tip;   % c/R normalized chord [c/R]
rotor.beta = [1.0 0.9 0.8 0.7...
              0.6 0.5 0.4 0.3...
              0.25 0.2 0.2]*25;         % Pitch angle of attack of 2D section. Sectional pitch (Beta) [deg]

%% COMPILE
robot.env = env;
robot.flow = flow;
robot.geom = geom;
robot.rotor = rotor;