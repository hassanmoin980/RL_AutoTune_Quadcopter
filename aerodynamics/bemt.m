% BLADE ELEMENT MOMENTUM THEORY FUNCTION

V           = robot.flow.V;                   % Drone flight speed in [m/s]
AoA_deg     = robot.flow.rotorAoA;            % Drone Angle of Attack / inrobot.flow angle to robot.rotor plane [deg]
B           = robot.rotor.blades;             % Number of blades
R           = robot.rotor.R_tip;              % robot.rotor tip radius [m]
r_R         = robot.rotor.r_R;                % Normalized sectional positions
c_R         = robot.rotor.c_R;                % Normalized sectional chord lengths
beta_deg    = robot.rotor.beta;               % Sectional pitch [deg]
n           = length(r_R);                    % Number of sections
azm         = robot.rotor.psi;                % Azimuth positions setup
if azm == 2                                   % For 2 points, set them to retreating/advancing positions
    psi_rad     = [pi/2,3*pi/2];
else                                          % Otherwise create points at each azimuth angle for one complete circle
    psi_rad     = linspace(0,2*pi-(2*pi/azm),azm);
end

%% Unit Conversions
beta_rad    = deg2rad(beta_deg);        % Sectional pitch [rad]
dia         = 2*R;                      % robot.rotor dia [m]
c           = c_R.*R;                   % Dimensional chord lengths [m]
AoA_rad     = deg2rad(AoA_deg);         % Angle of attack [rad]
y_span      = r_R.*R;                   % Radial station distance [m]
rps         = robot.rotor.rpm/60;             % Rotational speed [revs/sec]
omega       = 2*pi.*rps;                % Angular frequency [rad/s]. 'Rotational frequency' -> (2pi*rev/s)

%% SETTING UP MID-STATIONS
mid_span    = zeros(n-1,1);
beta_mid    = zeros(n-1,1);
chord_mid   = zeros(n-1,1);
r_mid       = zeros(n-1,1);

for i = 1:(n-1)
    r_mid(i,:)      = (r_R(i)+r_R(i+1))/2;                      % Non-dimensional radial position: "r" in Leishman
    beta_mid(i,:)   = (beta_rad(i)+beta_rad(i+1))/2;            % Mid location twist [rad]
    chord_mid(i,:)  = (c(i)+c(i+1))/2;                          % Mid location chord [m]
    mid_span(i,:)   = (y_span(i)+y_span(i+1))/2;                % Mid_span [m]
    sigma_mid(i,:)  = B.*chord_mid(i)./(2*pi.*mid_span(i));     % Sectional solidity ratio
end

%% INITIAL CALCULATIONS
% 0 [deg] is back of robot.rotor plane. Spins CCW: blade advances first -> pg. 22
omega_mid   = mid_span.*omega;          % Tangential velocity at each section [m/s]

% Velocity component normal to robot.rotor plane
Va_rp       = V.*sin(AoA_rad);          % Axial freestream velocity component [m/s] pg. 26 & 27
Vp_rp       = V.*cos(AoA_rad);          % Parallel freestream velocity component [m/s] pg. 26 & 27

% Velocity component parallel to disk, wrt blade frame
Vp_az       = Vp_rp.*sin(psi_rad);      % Inrobot.flow component perpendicular to blade (not robot.rotor!) perpendicular to robot.flow at current azimuth angle. Advances first.
Vt_az       = Vp_rp.*cos(psi_rad);      % Inrobot.flow component tangential (span wise along) to blade (not robot.rotor!)

% Velocity components at each blade element
V_A = Va_rp;                            % Axial velocity component with vi = 0 pg. 27
V_T = Vp_az + omega_mid;                % Tangential velocity component with ui = 0 pg. 27

% Advance ratios
mu_x        = Vp_rp./(omega.*R);        % robot.rotor advance ratio parallel to robot.rotor disk (tangential) pg. 24
mu_z        = Va_rp./(omega.*R);        % robot.rotor advance ratio normal to robot.rotor disk (axial) pg. 24
mu_inf      = V./(omega.*R);            % Freestream advance ratio pg. 24

% Global solidity (excluding hub area)
sigma_g     = (B*trapz(y_span,c))/((pi*R^2)-(pi*y_span(1)^2));    % Sectional solidity ratio.

%% robot.rotor INrobot.flow MODELS

if strcmp(robot.rotor.oper, 'forwardflight') == 1
    % From Uniform momentum & forward_flight models
    % Assume 0 inrobot.flow to get initial Ct guess from BET
    %------------------------- Blade Element Theory --------------------------
    vi          = 0;                                        % Induced velocity, normal to the robot.rotor disk
    ui          = 0;                                        % Swirl velocity, in-plane toward the robot.rotor blade
    V_R         = sqrt((V_A + vi).^2 + (V_T - ui).^2);      % Resultant velocity (Axial Velocity + Tangential Velocity) pg. 27
    phi         = atan2(V_A + vi, V_T - ui);                % Angle of V_R relative to robot.rotor plane. (AoA = beta + alpha0 - phi) pg. 27
    AoA_eff     = beta_mid - phi;                           % Effective Angle of Attack pg. 27
    
    % Assuming Thin Airfoil Theory and no viscous effects for simplicity
    AoA_eff     =       AoA_eff - robot.rotor.alpha0;       % pg. 27
    c_l         =       robot.rotor.cl_alpha.*AoA_eff;
    c_d         =       0;
    
    dL          =       0.5.*robot.flow.rho.*(V_R.^2).*chord_mid.*c_l;        % Differential lift forces pg. 28
    dD          =       0.5.*robot.flow.rho.*(V_R.^2).*chord_mid.*c_d;        % Differential drag forces pg. 28
    dT          =       B.*(dL.*cos(phi)-(dD.*sin(phi)));                     % Differential thrust pg. 28
    dCT         =       dT./(robot.flow.rho.*(pi.*R^2).*(omega.*R).^2);       % Differential torque pg. 28
    T           =       (sum(simps(mid_span,dT)+simps([mid_span(n-1);R],[dT(n-1,:);zeros(1,azm)])))./azm;
    CT          =       T./(robot.flow.rho.*(pi.*R^2).*(omega.*R).^2);
    
    %---------------------------- Momentum Theory ----------------------------
    CT_old      = CT;
    lambda_old  = sqrt(CT_old/2);
    
    % Check statement to see state of robot.rotor
    if CT > 0
        state   = 1;                          % Regular working state
    elseif CT == 0
        state   = 0;                          % No work state
    end
    
    % Outer loop for convergence on Thrust Loading
    count_CT    = 1;
    epsilonCT   = 1;
    while epsilonCT > 0.0005 && count_CT < 100
        
        % If no inrobot.flow component, solution reduces to BET (exact case)
        if state == 0
            lambda      = Va_rp./(omega.*R);
            break
        end
        
        % Inner loop for convergence on uniform inrobot.flow, given outer loop's current thrust loading
        count           = 1;
        epsilon_lambda  = 1;
        while epsilon_lambda > 0.0005 && count < 25
            
            if state == 1
                % Newton-Raphson iteration method
                f          =   lambda_old - mu_x.*tan(AoA_rad)-(CT_old./(2*sqrt(mu_x.^2+lambda_old.^2)));   % Forward flight inrobot.flow ratio pg. 41
                f_prime    =   1 + (CT_old./2).*(mu_x.^2+lambda_old.^2).^(-3/2).*lambda_old;                % Derivative of above equation pg. 42
                lambda_new =   lambda_old - (f/f_prime);
            end
            
            if isreal(lambda_new) == 0
                disp(['Error in FF iteration scheme at ' num2str(V) ' m/s'])
                break
            end
            
            % Convergence criteria
            epsilon_lambda              = abs((lambda_new - lambda_old)./lambda_new);
            
            % Convergence for lambda_i (centre of robot.rotor) and CT
            plot_lambda_old(count,:)    = lambda_old;
            lambda_old                  = lambda_new;
            count                       = count + 1;
        end
        
        % Induced inrobot.flow from uniform momentum theory (at center of robot.rotor)
        % Uniform inrobot.flow - freestream inrobot.flow ratio
        lambda_0    = lambda_new - (Va_rp./(omega.*R));         % pg. 24
        
        % Apply linear inrobot.flow model here
        chi         =       atan(mu_x/(mu_z+lambda_0));         % pg. 43
        
        % Pitt and Peters linear inrobot.flow model
        kx          =       (15*pi/23)*tan(chi/2);              % pg. 44
        ky          =       0;
        
        % Drees linear inrobot.flow model
        %         kx          =       (4/3).*(1 - cos(chi)-1.8.*(mu_x.^2))./sin(chi);
        %         ky          =       -2.*mu_x;
        
        lambda_i      =      lambda_0.*(1+kx.*r_mid.*cos(psi_rad)+ky.*r_mid.*sin(psi_rad));     % pg. 46
        
        % Induced velocity dependant on working state
        vi         =   (lambda_i.*omega.*R);
        ui = 0;
        
        % Re-run BET for new CT. Positive vi is down direction
        V_R         = sqrt((V_A + vi).^2 + (V_T - ui).^2);      % Resultant velocity (Axial Velocity + Tangential Velocity) pg. 27
        phi         = atan2(V_A + vi, V_T - ui);                % Angle of V_R relative to robot.rotor plane. (AoA = beta + alpha0 - phi) pg. 27
        AoA_eff     = beta_mid - phi;                           % Effective Angle of Attack pg. 27
        
        % Assuming Thin Airfoil Theory and no viscous effects for simplicity
        AoA_eff     = AoA_eff - robot.rotor.alpha0;       % pg. 27
        c_l         = robot.rotor.cl_alpha.*AoA_eff;
        c_d         = 0;
        
        dL          = 0.5.*robot.flow.rho.*(V_R.^2).*chord_mid.*c_l;        % Differential lift forces pg. 28
        dD          = 0.5.*robot.flow.rho.*(V_R.^2).*chord_mid.*c_d;        % Differential drag forces pg. 28
        dT          = B.*(dL.*cos(phi)-(dD.*sin(phi)));               % Differential thrust pg. 28
        dCT         = dT./(robot.flow.rho.*(pi.*R^2).*(omega.*R).^2);       % Differential torque pg. 28
        T           = (sum(simps(mid_span,dT)+simps([mid_span(n-1);R],[dT(n-1,:);zeros(1,azm)])))./azm;
        CT          = T./(robot.flow.rho.*(pi.*R^2).*(omega.*R).^2);
        
        if isreal(phi) == 0
            disp(['Error in FF iteration scheme at ' num2str(V) ' m/s'])
            break
        end
        
        % Update iterative convergence criteria and current CT
        epsilonCT = abs((CT - CT_old)./CT);
        CT_old = CT;
        count_CT = count_CT + 1;
    end
    lambda = (Va_rp+vi)./(omega.*R);
    
elseif strcmp(robot.rotor.oper, 'hover') == 1
    
    F = ones(n-1,1);
    checkF = 1;
    countF = 1;                                 % Initialize iteration count
    lambda = (Va_rp)./(omega.*R);
    lambda_inf = (Va_rp)./(omega.*R);           % Recall: lambda = (Vrp_a + vi)./(omega.*R);
    negative_inrobot.flow_stations = ones(n-1,1);
    
    while checkF == 1 && countF < 50            % Iteration scheme for tiploss factor, F (need to iterate because f affects induced velocity)
        F_old       = F;
        
        % Estimate zero lift angle based on section Re
        V_R            = sqrt((V_A + vi).^2 + (V_T - ui).^2);      % Resultant velocity (Axial Velocity + Tangential Velocity) pg. 27
        beta_zero      = -robot.rotor.alpha0;
        
        cl_alpha       = robot.rotor.cl_alpha;                           % Lift curve slope [1/rad]
        
        lambda         = sqrt(((sigma_mid.*(cl_alpha).*r_mid./(8.*F))-(lambda_inf./2)).^2+... % Tsodak approach
            (sigma_mid.*(cl_alpha).*(beta_mid+beta_zero).*r_mid.^2./(4.*F)))-...
            ((sigma_mid.*r_mid.*(cl_alpha)./(8.*F))-(lambda_inf./2));
        
        if (any(lambda < 0) == 1) || (any(isreal(lambda)) == 0)
            [row,column] = find((imag(lambda)~=0)|(lambda<0));
            negative_inrobot.flow_stations(row) = 0;
        end
        
        lambda = lambda.*negative_inrobot.flow_stations;
        
        % Tiploss function (Prandtl. Iterative since function of inrobot.flow ratio)
        f_t             =       (B/2).*((1-r_mid)./(r_mid.*(lambda./r_mid)));
        f_r             =       (B/2).*((r_mid)./((1-r_mid).*(lambda./r_mid)));
        F               =       (2/pi).*(acos(exp(-f_t)))*(2/pi).*(acos(exp(-f_r)));
        
        checkF          =       any(any(abs((F - F_old)./F)>0.005)==1);
        countF          =       countF + 1;
    end
    
    
end
ui = 0;

%% BLADE VELOCITIES, SECTIONAL COEFFICIENTS AND ADJACENT WAKE EFFECTS

% Inrobot.flow angle, (Pitch - inrobot.flow angle = Effective AoA)
phi         = atan2((lambda.*(omega.*R)),V_T - ui);
% Resultant velocity at each element
V_R         = sqrt((lambda.*(omega.*R)).^2+(Vp_az+omega_mid-ui).^2);
% Mach number at each element, assuming perfect gas @ sea level
Mach_mid    = abs(V_R)./340;
%  Determine section AoA and coefficients
AoA_eff     = beta_mid - phi;           % Effective Angle of Attack pg. 27

% Assuming Thin Airfoil Theory and no viscous effects for simplicity
AoA_eff     = AoA_eff - robot.rotor.alpha0;       % pg. 27
c_l         = robot.rotor.cl_alpha.*AoA_eff;
c_d         = 0;
c_m         = 0;

%% BLADE FORCES

% Spanwise elemental lift and drag as a function of azimuth position
dL          = 0.5.*robot.flow.rho.*(V_R.^2).*chord_mid.*c_l;
dD          = 0.5.*robot.flow.rho.*(V_R.^2).*chord_mid.*c_d;
dM          = 0.5.*robot.flow.rho.*(V_R.^2).*chord_mid.^2.*c_m;

% Spanwise elemental blade forces and moments
% Thrust (integrated from R(1) to tip)
% Torque (integrated from rotational axis, R = 0, to tip)
% Longitudinal force (normal force, "P-factor")
% Lateral force (side force)
% Rolling moment
% Pitching moment

dT          = B.*(dL.*cos(phi)-(dD.*sin(phi)));
dQ          = B.*(dL.*sin(phi)+(dD.*cos(phi))).*mid_span;
dP          = B.*(dL.*sin(phi)+(dD.*cos(phi))).*omega_mid;

dNx         = B.*-(dL.*sin(phi)+(dD.*cos(phi))).*sin(psi_rad);
dNy         = B.*(dL.*sin(phi)+(dD.*cos(phi))).*cos(psi_rad);
dMx         = B.*-(dL.*cos(phi)-(dD.*sin(phi))).*mid_span.*sin(psi_rad) + dM.*cos(psi_rad);
dMy         = B.*(dL.*cos(phi)-(dD.*sin(phi))).*mid_span.*cos(psi_rad) - dM.*sin(psi_rad);

% Induced drag and decomposed power sources ( Induced, Parasite)
dD_i        = dL.*sin(phi);                  % Induced drag
dP_i        = B.*(dL.*sin(phi)).*omega_mid;  % Induced power
dP_p        = B.*(dD.*cos(phi)).*omega_mid;  % Profile power

% Small angle approximations for the elemental blade forces
dT_small    = B.*(dL);
dP_small    = B.*(dL.*sin(phi)+(dD)).*omega_mid;
dP_p_small  = B.*(dD).*omega_mid;

% Blade forces averaged over full rotation
% Thrust (integrated from R(1) to tip)
% Torque (integrated from rotational axis, R = 0, to tip)
% Longitudinal force (normal force)
% Lateral force (side force)
% Rolling moment
% Pitching moment

T           =       (sum(simps(mid_span,dT)+simps([mid_span(n-1);R],[dT(n-1,:);zeros(1,azm)])))./azm;
Q           =       (sum(simps(mid_span,dQ)+simps([0;mid_span(1)],[zeros(1,azm);dQ(1,:)])+simps([mid_span(n-1);R],[dQ(n-1,:);zeros(1,azm)])))./azm;
Nx          =       (sum(simps(mid_span,dNx)+simps([0;mid_span(1)],[zeros(1,azm);dNx(1,:)])+simps([mid_span(n-1);R],[dNx(n-1,:);zeros(1,azm)])))./azm;
Ny          =       (sum(simps(mid_span,dNy)+simps([0;mid_span(1)],[zeros(1,azm);dNy(1,:)])+simps([mid_span(n-1);R],[dNy(n-1,:);zeros(1,azm)])))./azm;
Mx          =       (sum(simps(mid_span,dMx)+simps([0;mid_span(1)],[zeros(1,azm);dMx(1,:)])+simps([mid_span(n-1);R],[dMx(n-1,:);zeros(1,azm)])))./azm;
My          =       (sum(simps(mid_span,dMy)+simps([0;mid_span(1)],[zeros(1,azm);dMy(1,:)])+simps([mid_span(n-1);R],[dMy(n-1,:);zeros(1,azm)])))./azm;

% Vector sum of in-plane forces and azimuth direction (in rads)
Nsum        =       sqrt(Nx^2+Ny^2);
N_angle     =       atan2(Ny,Nx);                   % [Rads]

% Total power and decomposed power sources (Total, Induced, Parasite)
P           =       (sum(simps(mid_span,dP)+simps([0;mid_span(1)],[zeros(1,azm);dP(1,:)])+simps([mid_span(n-1);R],[dP(n-1,:);zeros(1,azm)])))./azm;
P_i         =       (sum(simps(mid_span,dP_i)+simps([0;mid_span(1)],[zeros(1,azm);dP_i(1,:)])+simps([mid_span(n-1);R],[dP_i(n-1,:);zeros(1,azm)])))./azm;
P_p         =       (sum(simps(mid_span,dP_p)+simps([0;mid_span(1)],[zeros(1,azm);dP_p(1,:)])+simps([mid_span(n-1);R],[dP_p(n-1,:);zeros(1,azm)])))./azm;

% Induced drag
D_i         =       (sum(simps(mid_span,dD_i)+simps([0;mid_span(1)],[zeros(1,azm);dD_i(1,:)])+simps([mid_span(n-1);R],[dD_i(n-1,:);zeros(1,azm)])))./azm;

% "Small angle" approximations
T_small     =       (sum(simps(mid_span,dT_small)+simps([0;mid_span(1)],[zeros(1,azm);dT_small(1,:)])+simps([mid_span(n-1);R],[dT_small(n-1,:);zeros(1,azm)])))./azm;
P_small     =       (sum(simps(mid_span,dP_small)+simps([0;mid_span(1)],[zeros(1,azm);dP_small(1,:)])+simps([mid_span(n-1);R],[dP_small(n-1,:);zeros(1,azm)])))./azm;
P_p_small   =       (sum(simps(mid_span,dP_p_small)+simps([0;mid_span(1)],[zeros(1,azm);dP_p_small(1,:)])+simps([mid_span(n-1);R],[dP_p_small(n-1,:);zeros(1,azm)])))./azm;

%% robot.rotor/propeller coefficients
% robot.rotor force coefficients averaged over full rotation, robot.rotor convention
CT          =       T./(robot.flow.rho.*(pi*R^2)*(omega*R)^2);     % Thrust, robot.rotor
CQ          =       Q./(robot.flow.rho.*(pi*R^2)*(omega*R)^2*R);   % Torque, robot.rotor
CP          =       P./(robot.flow.rho.*(pi*R^2)*(omega*R)^3);     % Power, robot.rotor
CNx         =       Nx./(robot.flow.rho.*(pi*R^2)*(omega*R)^2);    % Normal longitudinal force x-dir, robot.rotor
CNy         =       Ny./(robot.flow.rho.*(pi*R^2)*(omega*R)^2);    % Side lateral force y-dir, robot.rotor
CMx         =       Mx./(robot.flow.rho.*(pi*R^2)*(omega*R)^2*R);  % Rolling moment, robot.rotor
CMy         =       My./(robot.flow.rho.*(pi*R^2)*(omega*R)^2*R);  % Pitching moment, robot.rotor

robot.aero.Kthrust = CT*robot.flow.rho*pi*R^4;
robot.aero.Ktorque = CP*robot.flow.rho*pi*R^5;

% Blade loading coefficients averaged over full rotation
CT_sigma    =      (CT)./sigma_g;                       % Solidity weighted thrust (blade loading), robot.rotor
CQ_sigma    =      (CQ)./sigma_g;                       % Solidity weighted Torque, robot.rotor
CP_sigma    =      (CP)./sigma_g;                       % Solidity weighted Power, robot.rotor
CNx_sigma   =      (CNx)./sigma_g;                      % Solidity weighted Normal x force, robot.rotor
CNy_sigma   =      (CNy)./sigma_g;                      % Solidity weighted Normal y force, robot.rotor
CMx_sigma   =      (CMx)./sigma_g;                      % Solidity weighted Normal x moment, robot.rotor
CMy_sigma   =      (CMy)./sigma_g;                      % Solidity weighted Side y moment, robot.rotor

% robot.rotor Figure of Merit
FM          =      (CT^(3/2)/(sqrt(2)*CP));                  % Figure of Merit

% Propeller force coefficients averaged over full rotation, propeller convention
Ctp         =      T./(robot.flow.rho.*rps^2.*dia^4);         % Thrust, propeller
Cqp         =      Q./(robot.flow.rho.*rps^2.*dia^5);         % Torque, propeller
Cpp         =      P./(robot.flow.rho.*rps^3.*dia^5);         % Power, propeller
Cnxp        =      Nx./(robot.flow.rho.*rps^2.*dia^4);        % Normal x force, propeller
Cnyp        =      Ny./(robot.flow.rho.*rps^2.*dia^4);        % Normal y force, propeller
Cnsump      =      Nsum./(robot.flow.rho.*rps^2.*dia^4);      % Normal force sum, propeller