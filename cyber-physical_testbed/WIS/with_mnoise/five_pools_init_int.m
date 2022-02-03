%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Gabriel de Albuquerque Gleizer %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y0 = 0.05;  % Deviation to set-point, in meters

%% Model parameters
tau = [4, 2, 4, 4, 6];  %[min], Li and de Schutter (2011)
alpha = [6492 2478 6084 5658 7650];  % [m^2], idem
phi = [0.48 1.05 0.48 0.48 0.42];  % [rad/min], idem

% This parameter was inferred from Weyer et al (2001).
zeta = 0.0151;  % no unit

% Note: gates operate in the range of 100's of Ml/day
% 1Ml/day = 1000 m3/ 1440 min ~= 0.64 m3/min
% Thus, control actions should range about 50-100 m3/min

% State-space model
n = 10;  % Number of states (sensors)
m = 5;  % Number of inputs (actuators)
A = zeros(n,n);
B = zeros(n,n);  % 1-5: inputs; 6-10: disturbances
C = eye(n);
D = zeros(n,n);

% Building blocks
% x1' = -2/tau x1 + x2 - 1/alpha (u + v + d)
% x2' = 2/tau/alpha (u - v - d)
Atau = @(tau) [-2/tau, 1; 0, 0]; 
Bu = @(tau, alpha) [-1/alpha; 2/tau/alpha];
Bvd = @(tau, alpha) [-1/alpha; -2/tau/alpha];

% Build matrices
for pool = 1:5
    Ap = Atau(tau(pool));
    Bpu = Bu(tau(pool), alpha(pool));
    Bpv = Bvd(tau(pool), alpha(pool));
    Bpd = Bvd(tau(pool), alpha(pool));
    
    A(2*pool-1:2*pool,2*pool-1:2*pool) = Ap;
    B(2*pool-1:2*pool,pool) = Bpu;
    if pool < 5
        B(2*pool-1:2*pool,pool+1) = Bpv;
    end
    B(2*pool-1:2*pool,pool+5) = Bpd;
end

x0 = y0*ones(1,10);

% for initial condition, x2 = y' + 2/tau*x1 -> x2(0) = 2/tau*x1(0)
x0(2:2:10) = 2./tau*y0;

%% For "practical implementation", we need a measurable x2
% x1' = -2/tau x1 + x2 - 1/alpha (u + v + d)
% x2' = 2/tau/alpha (u - v - d)
%--------------------------------------
% 2 x1' - tau x2' = -4/tau x1 + 2 x2 - 4/alpha u
% z = - 2 x1 + tau x2 --> z' = -2/tau z  +4/alpha u <--- filter on u
% x1' = -z/tau - 1/alpha (u + v + d)
%
% Tinvz' = ATinvz + Bu <==> z' = TATinvz + TBu
Tp = @(tau) [1 0; -2, tau];
T = zeros(10,10);  % Similarity transformation matrix
for pool = 1:5
    T(2*pool-1:2*pool,2*pool-1:2*pool) = Tp(tau(pool));
end
A = T*A/T;
B = T*B;
x0 = T*x0(:);

%% Add integrators to the system
% for synthesis; this doesn't go to Simulink
AA = [A zeros(10,5); zeros(5, 15)];
for pool = 1:5
    AA(10+pool, 1+2*(pool-1)) = 1;
end
BB = [B; zeros(5,10)]; 

%% Control design
% x0 = 0.1*x0;  % Assume small perturbations...
% y0 = 0.1*y0;
BU = BB(:,1:5);

% LQR...
kx1 = 100;
kx2 = 1;
% Q = 100*diag([1.5, 100, 1.5, 30, 20, 80, 20, 80, 30, 160]);  % old
% Q = 100*diag([1, 0.01, 1, 0.01, 10, 0.1, 10, 0.1, 20, 0.2]); %pd
Q = 250*diag([2*0.5*0.5, 0, 1*0.5, 0, 2*0.5, 0, 2, 0, 3, 0,...
    0.001*[2*0.5*0.5, 1*0.5, 2*0.5, 2, 3]]); 
R = 0.1*eye(5);
[K,~,eigs_] = lqr(AA,BU,Q,R);

%% Triggering matrices
load('safe_sparse_trig_petc');
% For decentralized... compute sigmas
Dx = full(Dx);
De = full(De);
sigma = diag(Dx)./diag(De);

%% for TCP communication
h = 1;  % min
SAMPLE_TIME = 60*h;  % seconds
TS = 60;  % ms, relative to the beginning of the epoch
TE = 10;  % ms, relative to the beginning of the epoch

%% for fixed point treatment
x_range = 2;
u_range = 4000;
theta_range = 32;

N_bits = 16;
x_decimal_points = N_bits - ceil(log2(x_range)) - 1;
u_decimal_points = N_bits - ceil(log2(u_range)) - 1;
theta_decimal_points = N_bits - ceil(log2(theta_range)) - 1;

%% Save data for Python
% save('Python/data.mat','K','nT','Qt','Minv','n','epsilon','De','Dx',...
%      'Phit','SAMPLE_TIME','Etilde','times');