clc;

% Symbolic variables
syms x1 x2 x3 u s tt II AA

%% Equilibrium Points (4)

% Constants
R = 50;      % Resistance
L = 0.2;     % Inductance
g = 9.8;     % Gravitational acceleration
M = 0.491;   % Mass
c = 0.3;     % Damping coefficient
fv = 0.04;   % Friction coefficient

% Equilibrium conditions
eq1 = x2 == 0; 
eq2 = x1 == 0.06;
eq3 = -g + (c/M)*(x3^2/(0.1 - x1)) - (fv*x2/M) == 0;
eq4 = (1/L)*(-R*x3 + u) == 0;

% Solve equilibrium equations
stateEqs = [eq1, eq2, eq3, eq4];
sol = solve(stateEqs);

% Equilibrium point solutions
xe1 = sol.x1;
xe2 = sol.x2;
xe3 = sol.x3;

%% Linearization (5)

% Define system dynamics
f = x2;
g = -g + (c/M)*(x3^2/(0.1 - x1)) - (fv*x2/M);
h = (1/L)*(-R*x3 + u);

% Compute Jacobian matrix
A = jacobian([f, g, h], [x1, x2, x3]);
A

% Linearize the system at equilibrium points
A_1 = subs(A, {x1, x2, x3}, {xe1(1), xe2(1), xe3(1)});
A_2 = subs(A, {x1, x2, x3}, {xe1(2), xe2(2), xe3(2)});
A_1
A_2

% Compute eigenvalues
eigA1 = eig(A_1);
eigA2 = eig(A_2);
eigA1
eigA2
%% State-Space Analysis (6)

% Controllability Analysis
% Using ctrb(A, B) or manual calculations
B = [0; 0; 5];  % Input vector

% Compute controllability matrices
A_1B = A_1 * B;
A_2B = A_2 * B;
A_12B = A_1^2 * B;
A_22B = A_2^2 * B;

% Controllability matrices
Phi_c1 = [B A_1B A_12B];
Phi_c2 = [B A_2B A_22B];

% Determinant of controllability matrices
det_Phi_c1 = det(Phi_c1);
det_Phi_c2 = det(Phi_c2);

% Observability Analysis
% Using obsv(A, C) or manual calculations
C = [1 0 0];  % Output vector

% Compute observability matrices
A_1C = C * A_1;
A_2C = C * A_2;
A_12C = C * A_1^2;
A_22C = C * A_2^2;

% Observability matrices
Phi_o1 = [C; A_1C; A_12C];
Phi_o2 = [C; A_2C; A_22C];

% Rank of observability matrices
rank_Phi_o1 = rank(Phi_o1);
rank_Phi_o2 = rank(Phi_o2);

%% State Transition Matrix

% Compute inverse Laplace transform of the state transition matrix
eA1t = ilaplace(inv((s*eye(size(A_1,1)) - A_1)));
eA2t = ilaplace(inv((s*eye(size(A_2,1)) - A_2)));

% Define X matrix for some system analysis (customized)
X = [1 -250 62500 exp(-250*tt); 1 -15.7 246.49 exp(-15.7*tt); 1 15.61 243.67 exp(15.61*tt); II AA AA^2 exp(AA*tt)];

%% Transfer Function

% Define transfer function
g1 = C * (inv((s*eye(size(A_1,1)) - A_1))) * B;
g2 = C * (inv((s*eye(size(A_2,1)) - A_2))) * B;
g1
g2
% Solve for poles of the system
poleq = 491*s^3 + 122790*s^2 - 110295*s - 30073750;
solve(poleq, s);

%% PID Controller

% Define transfer function for G1 and G2
s = tf('s');
G1 = -60073.7 / (491*s^3 + 122790*s^2 - 110295*s - 30073750);
G2 = 60073.7 / (491*s^3 + 122790*s^2 - 110295*s - 30073750);

% Plot step response for open-loop G1 and G2
t = 0:0.01:2;
figure;
step(G1, t);
title('Step Response Open Loop G1');

figure;
step(G2, t);
title('Step Response Open Loop G2');

% P Controller
Kp = 1600;
C = pid(Kp);

T1 = feedback(C*G1, 1);
T2 = feedback(C*G2, 1);

% Plot closed-loop step responses for P controller
figure;
step(T1, t);
title('Step Response Closed Loop Kp.G1');

figure;
step(T2, t);
title('Step Response Closed Loop Kp.G2');

% PD Controller
Kd = 245;
C = pid(Kp, 0, Kd);
T3 = feedback(C*G2, 1);

info_T3 = stepinfo(T3);
disp('Step Info for T3 (PD Controller):');
disp(info_T3);

% Plot closed-loop step response for PD controller
figure;
stepinfo(T3);
step(T3, t);
title('Step Response Closed Loop Kd.Kp.G2');

% PID Controller
Ki = 2600;
C = pid(Kp, Ki, Kd);
T4 = feedback(C*G2, 1);

info_T4 = stepinfo(T4);
disp('Step Info for T4 (PD Controller):');
disp(info_T4);

% Plot closed-loop step response for PID controller
figure;
stepinfo(T4);
step(T4, t);
title('Step Response Closed Loop Kd.Ki.Kp.G2');
