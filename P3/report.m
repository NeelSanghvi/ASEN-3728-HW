%% Programming Homework 3 - (Neel Sanghvi)

%% Task 1
% (Nothing additional is needed for this task besides passing the tests.)

runtests('testLonAeroForcesAndMoments.m')

%% Task 2
% (Nothing additional is needed for this task besides passing the tests.)

runtests('testAircraftDynamics.m')

%% Task 3

evaluate('neel.sanghvi@colorado.edu') % Change the email to your own

%% Task 4

x_trim = [0; 0; -1800; 0; 0.02780; 0; 20.99; 0; 0.5837; 0; 0; 0];
u_trim = [0.1079; 0; 0; 0.3182];
A_lon = estimateAlon(@aircraftDynamics, x_trim, u_trim, ttwistor);
[V,D] = eig(A_lon);

V_ph = V(:,4);
V_sp = V(:,1);

dx_ph = real(V_ph);
dx_sp = real(V_sp);

% Calculate eigenvectors and indicate which corresponds to phugoid and short period

%% Task 5

ss_lon = ss(A_lon, zeros(4,1), [0 0 0 1], [0]); % Change this to the desired state-space model
dx_ph = (deg2rad(10)/dx_ph(4))*dx_ph;
[linear_theta, linear_time] = initial(ss_lon, dx_ph, 50);

x_ph = x_trim + [0;0;0;0;dx_ph(4);0;dx_ph(1);0;dx_ph(2);0;dx_ph(3);0]; % Replace this with an initial state that excites the phugoid mode
[nonlinear_time, nonlinear_x] = ode45(@(t, x) aircraftDynamics(x, u_trim, ttwistor), [0, 50], x_ph);

figure(1)
plot(linear_time, linear_theta + x_trim(5), nonlinear_time, nonlinear_x(:,5))

%% Task 6

dx_sp = (deg2rad(10)/dx_sp(4))*dx_sp;
[linear_theta, linear_time] = initial(ss_lon, dx_sp, 10);

x_sp = x_trim + [0;0;0;0;dx_sp(4);0;dx_sp(1);0;dx_sp(2);0;dx_sp(3);0]; % Replace this with an initial state that excites the short period mode
[nonlinear_time, nonlinear_x] = ode45(@(t, x) aircraftDynamics(x, u_trim, ttwistor), [0, 10], x_sp);

figure(2);
hold on
legend on
plot(linear_time, linear_theta + x_trim(5),'DisplayName','Linear')
plot(nonlinear_time, nonlinear_x(:,5),'DisplayName','Non-linear')
hold off

%% Task 7
%
% There is a large deviation for the short period mode compared to the
% Phugoid mode. This is because of the really high damping ratio of our
% linear model and it can be clearly seen in the way the linear plot directly
% gets damped and flat vs the nonlinear model that has few decaying
% oscillations before getting back to equilibrium (this is shown better
% when the time is extended to 100). 