%%------------------------------------------------------------------------ 
%  u IMU data vector (6 x l)
%  y position measurement
%%------------------------------------------------------------------------



flightData = table2array(readtable("flightData.csv"));
flightData(1,:) = []; % Remove first row

t = flightData(:,1)*1e-06;  % Time [sec]
u = flightData(:,2:7)';  % IMU data [fx; fy; fz; gx; gy; gz] [m/s^2, rad/s]
y = flightData(:,8:10)';  % Position measurements [m]
tow = flightData(:,11);
mocapPos = flightData(:, 12:14)'; % ground truth position [m]

run('./constants.m');
const.tau_a = tau_a;
const.tau_g = tau_g;

% get the number of IMU measurement samples
l = size(u,2);

% state vector dimension
% [pos(3x1) vel(3x1) eul(3x1) biasg(3x1) biasw(3x1)]
N = 15;

%--------------------Initialize Covariance Matrix-------------------------
P = zeros(N,N);
Q = zeros(12,12);
P(1:3,1:3) = diag( sigma_initial_pos.^2 );
P(4:6,4:6) = diag( sigma_initial_vel.^2 );
P(7:9,7:9) = diag( sigma_initial_att.^2 );
P(10:12,10:12) = diag(sigma_initial_acc_bias.^2);
P(13:15,13:15) = diag(sigma_initial_gyro_bias.^2);

Q(1:3,1:3) = diag( sigma_acc.^2 );
Q(4:6,4:6) = diag( sigma_gyro.^2 );
Q(7:9,7:9) = diag( 2*acc_bias_driving_noise.^2./tau_a );
Q(10:12,10:12) = diag( 2*gyro_bias_driving_noise.^2./tau_g );

H = zeros(3, N);
H(1:3,1:3) = eye(3);

R = diag( sigma_pos.^2 );

%---------------------------placeholders----------------------------------
x_h = zeros(N,l);
cov = zeros(N,l);

%---------------------------Initialization--------------------------------
attitude = [ini_roll ini_pitch ini_yaw]';
Cb2n = Rt2b(attitude)'; 
quat = dcm2q(Cb2n);     
x_h(1:3,1) = y(:,1);
x_h(7:9,1) = attitude;

%--------------------------Run the Filter---------------------------------
for k = 2:l

    % compensate IMU measurements
    u_h = u(:,k) + x_h(10:15,k-1);

    % propagate navigation state vectors
    [x_h(:,k), quat]=Navigation_equations(x_h(:,k-1),u_h,quat,Ts);

    % update state transition matrix
    [Fc, Gc]=state_matrix1(quat,u_h,Ts,const);

    % discretize Fc and Gc
    % F = expm(Fc*Ts);
    % Q_k = discrete_Q(Fc,Gc,Ts,Q);
    F = eye(15) + Ts*Fc;
    Q_k = F*Ts*Gc*Q*Gc';
    Q_k = (Q_k+Q_k')/2;

    % update the filter state covariance matrix
    P=F*P*F'+ Q_k;  
    P=(P+P')/2;     % Forces symmetry

    cov(:,k)=diag(P);

    if  (tow(k) - tow(k-1)) > 0 
        S = H*P*H' + R;
        K=(P*H')/S; 

        z=y(:,k)-x_h(1:3,k); % Measurement error

        dx=K*z; % state error estimate

        [x_h(:,k), quat]=comp_internal_states(x_h(:,k),dx,quat);

        P=(eye(15)-K*H)*P;
        I_KH = eye(15) - K*H;
        P = I_KH*P*I_KH' + K*R*K';

        P=(P+P')/2;

        cov(:,k)=diag(P);
    end

end