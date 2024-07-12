function [X, P, quat] = EKF(X, Y, u, P, constants, Ts, quat)

Q = constants.Q;
H = constants.H;
R = constants.R;

% compensate IMU measurements
u = u - X(10:15);

% propagate navigation state vectors
[X, quat] = Navigation_equations(X, u, quat, Ts);

% update state transition matrix
[Fc, Gc] = state_matrix1(quat, u, Ts, constants);

% discretize Fc and Gc
% F = expm(Fc*Ts);
% Q_k = discrete_Q(Fc,Gc,Ts,Q);
F = eye(15) + Ts*Fc;
Q_k = F*Ts*Gc*Q*Gc';
Q_k = (Q_k+Q_k')/2;

% update the filter state covariance matrix
P = F*P*F'+ Q_k;
P = (P+P')/2;     % Forces symmetry

if ~isempty(Y)
    S = H*P*H' + R;
    K = (P*H')/S; 
    z = Y - X(1:3); % Measurement error
    dx=K*z; % state error estimate

    [X, quat]=comp_internal_states(X, dx, quat);

    I_KH = eye(15) - K*H;
    P = I_KH*P*I_KH' + K*R*K';

    P=(P+P')/2;
end

end