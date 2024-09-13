classdef memsIMU < handle
  properties
    previousMarkovBias = [0; 0; 0]
    nullShift

    tau
    sigmaWhite
    sigmaGM
		inRunBias = [0; 0; 0]
  end
  methods
    function obj = memsIMU(tau, sigmaWhite, sigmaGM)
      % Initializes the Gyroscope object with the model parameters.
      % Args:
      %   tau (3x1): Gauss-Markov process time constant [s]
      %   sigmaWhite (3x1): White noise process deviation [rad/s/sqrt(Hz)]
      %   sigmaGM (3x1): Gauss-Markov process deviation [rad/s]
      
      obj.tau = tau;
      obj.sigmaWhite = sigmaWhite;
      obj.sigmaGM = sigmaGM;

      % Setting this to zero for now
      % TODO: Select this randomly
      obj.nullShift = [0; 0; 0];
    end

    function output = GetMeasurement(obj, truth, dt)
      % The output of the accel/gyro is modeled as
      %   output = truth + nullShift + inRunBias + noise
      whiteNoise = randn(3, 1).*obj.sigmaWhite;
      obj.inRunBias = CalculateInRun(obj, dt);

      output = truth + obj.nullShift + whiteNoise + obj.inRunBias;
    end

    function bias = CalculateInRun(obj, dt)
      % The stochastic DiffEq for FOGM process is 
      %   x_dot = -1/tau * x + w 
      % w is the driving noise with PSD Q = 2*sigma^2/tau

      % Continuous state space model 
      Ac = diag(-1./obj.tau); Bc = eye(3); Cc = eye(3); Dc = 0;
      Qc = diag(2*obj.sigmaGM.^2./obj.tau);

      % Discretize the noise
      Qd = discrete_Q(Ac, Bc, dt, Qc);

      % Discretize the state space model (only need A) 
      Ad = c2d(ss(Ac, Bc, Cc, Dc), dt).A;

      w = sqrt(Qd)*randn(3, 1);
      bias = Ad*obj.previousMarkovBias + w;
      obj.previousMarkovBias = bias;
    end
  end
end
