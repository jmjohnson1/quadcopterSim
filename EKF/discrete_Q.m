function Q = discrete_Q(F,G,dt,Qc)

%=========================================================================%
%               Q = = discrete_Q(F,G,dt,Qc)
%
%   This function evaluates the integral in Equation (4.27) of Simon.  The 
%   algorithm used is based on the algorithm by Van Loan described in the 
%   following paper:
%
%   Van Loan, C. F., "Computing Integrals Involving the Matrix 
%   Exponential," IEEE Transaction of Automatic Control, Vol 23, No. 3.  
%   June 1978.
%
%=========================================================================%

Z = zeros(size(F));
[n,~]=size(F);
M = [-F G*Qc*G';Z F'];
L = expm(M*dt);
L12 = L(1:n,n+1:2*n);
L22 = L(n+1:2*n,n+1:2*n);
Q = L22'*L12;
