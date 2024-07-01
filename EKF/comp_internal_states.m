function [x_out, q_out]=comp_internal_states(x_in,dx,q_in)

  % Correct the state vector
  x_out = x_in + dx;

  % Correct the attitude
  dq = [1; 0.5*dx(7:9)];
  q_out = quatmultiply(q_in', dq')';
  % Normalize
  q_out = q_out./norm(q_out);

end
