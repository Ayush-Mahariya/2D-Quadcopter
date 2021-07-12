function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% FILL IN YOUR CODE HERE

% Constants
m = params.mass;
Ixx = params.Ixx;
g = params.gravity;

% PID values
Kdz = 10;       Kpz = 50;
Kdphi = 20;     Kpphi = 80;
Kdy = 10;       Kpy = 50;

y = state.pos(1); z = state.pos(2);
y_dot = state.vel(1); z_dot = state.vel(2);
phi = state.rot; phi_dot = state.omega;

y_des = des_state.pos(1); z_des = des_state.pos(2);
y_dot_des = des_state.vel(1); z_dot_des = des_state.vel(2);
y_ddot_des = des_state.acc(1); z_ddot_des = des_state.acc(2);

phi_c = -(y_ddot_des + Kdy*(y_dot_des - y_dot) + Kpy*(y_des - y))/g;
phi_c_dot = -(Kdy*(y_ddot_des + g*phi) + Kpy*(y_dot_des - y_dot))/g;
u1 = m*(g + z_ddot_des + Kdz*(z_dot_des - z_dot) + Kpz*(z_des - z));
u2 = Ixx*( Kdphi*(phi_c_dot - phi_dot) + Kpphi*(phi_c - phi));
 
end

