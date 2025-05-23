function f=simfun(k)
assignin('base','rol_pit_pos_c',k(1))%%30
assignin('base','yaw_pos_c',k(2))%%1
 
assignin('base','rol_pit_at_cp',k(3))%%0.3
assignin('base','rol_pit_at_ci',k(4))%%1.4
assignin('base','rol_pit_at_cd',k(5))%%0.01

assignin('base','yaw_at_cp',k(6))%%20
assignin('base','yaw_at_cd',k(7))%%0.1

[t,x,simout]=sim('drone_attitude_control_nogear_nopos_fuzzy_pso.slx');
f=simout(end,1);