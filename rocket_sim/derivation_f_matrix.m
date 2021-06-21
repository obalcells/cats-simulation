syms q_est [4, 1];
syms delta_q [4,1];
syms acc [4, 1];
syms b_acc eta_acc;
q_true = quat_mult(q_est, delta_q);
v_dot_dash = quat_mult(quat_mult((q_true), acc), conjugate_q(q_true));
v_dot = v_dot_dash(4) - b_acc + eta_acc;
v_dot = simplify(v_dot);
diff_1 = diff(v_dot, delta_q2);
diff_2 = diff(v_dot, delta_q3);
diff_3 = diff(v_dot, delta_q4);
diff(v_dot, b_acc);
assume(delta_q1 == 1);
assume(delta_q2 == 0);
assume(delta_q3 == 0);
assume(delta_q4 == 0);
simplify(diff_3)