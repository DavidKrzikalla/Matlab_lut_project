function C_r_dtt = revolute_joint_dtt(i, j, s_i, s_j, q,dq)

idx_i = body_idx(i);
dphi_i = dq(idx_i(3));
phi_i = q(idx_i(3));
idx_j = body_idx(j);
dphi_j = dq(idx_j(3));
phi_j = q(idx_j(3));

C_r_dtt = rot(phi_i) * s_i * dphi_i.^2 - rot(phi_j) * s_j * dphi_j.^2;
