function ddq = accODE(t, q, M, F_dyn, Cq_fun_dyn, g_hat)
ddq=[q(13:24); inv(M)*F_dyn(q(1:12))+inv(M)*Cq_fun_dyn(t, q(1:12))'*inv((Cq_fun_dyn(t, q(1:12))*inv(M)*Cq_fun_dyn(t, q(1:12))'))*(g_hat(t, q(1:12), q(13:24))-Cq_fun_dyn(t, q(1:12))*inv(M)*F_dyn(q(1:12)))];
end