t1 = cos(q(1));
t2 = cos(q(2));
t3 = t1 * t2;
t4 = cos(q(3));
t5 = t3 * t4;
t6 = sin(q(2));
t7 = t1 * t6;
t8 = sin(q(3));
t9 = t7 * t8;
t10 = t5 - t9;
t11 = cos(q(4));
t15 = -t3 * t8 - t4 * t7;
t16 = sin(q(4));
t18 = t10 * t11 + t15 * t16;
t19 = cos(q(5));
t21 = sin(q(1));
t22 = sin(q(5));
t25 = t18 * t19 + 0.1e1 * t21 * t22;
t26 = cos(q(6));
t28 = t10 * t16;
t30 = t15 * t11;
t32 = -0.1e1 * t28 + 0.1e1 * t30;
t33 = sin(q(6));
t39 = t18 * t22;
t41 = t21 * t19;
t53 = t21 * t2;
t54 = t53 * t4;
t55 = t21 * t6;
t56 = t55 * t8;
t57 = t54 - t56;
t61 = -t55 * t4 - t53 * t8;
t63 = t57 * t11 + t61 * t16;
t67 = t63 * t19 - 0.1e1 * t1 * t22;
t69 = t57 * t16;
t71 = t61 * t11;
t73 = -0.1e1 * t69 + 0.1e1 * t71;
t79 = t63 * t22;
t81 = t1 * t19;
t93 = t6 * t4;
t95 = t2 * t8;
t97 = 0.1e1 * t93 + 0.1e1 * t95;
t103 = -0.1e1 * t6 * t8 + 0.1e1 * t2 * t4;
t105 = t103 * t16 + t97 * t11;
t106 = t105 * t19;
t108 = t97 * t16;
t110 = t103 * t11;
t112 = -0.1e1 * t108 + 0.1e1 * t110;
t118 = t105 * t22;
unknown(1,1) = t25 * t26 + t32 * t33;
unknown(1,2) = -t25 * t33 + t32 * t26;
unknown(1,3) = -0.1e1 * t39 + 0.1e1 * t41;
unknown(1,4) = -0.82e-1 * t39 + 0.82e-1 * t41 + 0.95e-1 * t28 - 0.95e-1 * t30 + 0.109e0 * t21 - 0.392e0 * t5 + 0.392e0 * t9 - 0.425e0 * t3;
unknown(2,1) = t67 * t26 + t73 * t33;
unknown(2,2) = t73 * t26 - t67 * t33;
unknown(2,3) = -0.1e1 * t79 - 0.1e1 * t81;
unknown(2,4) = -0.82e-1 * t79 - 0.82e-1 * t81 + 0.95e-1 * t69 - 0.95e-1 * t71 - 0.109e0 * t1 - 0.392e0 * t54 + 0.392e0 * t56 - 0.425e0 * t53;
unknown(3,1) = t106 * t26 + t112 * t33;
unknown(3,2) = -t106 * t33 + t112 * t26;
unknown(3,3) = -0.1e1 * t118;
unknown(3,4) = 0.89e-1 - 0.82e-1 * t118 + 0.95e-1 * t108 - 0.95e-1 * t110 - 0.392e0 * t93 - 0.392e0 * t95 - 0.425e0 * t6;
unknown(4,1) = 0.0e0;
unknown(4,2) = 0.0e0;
unknown(4,3) = 0.0e0;
unknown(4,4) = 0.1e1;
