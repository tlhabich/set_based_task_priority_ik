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
t26 = t10 * t16;
t28 = t15 * t11;
t42 = t21 * t2;
t43 = t42 * t4;
t44 = t21 * t6;
t45 = t44 * t8;
t46 = t43 - t45;
t50 = -t4 * t44 - t42 * t8;
t52 = t11 * t46 + t16 * t50;
t57 = t46 * t16;
t59 = t50 * t11;
t73 = t6 * t4;
t75 = t2 * t8;
t77 = 0.1e1 * t73 + 0.1e1 * t75;
t83 = -0.1e1 * t6 * t8 + 0.1e1 * t2 * t4;
t85 = t11 * t77 + t16 * t83;
t87 = t77 * t16;
t89 = t83 * t11;
unknown(1,1) = t18 * t19 + 0.1e1 * t21 * t22;
unknown(1,2) = -0.1e1 * t26 + 0.1e1 * t28;
unknown(1,3) = -t18 * t22 + 0.1e1 * t21 * t19;
unknown(1,4) = 0.95e-1 * t26 - 0.95e-1 * t28 + 0.109e0 * t21 - 0.392e0 * t5 + 0.392e0 * t9 - 0.425e0 * t3;
unknown(2,1) = t52 * t19 - 0.1e1 * t1 * t22;
unknown(2,2) = -0.1e1 * t57 + 0.1e1 * t59;
unknown(2,3) = -t52 * t22 - 0.1e1 * t1 * t19;
unknown(2,4) = 0.95e-1 * t57 - 0.95e-1 * t59 - 0.109e0 * t1 - 0.392e0 * t43 + 0.392e0 * t45 - 0.425e0 * t42;
unknown(3,1) = t85 * t19;
unknown(3,2) = -0.1e1 * t87 + 0.1e1 * t89;
unknown(3,3) = -t85 * t22;
unknown(3,4) = 0.89e-1 + 0.95e-1 * t87 - 0.95e-1 * t89 - 0.392e0 * t73 - 0.392e0 * t75 - 0.425e0 * t6;
unknown(4,1) = 0.0e0;
unknown(4,2) = 0.0e0;
unknown(4,3) = 0.0e0;
unknown(4,4) = 0.1e1;
