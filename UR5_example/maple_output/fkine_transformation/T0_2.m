t1 = cos(q(1));
t2 = cos(q(2));
t3 = t1 * t2;
t4 = sin(q(2));
t6 = sin(q(1));
t9 = t6 * t2;
unknown(1,1) = t3;
unknown(1,2) = -t1 * t4;
unknown(1,3) = 0.1e1 * t6;
unknown(1,4) = -0.425e0 * t3;
unknown(2,1) = t9;
unknown(2,2) = -t6 * t4;
unknown(2,3) = -0.1e1 * t1;
unknown(2,4) = -0.425e0 * t9;
unknown(3,1) = 0.1e1 * t4;
unknown(3,2) = 0.1e1 * t2;
unknown(3,3) = 0.0e0;
unknown(3,4) = 0.89e-1 - 0.425e0 * t4;
unknown(4,1) = 0.0e0;
unknown(4,2) = 0.0e0;
unknown(4,3) = 0.0e0;
unknown(4,4) = 0.1e1;
