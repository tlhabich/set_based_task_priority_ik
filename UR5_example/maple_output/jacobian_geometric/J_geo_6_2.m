t1 = sin(q(1));
t2 = cos(q(2));
t5 = cos(q(1));
t6 = sin(q(2));
t13 = t1 ^ 2;
t16 = t5 ^ 2;
unknown(1,1) = 0.425e0 * t1 * t2;
unknown(1,2) = 0.425e0 * t5 * t6;
unknown(2,1) = -0.425e0 * t5 * t2;
unknown(2,2) = 0.425e0 * t1 * t6;
unknown(3,1) = 0.0e0;
unknown(3,2) = -0.425e0 * t13 * t2 - 0.425e0 * t16 * t2;
unknown(4,1) = 0.0e0;
unknown(4,2) = t1;
unknown(5,1) = 0.0e0;
unknown(5,2) = -t5;
unknown(6,1) = 0.1e1;
unknown(6,2) = 0.0e0;
