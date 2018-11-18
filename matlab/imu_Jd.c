  /*
  V1.2
  */
  t2 = 3.141592653589793*(1.0/2.0);
  t3 = q1+t2;
  t4 = q2-t2;
  t5 = sin(t4);
  t6 = sin(t3);
  t7 = q3-1.23E2/2.5E2;
  t8 = cos(t3);
  t9 = cos(t4);
  A0[0][0] = -qd3*t5*t6-qd1*t5*t7*t8-qd2*t6*t7*t9;
  A0[0][1] = qd3*t8*t9-qd2*t5*t7*t8-qd1*t6*t7*t9;
  A0[0][2] = -qd1*t5*t6+qd2*t8*t9;
  A0[1][1] = qd3*t5+qd2*t7*t9;
  A0[1][2] = qd2*t5;
  A0[2][0] = qd3*t5*t8-qd1*t5*t6*t7+qd2*t7*t8*t9;
  A0[2][1] = qd3*t6*t9-qd2*t5*t6*t7+qd1*t7*t8*t9;
  A0[2][2] = qd1*t5*t8+qd2*t6*t9;
