function I_Je = I_Je_fun(in1)
%I_JE_FUN
%    I_JE = I_JE_FUN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    21-Oct-2020 10:33:08

phi1 = in1(1,:);
phi2 = in1(2,:);
phi3 = in1(3,:);
phi4 = in1(4,:);
phi5 = in1(5,:);
t2 = cos(phi1);
t3 = cos(phi2);
t4 = cos(phi4);
t5 = cos(phi5);
t6 = sin(phi1);
t7 = sin(phi2);
t8 = sin(phi4);
t9 = sin(phi5);
t10 = phi2+phi3;
t11 = cos(t10);
t12 = sin(t10);
t13 = t2.*t4;
t14 = t2.*t8;
t15 = t4.*t6;
t16 = t6.*t8;
t17 = -t6;
t18 = t3.*1.35e+2;
t19 = t7.*1.35e+2;
t20 = t11.*3.5e+1;
t21 = t12.*3.5e+1;
t22 = -t18;
t24 = t11.*1.51e+2;
t25 = t12.*1.51e+2;
t26 = t12.*t13;
t27 = t12.*t14;
t28 = t12.*t15;
t29 = t12.*t16;
t30 = t5.*t12.*3.6e+1;
t31 = t12.*(7.0./1.0e+2);
t33 = t4.*t9.*t11.*3.6e+1;
t35 = t11.*(1.51e+2./5.0e+2);
t37 = t5.*t11.*(9.0./1.25e+2);
t41 = t4.*t9.*t12.*(9.0./1.25e+2);
t23 = -t20;
t32 = -t31;
t34 = -t28;
t36 = -t35;
t38 = -t37;
t39 = t13+t29;
t40 = t16+t26;
t43 = t19+t21+t24;
t42 = t14+t34;
t44 = t23+t25+t30+t33;
t45 = t22+t44;
I_Je = reshape([t6.*t43.*(-1.0./5.0e+2)-t9.*t42.*(9.0./1.25e+2)-t5.*t6.*t11.*(9.0./1.25e+2),t2.*t37+(t2.*t43)./5.0e+2-t9.*t40.*(9.0./1.25e+2),0.0,0.0,0.0,1.0,t2.*t45.*(-1.0./5.0e+2),t6.*t45.*(-1.0./5.0e+2),t7.*(-2.7e+1./1.0e+2)+t32+t36+t38+t41,t17,t2,0.0,t2.*t44.*(-1.0./5.0e+2),t6.*t44.*(-1.0./5.0e+2),t32+t36+t38+t41,t17,t2,0.0,t9.*(t15-t27).*(-9.0./1.25e+2),t9.*t39.*(9.0./1.25e+2),t8.*t9.*t11.*(9.0./1.25e+2),t2.*t11,t6.*t11,-t12,t5.*t40.*(-9.0./1.25e+2)-t2.*t9.*t11.*(9.0./1.25e+2),t5.*t42.*(9.0./1.25e+2)-t6.*t9.*t11.*(9.0./1.25e+2),t9.*t12.*(9.0./1.25e+2)-t4.*t5.*t11.*(9.0./1.25e+2),-t15+t27,t39,t8.*t11,0.0,0.0,0.0,-t9.*t40+t2.*t5.*t11,t9.*t42+t5.*t6.*t11,-t5.*t12-t4.*t9.*t11],[6,6]);
