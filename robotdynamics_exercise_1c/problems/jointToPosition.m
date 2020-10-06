function I_r_IE = jointToPosition(q)
  % Input: joint angles
  % Output: position of end-effector w.r.t. inertial frame. I_r_IE
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  I_r_IE = zeros(3,1);
  % Position of E in the frame of E
  E_r_EE = zeros(3,1);
  % TF matrix between adjacent joints
  TI0 = getTransformI0();
  T01 = jointToTransform01(q);
  T12 = jointToTransform12(q);
  T23 = jointToTransform23(q);
  T34 = jointToTransform34(q);
  T45 = jointToTransform45(q);
  T56 = jointToTransform56(q);
  T6E = getTransform6E();
  TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
  p_IE = TIE * [E_r_EE; 1];
  I_r_IE = p_IE(1:3);
end