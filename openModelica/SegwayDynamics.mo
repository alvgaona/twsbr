model SegwayDynamics "Core dynamics equations"

  parameter Real d = 0.59 "Distance between wheels";
  parameter Real l = 0.14 "Pendulum length";
  parameter Real r = 0.2 "Wheel radius";
  parameter Real mb = 41 "Body mass";
  parameter Real mw = 2 "Wheel mass";
  parameter Real J = 0.04 "Wheel inertia";
  parameter Real K_inertia = 0.02 "Vertical axis inertia";
  parameter Real I1 = 1 "Body inertia 1";
  parameter Real I2 = 1 "Body inertia 2";
  parameter Real I3 = 1 "Body inertia 3";
  parameter Real calpha = 0.01 "Friction coefficient";
  parameter Real g = 9.81 "Gravity";
  
  // Inputs
  Modelica.Blocks.Interfaces.RealInput u[2] "Control torques"
    annotation(Placement(transformation(extent={{-120,-10},{-100,10}})));
  
  Modelica.Blocks.Interfaces.RealInput disturbance[6](start=fill(0.0, 6)) "Disturbance for [x,theta,psi,dx,dtheta,dpsi]"
    annotation(Placement(transformation(extent={{20,120},{0,100}})));
  
  // Outputs
  Modelica.Blocks.Interfaces.RealOutput state[6] "State vector [x,theta,psi,dx,dtheta,dpsi]" 
    annotation(Placement(transformation(extent={{100,-10},{120,10}})));
      
  // State variables
  Real x(start=0) "Forward position";
  Real theta(start=0) "Pitch angle";
  Real psi(start=0) "Yaw angle";
  Real dx(start=0) "Forward velocity";
  Real dtheta(start=0) "Pitch velocity";
  Real dpsi(start=0) "Yaw velocity";
  Real q_dot[3];
  
  // Dynamic matrices
  Real M[3,3], C[3,3], D[3,3], B[3,2], G[3];
  Real a11, a12, a21, a22, a33;
  Real c12, c13, c23, c31, c32, c33;
  Real d11, d12, d21, d22, d33;
  Real b11, b12, b21, b22, b31, b32, g2;
  
equation
  // State derivatives
  q_dot[1] = dx;
  q_dot[2] = dtheta;
  q_dot[3] = dpsi;
  
  der(x) = dx + disturbance[1];
  der(theta) = dtheta + disturbance[2];
  der(psi) = dpsi + disturbance[3];

  
  // Output state vector
  state = {x, theta, psi, dx, dtheta, dpsi};
  
  // Mass matrix M
  a11 = mb + 2*mw + 2*J/r^2;
  a12 = mb*l*cos(theta);
  a21 = a12;
  a22 = I2 + mb*l^2;
  a33 = I3 + 2*K_inertia + mw*d^2/2 + J*d^2/(2*r^2) - (I3-I1-mb*l^2)*(sin(theta))^2;
  M = [a11, a12, 0; a21, a22, 0; 0, 0, a33];
  
  // Coriolis matrix C
  c12 = -mb*l*dtheta*sin(theta);
  c13 = -mb*l*dpsi*sin(theta);
  c23 = (I3-I1-mb*l^2)*dpsi*sin(theta)*cos(theta);
  c31 = mb*l*dpsi*sin(theta);
  c32 = -(I3-I1-mb*l^2)*dpsi*sin(theta)*cos(theta);
  c33 = -(I3-I1-mb*l^2)*dtheta*sin(theta)*cos(theta);
  C = [0, c12, c13; 0, 0, c23; c31, c32, c33];
  
  // Damping matrix D
  d11 = 2*calpha/r^2;
  d12 = -2*calpha/r;
  d21 = d12;
  d22 = 2*calpha;
  d33 = calpha*d^2/(2*r^2);
  D = [d11, d12, 0; d21, d22, 0; 0, 0, d33];
  
  // Input matrix B
  b11 = 1/r;
  b12 = b11;
  b21 = -1;
  b22 = b21;
  b31 = -d/(2*r);
  b32 = -b31;
  B = [b11, b12; b21, b22; b31, b32];
  
  // Gravity vector G
  g2 = -mb*l*g*sin(theta);
  G = {0, g2, 0};
  
  // Equations of motion: M*qddot + (C+D)*qdot + G = B*u + disturbance_forces
  M*der(q_dot) = B*u - (C+D)*q_dot - G + {disturbance[4], disturbance[5], disturbance[6]};
  

  
  annotation(Icon(graphics={
    Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0}, fillColor={200,200,200}, fillPattern=FillPattern.Solid),
    Text(extent={{-60,40},{60,-40}}, textString="Dynamics"),
    Text(extent={{-150,150},{150,110}}, textString="%name")}));
end SegwayDynamics;
