block Segway2D_test
  Segway_2D seg(l = 0.14, r = 0.2, mBody = 41, mWheel = 2, c_alpha = 0.01, theta(start = 5*Modelica.Constants.pi/180));
  
  Modelica.Blocks.Interfaces.RealInput TL "Control torques"
    annotation(Placement(transformation(extent={{-120,20},{-100,40}})));
  Modelica.Blocks.Interfaces.RealInput TR 
    annotation(Placement(transformation(extent={{-120,-40},{-100,-20}})));
  
  Modelica.Blocks.Interfaces.RealOutput x, x_vel;
  Modelica.Blocks.Interfaces.RealOutput pitch, pitch_vel;
  
equation
  seg.TL = TL;
  seg.TR = TR;
  
  x = seg.x;
  x_vel = seg.x_velocity;
  
  pitch = seg.theta * 180 / Modelica.Constants.pi;
  pitch_vel = seg.theta_velocity * 180 / Modelica.Constants.pi;
  
  
  annotation(
  Icon(graphics = {
    Rectangle(extent={{-80,80},{80,-80}}, lineColor={0,0,255}),
    Text(extent={{-70,20},{70,-20}}, textString="Segway")
  })
);
end Segway2D_test;
