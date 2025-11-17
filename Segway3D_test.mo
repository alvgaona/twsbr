model Segway3D_test
Segway_3D seg(d = 0.59, l = 0.14, r = 0.2, mBody = 41, mWheel = 2, c_alpha = 0.01, theta(start = 0*Modelica.Constants.pi/180));

input Real TL(start= 10.0), TR(start= -10.0);

output Real x, x_vel;
output Real pitch, pitch_vel;
output Real yaw, yaw_vel;

equation
  seg.TL = TL;
  seg.TR = TR;
  
  x = seg.x;
  x_vel = seg.x_velocity;
  
  pitch = seg.theta * 180 / Modelica.Constants.pi;
  pitch_vel = seg.theta_velocity * 180 / Modelica.Constants.pi;
  
  yaw = seg.psi * 180 / Modelica.Constants.pi;
  yaw_vel = seg.psi_velocity * 180 / Modelica.Constants.pi;
  
end Segway3D_test;
