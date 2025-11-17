model SegwaySystem "Complete Segway with block diagram structure"
  SegwayDynamics plant(d = 0.59, l = 0.14, r = 0.2, mb = 41, mw = 2, J = 0.04, K_inertia = 0.02, I1 = 1, I2 = 1, I3 = 1, calpha = 0.01, g = 9.81, theta(start = 0.5), x(start = 0), psi(start = 3.1415)) annotation(
    Placement(transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}})));
  LQRController lQRController(max_torque = 20) annotation(
    Placement(transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Noise.NormalNoise ruidoSensor[6](samplePeriod = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01}, sigma = {0.01, 0.005, 0.005, 0.02, 0.01, 0.01}, mu = {0, 0, 0, 0, 0, 0}) annotation(
    Placement(transformation(origin = {30, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add[6]annotation(
    Placement(transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Pulse perturbaciones[6](amplitude = {0, 0, 0, 0, 0, 0}, width = {50, 50, 50, 50, 50, 50}, period = {0.2, 0.1, 0.1, 0.1, 0.1, 0.1}, nperiod = {1, 0, 0, 0, 0, 0}, offset = {0, 0, 0, 0, 0, 0}, startTime = {10, 0, 0, 0, 0, 0}) annotation(
    Placement(transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Sine sine[6](amplitude = {0, 0, 0, 0, 0, 0}, f = {0.05, 0, 0, 0.05, 0, 0}, phase = {0, 0, 0, 1.5707963267948966, 0, 0}, offset = {0, 0, 0, 0, 0, 0}, startTime = {0, 0, 0, 0, 0, 0}) annotation(
    Placement(transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Step setpoints[6](height = {1, 0, 0, 0, 0, 0}) annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}})));
  DriftingVectorNoise driftingVectorNoise(n = 6, sigma_d = {0.005, 0.003, 0.003, 0.01, 0.005, 0.005})  annotation(
    Placement(transformation(origin = {-4, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
equation
  connect(ruidoSensor.y, add.u1) annotation(
    Line(points = {{42, -70}, {52, -70}, {52, -36}, {42, -36}}, color = {0, 0, 127}, thickness = 0.5));
  connect(lQRController.torque, plant.u) annotation(
    Line(points = {{2, 10}, {20, 10}}, color = {0, 0, 127}, thickness = 0.5));
  connect(plant.state, add.u2) annotation(
    Line(points = {{42, 10}, {70, 10}, {70, -24}, {42, -24}}, color = {0, 0, 127}, thickness = 0.5));
  connect(perturbaciones.y, plant.disturbance) annotation(
    Line(points = {{22, 50}, {32, 50}, {32, 22}}, color = {0, 0, 127}, thickness = 0.5));
  connect(sine.y, lQRController.state_setpoint) annotation(
    Line(points = {{-59, -10}, {-44, -10}, {-44, 13}, {-21, 13}}, color = {0, 0, 127}, thickness = 0.5));
  connect(lQRController.state, driftingVectorNoise.y) annotation(
    Line(points = {{-20, 8}, {-30, 8}, {-30, -30}, {-14, -30}}, color = {0, 0, 127}, thickness = 0.5));
  connect(driftingVectorNoise.u,add.y) annotation(
    Line(points = {{8, -30}, {20, -30}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {170, 255, 170}, fillPattern = FillPattern.Solid), Text(extent = {{-80, 40}, {80, -40}}, textString = "Segway"), Text(extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end SegwaySystem;