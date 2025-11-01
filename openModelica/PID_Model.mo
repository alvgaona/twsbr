model PID_Model
  SegwayDynamics plant(I1 = 1.1, I2 = 0.9, I3 = 1.1, J = 0.06, K_inertia = 0.018, calpha = 0.011, d = 0.6, g = 10.8, l = 0.154, mb = 45, mw = 1.8, r = 0.18, theta(start = 0.88)) annotation(
    Placement(transformation(origin = {90, 1}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.PID PID_x(k = 10, Td = 2, Ti = 10000)  annotation(
    Placement(transformation(origin = {-170, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.PID PID_phi(k = 0.5, Ti = 100000, Td = 0.5)  annotation(
    Placement(transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
    Placement(transformation(origin = {-250, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const1(k = -0.5) annotation(
    Placement(transformation(origin = {-250, 31}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const11(k = 0.5) annotation(
    Placement(transformation(origin = {-99, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add2(k2 = 1, k1 = -1)  annotation(
    Placement(transformation(origin = {10, 29}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add22(k2 = -1, k1 = -1)  annotation(
    Placement(transformation(origin = {10, -31}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add23(k2 = -1, k1 = +1) annotation(
    Placement(transformation(origin = {-69, -2}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add231(k2 = +1, k1 = -1) annotation(
    Placement(transformation(origin = {-210, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Pulse perturbaciones1[6](amplitude = {0, 0, 0, 0, 0, 0}, nperiod = {0, 1, 0, 0, 0, 0}, offset = {0, 0, 0, 0, 0, 0}, period = {0.1, 0.5, 0.1, 0.1, 0.1, 0.1}, startTime = {0, 10, 0, 0, 0, 0}, width = {50, 50, 50, 50, 50, 50}) annotation(
    Placement(transformation(origin = {90, 33}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add2311(k1 = -1, k2 = +1) annotation(
    Placement(transformation(origin = {-209, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.PID PID_theta(Td = 0.1, Ti = 5000, k = 100) annotation(
    Placement(transformation(origin = {-170, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add23111(k1 = +1, k2 = +1) annotation(
    Placement(transformation(origin = {-130, -1}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Noise.NormalNoise noise_x(sigma = 0.01, mu = 1)  annotation(
    Placement(transformation(origin = {130, 31}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Noise.NormalNoise noise_phi(sigma = 0.01, mu = 1) annotation(
    Placement(transformation(origin = {130, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Noise.NormalNoise noise_theta(sigma = 0.005, mu = 1) annotation(
    Placement(transformation(origin = {130, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Product product annotation(
    Placement(transformation(origin = {170, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Product product1 annotation(
    Placement(transformation(origin = {170, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Product product11 annotation(
    Placement(transformation(origin = {170, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Noise.UniformNoise uniNoise_x(y_min = -0.00, y_max = 0.00)  annotation(
    Placement(transformation(origin = {210, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integrator_x annotation(
    Placement(transformation(origin = {250, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Noise.UniformNoise uniNoise_psi(y_max = 0.00, y_min = -0.00)  annotation(
    Placement(transformation(origin = {209, -49}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integrator_psi annotation(
    Placement(transformation(origin = {250, -49}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Noise.UniformNoise uniNoise_theta(y_min = -0.00, y_max = 0.00)  annotation(
    Placement(transformation(origin = {210, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integrator_theta annotation(
    Placement(transformation(origin = {250, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add232(k1 = +1, k2 = +1) annotation(
    Placement(transformation(origin = {290, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add2321(k1 = +1, k2 = +1) annotation(
    Placement(transformation(origin = {289, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add2322(k1 = +1, k2 = +1) annotation(
    Placement(transformation(origin = {290, -49}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax = 20)  annotation(
    Placement(transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax = 20) annotation(
    Placement(transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(PID_phi.y, add2.u2) annotation(
    Line(points = {{-19, 0}, {-10, 0}, {-10, 23}, {-2, 23}}, color = {0, 0, 127}));
  connect(PID_phi.y, add22.u1) annotation(
    Line(points = {{-19, 0}, {-10, 0}, {-10, -25}, {-2, -25}}, color = {0, 0, 127}));
  connect(add23.y, PID_phi.u) annotation(
    Line(points = {{-58, -2}, {-42, -2}, {-42, 0}}, color = {0, 0, 127}));
  connect(const11.y, add23.u2) annotation(
    Line(points = {{-88, -20}, {-81, -20}, {-81, -8}}, color = {0, 0, 127}));
  connect(add231.y, PID_x.u) annotation(
    Line(points = {{-199, 30}, {-182, 30}}, color = {0, 0, 127}));
  connect(perturbaciones1.y, plant.disturbance) annotation(
    Line(points = {{101, 33}, {101, 22}, {91, 22}, {91, 12}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const.y, add2311.u2) annotation(
    Line(points = {{-239, -30}, {-239, -36.5}, {-221, -36.5}, {-221, -36}}, color = {0, 0, 127}));
  connect(add23111.y, add2.u1) annotation(
    Line(points = {{-119, -1}, {-116, -1}, {-116, 35}, {-2, 35}}, color = {0, 0, 127}));
  connect(add23111.y, add22.u2) annotation(
    Line(points = {{-119, -1}, {-116, -1}, {-116, -37}, {-2, -37}}, color = {0, 0, 127}));
  connect(PID_x.y, add23111.u1) annotation(
    Line(points = {{-159, 30}, {-141, 30}, {-141, 5}, {-142, 5}}, color = {0, 0, 127}));
  connect(add2311.y, PID_theta.u) annotation(
    Line(points = {{-198, -30}, {-182, -30}}, color = {0, 0, 127}));
  connect(PID_theta.y, add23111.u2) annotation(
    Line(points = {{-159, -30}, {-142, -30}, {-142, -7}}, color = {0, 0, 127}));
  connect(const1.y, add231.u2) annotation(
    Line(points = {{-239, 31}, {-239, 24}, {-222, 24}}, color = {0, 0, 127}));
  connect(noise_x.y, product.u1) annotation(
    Line(points = {{141, 31}, {150, 31}, {150, 35.5}, {158, 35.5}, {158, 36}}, color = {0, 0, 127}));
  connect(noise_phi.y, product1.u1) annotation(
    Line(points = {{141, -50}, {150, -50}, {150, -43}, {158, -43}, {158, -44}}, color = {0, 0, 127}));
  connect(noise_theta.y, product11.u1) annotation(
    Line(points = {{141, -10}, {150, -10}, {150, -4}, {158, -4}}, color = {0, 0, 127}));
  connect(uniNoise_x.y, integrator_x.u) annotation(
    Line(points = {{221, 30}, {238, 30}}, color = {0, 0, 127}));
  connect(uniNoise_psi.y, integrator_psi.u) annotation(
    Line(points = {{220, -49}, {238, -49}}, color = {0, 0, 127}));
  connect(uniNoise_theta.y, integrator_theta.u) annotation(
    Line(points = {{221, -10}, {238, -10}}, color = {0, 0, 127}));
  connect(integrator_theta.y, add232.u1) annotation(
    Line(points = {{261, -10}, {278, -10}, {278, -4}}, color = {0, 0, 127}));
  connect(integrator_psi.y, add2322.u1) annotation(
    Line(points = {{261, -49}, {278, -49}, {278, -43}}, color = {0, 0, 127}));
  connect(integrator_x.y, add2321.u1) annotation(
    Line(points = {{261, 30}, {277, 30}, {277, 36}}, color = {0, 0, 127}));
  connect(product1.y, add2322.u2) annotation(
    Line(points = {{181, -50}, {187, -50}, {187, -67}, {278, -67}, {278, -55}}, color = {0, 0, 127}));
  connect(product11.y, add232.u2) annotation(
    Line(points = {{181, -10}, {189, -10}, {189, -26}, {278, -26}, {278, -16}}, color = {0, 0, 127}));
  connect(product.y, add2321.u2) annotation(
    Line(points = {{181, 30}, {189, 30}, {189, 14}, {277, 14}, {277, 24}}, color = {0, 0, 127}));
  connect(add2321.y, add231.u1) annotation(
    Line(points = {{300, 30}, {319, 30}, {319, 81}, {-231, 81}, {-231, 35}, {-223, 35}}, color = {0, 0, 127}));
  connect(add232.y, add2311.u1) annotation(
    Line(points = {{301, -10}, {331, -10}, {331, 100}, {-281, 100}, {-281, 0}, {-230, 0}, {-230, -24}, {-222, -24}}, color = {0, 0, 127}));
  connect(add2322.y, add23.u1) annotation(
    Line(points = {{301, -49}, {340, -49}, {340, 61}, {-90, 61}, {-90, 3}, {-81, 3}}, color = {0, 0, 127}));
  connect(plant.state[3], product1.u2) annotation(
    Line(points = {{101, 1}, {101, -78}, {150, -78}, {150, -55.5}, {158, -55.5}, {158, -56}}, color = {0, 0, 127}));
  connect(plant.state[1], product.u2) annotation(
    Line(points = {{101, 1}, {101, 0.5}, {109, 0.5}, {109, 12}, {150, 12}, {150, 24}, {158, 24}}, color = {0, 0, 127}));
  connect(plant.state[2], product11.u2) annotation(
    Line(points = {{101, 1}, {101, -0.75}, {103, -0.75}, {103, 1.5}, {109, 1.5}, {109, -28}, {152, -28}, {152, -16}, {158, -16}}, color = {0, 0, 127}));
  connect(add2.y, limiter.u) annotation(
    Line(points = {{21, 29}, {38, 29}, {38, 30}}, color = {0, 0, 127}));
  connect(add22.y, limiter1.u) annotation(
    Line(points = {{21, -31}, {38, -31}, {38, -30}}, color = {0, 0, 127}));
  connect(limiter1.y, plant.u[2]) annotation(
    Line(points = {{61, -30}, {70.5, -30}, {70.5, -2}, {79, -2}, {79, 1}}, color = {0, 0, 127}));
  connect(limiter.y, plant.u[1]) annotation(
    Line(points = {{61, 30}, {69, 30}, {69, 4}, {78, 4}, {78, 1}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "4.0.0")),
  experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-06, Interval = 0.001),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end PID_Model;
