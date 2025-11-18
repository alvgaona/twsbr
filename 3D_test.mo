model test_block3D
  Modelica.Blocks.Sources.Constant TorqueL(k = 1)  annotation(
    Placement(visible = true, transformation(origin = {-80, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant TorqueR annotation(
    Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Block3D Segway3D annotation(
    Placement(visible = true, transformation(origin = {-2, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(TorqueR.y, Segway3D.TR) annotation(
    Line(points = {{-69, 0}, {-43, 0}, {-43, 10}, {-14, 10}}, color = {0, 0, 127}));
  connect(TorqueL.y, Segway3D.TL) annotation(
    Line(points = {{-68, 36}, {-56, 36}, {-56, 18}, {-14, 18}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "4.0.0")));
end test_block3D;
