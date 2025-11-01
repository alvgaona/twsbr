block LQRController "LQR Controller with State Setpoint"
  // --- Gains computed in MATLAB ---
  parameter Real K[2,6] = {
    {-2.23606798, -53.84641166, -0.70710678, -7.03068959, -10.66506182, -1.10288302},
    {-2.23606798, -53.84641166, 0.70710678, -7.03068959, -10.66506182, 1.10288302}
  } "LQR gain matrix";

  parameter Modelica.Units.SI.Torque max_torque = 40 "Maximum absolute output torque (Nm)";

  // --- Inputs ---
  Modelica.Blocks.Interfaces.RealInput state_setpoint[6] "State setpoint vector (x_ref)"
    annotation(Placement(transformation(extent={{-120,20},{-100,40}})));

  Modelica.Blocks.Interfaces.RealInput state[6] "Measured state vector (x)"
    annotation(Placement(transformation(extent={{-120,-40},{-100,-20}})));

  // --- Output ---
  Modelica.Blocks.Interfaces.RealOutput torque[2] "Control torques [tau_L, tau_R]"
    annotation(Placement(transformation(extent={{100,-10},{120,10}})));

protected
  Real error_state[6] "Error vector (x - x_ref)";
  Modelica.Units.SI.Torque unbounded_torque[2] "Calculated torque before limits";
  parameter Modelica.Units.SI.Torque min_torque = -max_torque "Minimum output torque (Nm)";

equation
  error_state = state - state_setpoint;

  unbounded_torque = -K * error_state;

  for i in 1:2 loop
    torque[i] = max(min_torque, min(max_torque, unbounded_torque[i]));
  end for;

  annotation(Icon(graphics={
    Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,255}, fillColor={255,255,255}, fillPattern=FillPattern.Solid),
    Text(extent={{-80,40},{80,-40}}, textString="LQR"),
    Text(extent={{-150,150},{150,110}}, textString="%name")}));
end LQRController;
