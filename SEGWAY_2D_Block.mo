block Block2D
  parameter Real mBody = 2.0;               //Masa del cuerpo [kg]
  parameter Real mWheel = 0.2;              //Masa de cada rueda [kg]
  parameter Real r = 0.05;                  //Radio de rueda [m]
  parameter Real l = 0.2;                   //Distancia al centro de masa [m]
  constant Real g = 9.81;                   //Gravedad [m/s²]
  parameter Real J = 0.5 * mWheel * r ^ 2;  //Inercia de cada rueda [kg·m²]
  parameter Real I2 = mBody * l ^ 2;        //Inercia del cuerpo [kg·m²] respecto al eje de giro de las ruedas
  parameter Real c_alpha = 0.001;           //Coeficiente de fricción
  
  // Variables de estado
  Real x(start = 0);                //Desplazamiento lineal [m]
  Real x_velocity(start = 0.0);     //Velocidad sobre el eje X [m/s]
  Real x_acceleration;              //Aceleración eje x[m/s^2]
  Real theta(start = 0.0);          //Ángulo de inclinación(pitch)[rad]
  Real theta_velocity(start = 0.0); //Velocidad de rotación sobre el eje Y [rad/s]
  Real theta_acceleration;          //Aceleración eje Y[rad/s^2]
  
  // Entradas
  Modelica.Blocks.Interfaces.RealInput TL //Torque izquierdo [N·m]
    annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  
  Modelica.Blocks.Interfaces.RealInput TR //Torque derecho [N·m]
    annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  
  //Salidas
  Modelica.Blocks.Interfaces.RealOutput State[4] annotation(
    Placement(visible = true, transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  //Limit
  Real theta_max = Modelica.Constants.pi - acos(r / (l * 2));
  
protected
  Real m11, m12, m21, m22;
  Real b11, b12, b21, b22;
  Real c12;
  Real d11, d12, d21, d22;
  Real g1, g2;  
  
  
equation
  // Cálculo de componentes
  m11 = mBody + 2*mWheel + 2*J/r^2;
  m12 = mBody*l*cos(theta);
  m21 = m12;
  m22 = I2 + mBody*l^2;

  b11 = 1/r;
  b12 = b11;
  b21 = -1;
  b22 = b21;

  c12 = -mBody*l*sin(theta)*theta_velocity;
  
  d11 = 2*c_alpha/r^2;
  d12 = -2*c_alpha/r;
  d21 = d12;
  d22 = 2*c_alpha;
  
  g1 = 0;
  g2 = -mBody*l*g*sin(theta);
  
 
  
  m11*x_acceleration + m12*theta_acceleration + c12 * theta_velocity + d11 * x_velocity + d12 *theta_velocity + g1 = b11*TL + b12*TR;
  m21*x_acceleration + m22*theta_acceleration + d21 * x_velocity + d22 *theta_velocity + g2 = b21*TL + b22*TR;

  der(x) = x_velocity;
  der(x_velocity) = x_acceleration;
  
  der(theta) = theta_velocity;
  der(theta_velocity) = theta_acceleration;


  when abs(theta) > theta_max then 
    terminate("Desestablilización total");  
  end when;
  annotation(
    uses(Modelica(version = "4.0.0")));
end Block2D;
