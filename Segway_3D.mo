class Segway_3D "Segway 3D Model"
  parameter Real mBody(start = 2.0);  //Masa del cuerpo [kg]
  parameter Real mWheel(start = 0.2); //Masa de cada rueda [kg]
  parameter Real r  = 0.05;           //Radio de rueda [m]
  parameter Real l  = 0.2;            //Distancia al centro de masa [m]
  parameter Real d = 0.01;            //Distancia entre las ruedas [m]
  parameter Real lWheel = 0.02;       //Anchura de las ruedas [m]
  constant Real g  = 9.81;            //Gravedad [m/s²]
  parameter Real J = 0.5*mWheel*r^2;            //Inercia de cada rueda [kg·m²]
  parameter Real K = (mWheel*r^2)/4;             //Inercia de la rueda respecto al diametro central [kg·m²] 1/4 mr^2 + 1/12 mlWheel^2 despreciamos el segundo termino
  parameter Real I1 = mBody * l^2;              //Inercia del cuerpo [kg·m²] respecto al eje X
  parameter Real I2 = I1;                       //Inercia del cuerpo [kg·m²] respecto al eje de giro de las ruedas
  parameter Real I3 = 0;                        //Inercia del cuerpo [kg·m²] respecto al eje de giro vertical (suponemos masa puntual)
  parameter Real c_alpha = 0.001;               //Coeficiente de fricción
 
  // Variables de estado
  Real x(start=0);                //Desplazamiento lineal [m]
  Real x_velocity(start=0.0);     //Velocidad sobre el eje X [m/s]
  Real x_acceleration;            //Aceleración eje x[m/s^2]
  Real theta(start=0.0);          //Ángulo de inclinación(pitch)[rad]
  Real theta_velocity(start=0.0); //Velocidad de rotación sobre el eje Y [rad/s]
  Real theta_acceleration;        //Aceleración eje Y[rad/s^2]
  Real psi(start = 0.0);          //Ángulo de rotación (yaw) [rad]
  Real psi_velocity(start = 0.0); //Velocidad de rotación sobre el eje Z [rad/s]
  Real psi_accel;                  //Aceleración eje Z[rad/s^2]
  // Entradas
  Real TL;            //Torque izquierdo [N·m]
  Real TR;            //Torque derecho [N·m]
  
  
  //Limit 
  Real theta_max = Modelica.Constants.pi-acos(r/(l*2));

protected
  Real m11, m12, m21, m22, m33;
  Real b11, b12, b21, b22, b31, b32;
  Real c12, c13, c23, c31, c32, c33;
  Real d11, d12, d21, d22, d33;
  Real g1, g2, g3;  
  
  
equation
  // Cálculo de componentes
  m11 = mBody + 2*mWheel + 2*J/r^2;
  m12 = mBody*l*cos(theta);
  m21 = m12;
  m22 = I2 + mBody*l^2;
  m33 = I3 + 2*K + ((mWheel * d^2)/2) + ((J * d^2)/(2*r^2)) - ((I3-I1-mBody*l^2)*(sin(theta))^2);
  
  b11 = 1/r;
  b12 = b11;
  b21 = -1;
  b22 = b21;
  b31 = -d/2*r;
  b32 = -b31;

  c12 = -mBody*l*sin(theta)* theta_velocity;
  c13 = -mBody*l*sin(theta)* psi_velocity;
  c23 = (I3 - I1 - mBody*l^2)* psi_velocity * sin(theta) * cos(theta);
  c31 = mBody * l * psi_velocity * sin(theta);
  c32 = -(I3 -I1 - mBody*l^2) * sin(theta) * cos(theta) * psi_velocity;
  c33 = -(I3 -I1 - mBody*l^2) * sin(theta) * cos(theta) * theta_velocity;

  d11 = 2*c_alpha/r^2;
  d12 = -2*c_alpha/r;
  d21 = d12;
  d22 = 2*c_alpha;
  d33 = c_alpha * d^2 / (2*r^2);
  
  g1 = 0;
  g2 = -mBody*l*g*sin(theta);
  g3 = 0;
  
  
  m11*x_acceleration + m12*theta_acceleration + c12 * theta_velocity + c13 * psi_velocity + d11 * x_velocity + d12 *theta_velocity + g1 = b11*TL + b12*TR;
  m21*x_acceleration + m22*theta_acceleration + c23 * psi_velocity + d21 * x_velocity + d22 *theta_velocity + g2 = b21*TL + b22*TR;
  m33 * psi_accel + c31 * x_velocity + c32 * theta_velocity + c33 * psi_velocity + d33 * psi_velocity = b31 * TL + b32 * TR;

  
  der(x) = x_velocity;
  der(x_velocity) = x_acceleration;
  
  der(theta) = theta_velocity;
  der(theta_velocity) = theta_acceleration;
  
  der(psi) = psi_velocity;
  der(psi_velocity) = psi_accel;

  when abs(theta) > theta_max then
    terminate("Desestabilización total");
  end when;

end Segway_3D;
