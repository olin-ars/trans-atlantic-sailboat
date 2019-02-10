within SailBoatModel.Components.Templates.Internal;

model Sail
  // Import relevant sub-packages from MSL 3.2.2
  import SI = Modelica.SIunits;
  import Types = Modelica.Mechanics.MultiBody.Types;
  import Frames = Modelica.Mechanics.MultiBody.Frames;
  import Modelica.SIunits.Conversions.to_unit1;
  import Modelica.SIunits.Conversions.to_deg;
  // Sail Rigid Body Parameters
  parameter SI.Position r_COL[3] = {0, 0, 0} "Vector from frame_a of the sail rigid body to the center of lift" annotation(Dialog(group = "Rigid Body Parameters"));
  parameter SI.Mass m = 1 "Mass of the sail" annotation(Dialog(group = "Rigid Body Parameters"));
  // General Parameters
  parameter Boolean animation = true "= true, if animation shall be enabled (show box between frame_a and frame_b)";
  // Sail Force Parameters
  parameter Modelica.Mechanics.MultiBody.Types.Axis lengthDirection = {1, 0, 0} "Unit vector in length direction of sail, resolved in frame_a" annotation(Evaluate = true, Dialog(group = "Sail Force Parameters"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis heightDirection = {0, 1, 0} "Unit vector in height direction of mast, resolved in frame_a" annotation(Evaluate = true, Dialog(group = "Sail Force Parameters"));
  parameter SI.Density rho = 1225 "Density of the fluid (e.g. air: 1225 or water: 997)" annotation(Evaluate = true, Dialog(group = "Sail Force Parameters"));
  parameter SI.Area area = mastHeight * sailLength "Maximum area of sail, when directly facing wind" annotation(Evaluate = true, Dialog(group = "Sail Force Parameters"));
  final parameter Modelica.Mechanics.MultiBody.Types.Axis sail_n = cross(lengthDirection, heightDirection) "Vector normal to sail surface (surface defined as length x height)" annotation(Evaluate = true, Dialog(group = "Sail Force Parameters"));
  // Kinematic variables
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  // Initialization Parameters
  parameter Boolean angles_fixed = false "= true, if angles_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
  parameter SI.Angle angles_start[3] = {0, 0, 0} "Initial values of angles to rotate frame_a around 'sequence_start' axes into frame_b" annotation(Dialog(tab = "Initialization"));
  parameter Types.RotationSequence sequence_start = {1, 2, 3} "Sequence of rotations to rotate frame_a into frame_b at initial time" annotation(Evaluate = true, Dialog(tab = "Initialization"));
  parameter Boolean w_0_fixed = false "= true, if w_0_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
  parameter SI.AngularVelocity w_0_start[3] = {0, 0, 0} "Initial or guess values of angular velocity of frame_a resolved in world frame" annotation(Dialog(tab = "Initialization"));
  parameter Boolean z_0_fixed = false "= true, if z_0_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
  parameter SI.AngularAcceleration z_0_start[3] = {0, 0, 0} "Initial values of angular acceleration z_0 = der(w_0)" annotation(Dialog(tab = "Initialization"));
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Dialog(tab = "Advanced"));
  parameter Types.RotationSequence sequence_angleStates = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a around the 3 angles used as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced", enable = not useQuaternions));
  // Animation inputs
  parameter SI.Length mastHeight = 1 "Length of box" annotation(Evaluate = true, Dialog(tab = "Animation", enable = animation));
  parameter SI.Length mastDiameter = world.defaultBodyDiameter "Diameter of mast, as used in animation" annotation(Evaluate = true, Dialog(tab = "Animation", enable = animation));
  parameter SI.Length sailLength = 1 "Length of box" annotation(Evaluate = true, Dialog(tab = "Animation", enable = animation));
  final parameter SI.Distance sailHeight = mastHeight "Height of mast" annotation(Evaluate = true, Dialog(tab = "Animation", enable = animation));
  parameter SI.Distance sailWidth = sailLength "Width of sail" annotation(Evaluate = true, Dialog(tab = "Animation", enable = animation));
  input Types.Color color = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of cylinder" annotation(Evaluate = true, Dialog(tab = "Animation", colorSelector = true, enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Evaluate = true, Dialog(tab = "Animation", enable = animation));
  // Sail components
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque windLiftAndDrag(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(Placement(visible = true, transformation(origin = {-32.632, -15.849}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  // System Inputs
  Modelica.Blocks.Interfaces.RealInput windVel[3] annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  // Angle of Attack Table Lookup: Output Column 1 = C_D, Output Column 2 = C_L
  Modelica.Blocks.Tables.CombiTable1Ds aoa_lookup(tableOnFile = true, tableName = "c_data", fileName = Modelica.Utilities.Files.loadResource("modelica://SailBoatModel.Components.Templates.Internal.Sail/../../../c_sail_data.mat"), columns = 2:3) "Looks up appropriate C_L and C_D coefficient values for given angles of attacks" annotation(Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Derived values from input
  SI.Velocity v_w_s[3] "Wind velocity resolved in body frame";
  SI.Velocity v_w_s_m "Magnitude of the wind velocity";
  SI.Velocity v_w_s_u[3] "Unit vector in direction of the wind velocity (resolved in the body frame)";
  SI.Angle alpha "Angle of attack of sail";
  Real liftVector[3];
  Real kvj;
  Real vk;
  Real vkkvj;
  // Real dir_l[3] "Unit vector in direction of the sail's lift force";
  // Real dir_d[3] "Unit vector in direction of the sail's drag force";
  // Forces
  Real f_l[3] "Lift force";
  Real f_d[3] "Drag force";
  Modelica.Mechanics.MultiBody.Forces.Damper damping(d = 10, animation = false) annotation(Placement(visible = true, transformation(origin = {-40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedFrame(r = {0, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-80, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Debugging bodies
  Modelica.Mechanics.MultiBody.Parts.Body sailBody(r_CM = r_COL, m = m, specularCoefficient = specularCoefficient, animation = false, w_0_start = w_0_start, w_0_fixed = w_0_fixed, sequence_start = sequence_start, angles_fixed = angles_fixed, angles_start = angles_start, z_0_fixed = z_0_fixed, z_0_start = z_0_start, useQuaternions = useQuaternions, enforceStates = enforceStates, sphereColor = color, cylinderColor = color) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation sailAnimation(r = {sailLength / 2, -sailHeight / 2, 0}, animation = animation, shapeType = "box", r_shape = {0, 0, 0}, lengthDirection = lengthDirection, widthDirection = heightDirection, length = sailLength, width = sailHeight, height = sailWidth, color = color, specularCoefficient = specularCoefficient) annotation(Placement(visible = true, transformation(origin = {46.716, 2.985}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation mastAnimation(shapeType = "cylinder", r_shape = {0, 0, 0}, lengthDirection = heightDirection, length = mastHeight, width = mastDiameter, height = mastDiameter, extra = 0, color = color, specularCoefficient = specularCoefficient, widthDirection = lengthDirection, animation = animation, r = zeros(3)) annotation(Placement(visible = true, transformation(origin = {46.716, 24.025}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
protected
  // World component
  outer Modelica.Mechanics.MultiBody.World world;
equation
  // Kinematic equations
  r_0 = frame_a.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  // Resolving the wind velocity's direction, angle of attack, and sail force direction
  v_w_s = Frames.resolve2(frame_a.R, windVel);
  v_w_s_m = Modelica.Math.Vectors.norm(v_w_s);
  v_w_s_u = Modelica.Math.Vectors.normalize(v_w_s);
  alpha = Modelica.Math.acos(abs(v_w_s_u * lengthDirection));
  liftVector = cross(v_w_s_u, heightDirection);
  // v x j
  kvj = sign(sail_n * liftVector);
  //  k dot (v x j)
  vk = sign(v_w_s_u * sail_n);
  // v dot k
  vkkvj = if vk * kvj > 0 then 1 else -1;
  // does truth table
  // dir_l = if sign(v_w_s_u * sail_n) < 1e-10 then sail_n else sign(v_w_s_u * sail_n) * sail_n;
  // dir_d = if sign(v_w_s_u * lengthDirection) < 1e-10 then lengthDirection else sign(v_w_s_u * lengthDirection) * lengthDirection;
  // Calculating sail force
  aoa_lookup.u = to_deg(abs(alpha));
  // alpha is in radians, the table expects degrees
  f_l = 0.5 * rho * area * aoa_lookup.y[2] * v_w_s_m ^ 2 * vkkvj * liftVector;
  f_d = 0.5 * rho * area * aoa_lookup.y[1] * v_w_s_m ^ 2 * v_w_s_u;
  windLiftAndDrag.torque = zeros(3);
  windLiftAndDrag.force = f_d + f_l;
  connect(fixedFrame.frame_b, damping.frame_a) annotation(Line(visible = true, origin = {-60, 50}, points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(sailBody.frame_a, frame_a) annotation(Line(visible = true, origin = {0, -55}, points = {{0, 45}, {0, -45}}, color = {95, 95, 95}));
  connect(windLiftAndDrag.frame_b, sailBody.frame_a) annotation(Line(visible = true, origin = {-7.544, -13.899}, points = {{-15.088, -1.95}, {7.544, -1.95}, {7.544, 3.899}}, color = {95, 95, 95}));
  connect(damping.frame_b, sailBody.frame_a) annotation(Line(visible = true, origin = {-15.595, 12.795}, points = {{-14.405, 37.205}, {-8.392, 37.205}, {-8.392, -25.807}, {15.595, -25.807}, {15.595, -22.795}}, color = {95, 95, 95}));
  connect(sailAnimation.frame_b, sailBody.frame_a) annotation(Line(visible = true, origin = {20.784, -6.011}, points = {{15.931, 8.996}, {12.819, 8.996}, {12.819, -7.001}, {-20.784, -7.001}, {-20.784, -3.989}}, color = {95, 95, 95}));
  connect(mastAnimation.frame_b, sailBody.frame_a) annotation(Line(visible = true, origin = {20.784, 2.405}, points = {{15.931, 21.62}, {12.819, 21.62}, {12.819, -15.417}, {-20.784, -15.417}, {-20.784, -12.405}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {0, 0, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-100, -100}, {100, 100}}), Line(visible = true, origin = {-64.851, 20.167}, points = {{-25.149, -0.167}, {12.98, -0.167}, {-0.406, 12.382}, {12.98, -0.167}, {-0.406, -11.88}}, color = {0, 255, 0}, pattern = LinePattern.Dot), Line(visible = true, origin = {-64.851, 1.88}, points = {{-25.149, -0.167}, {12.98, -0.167}, {-0.406, 12.382}, {12.98, -0.167}, {-0.406, -11.88}}, color = {0, 255, 0}, pattern = LinePattern.Dot), Line(visible = true, origin = {-64.851, -15.323}, points = {{-25.149, -0.167}, {12.98, -0.167}, {-0.406, 12.382}, {12.98, -0.167}, {-0.406, -11.88}}, color = {0, 255, 0}, pattern = LinePattern.Dot)}));
end Sail;
