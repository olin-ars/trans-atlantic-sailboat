within SailBoatModel.Components.Templates;

partial model SailWithMount
  // Import relevant sub-packages from MSL 3.2.2
  import SI = Modelica.SIunits;
  import Types = Modelica.Mechanics.MultiBody.Types;
  import Frames = Modelica.Mechanics.MultiBody.Frames;
  import Modelica.SIunits.Conversions.to_unit1;
  import Modelica.SIunits.Conversions.to_deg;
  // Sail Rigid Body Parameters
  parameter SI.Position r_COL[3] = {0, 0, 0} "Vector from frame_a of the sail rigid body to the center of lift" annotation(Evaluate = true, Dialog(group = "Rigid Body Parameters"));
  parameter SI.Mass m = 1 "Mass of the sail + mount" annotation(Evaluate = true, Dialog(group = "Rigid Body Parameters"));
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
  // Sail and Mount Components
  Internal.Sail sail(mastHeight = mastHeight, mastDiameter = mastDiameter, sailLength = sailLength, sailWidth = sailWidth, color = color, specularCoefficient = specularCoefficient, enforceStates = enforceStates, useQuaternions = useQuaternions, angles_fixed = angles_fixed, angles_start = angles_start, sequence_start = sequence_start, w_0_fixed = w_0_fixed, w_0_start = w_0_start, z_0_fixed = z_0_fixed, z_0_start = z_0_start, r_COL = r_COL, m = m, animation = animation, lengthDirection = lengthDirection, heightDirection = heightDirection, rho = rho, area = area) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute servoJoint(useAxisFlange = true) annotation(Placement(visible = true, transformation(origin = {0, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_auxillary annotation(Placement(visible = true, transformation(origin = {100, -60}, extent = {{-16, -16}, {16, 16}}, rotation = 180), iconTransformation(origin = {100, -60}, extent = {{-16, -16}, {16, 16}}, rotation = 180)));
  Modelica.Mechanics.Rotational.Sources.Position servo(useSupport = true) annotation(Placement(visible = true, transformation(origin = {-44, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D servoMount annotation(Placement(visible = true, transformation(origin = {-60, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Inputs
  Modelica.Blocks.Interfaces.RealInput windVel[3] annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 1.673}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput heading annotation(Placement(visible = true, transformation(origin = {-100, -34}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -74}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
protected
  outer Modelica.Mechanics.MultiBody.World world;
equation
  r_0 = frame_a.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(frame_auxillary, servoJoint.frame_b) annotation(Line(points = {{100, -60}, {30, -60}, {30, -24}, {0, -24}, {0, -24}}, color = {95, 95, 95}));
  connect(servoMount.flange_b, servo.support) annotation(Line(points = {{-50, -56}, {-44, -56}, {-44, -44}, {-44, -44}}));
  connect(heading, servo.phi_ref) annotation(Line(points = {{-100, -34}, {-56, -34}, {-56, -34}, {-56, -34}}, color = {0, 0, 127}, visible = true));
  connect(windVel, sail.windVel) annotation(Line(visible = true, origin = {-56, 0}, points = {{-44, 0}, {44, 0}}, color = {1, 37, 163}));
  connect(sail.frame_a, servoJoint.frame_b) annotation(Line(visible = true, origin = {0, -17}, points = {{0, 7}, {-0, -7}}, color = {95, 95, 95}));
  connect(servoJoint.frame_a, frame_a) annotation(Line(visible = true, origin = {0, -72}, points = {{0, 28}, {0, -28}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {0, 0, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-100, -100}, {100, 100}}), Polygon(visible = true, origin = {21.076, 35.696}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, points = {{-103.065, 40.437}, {34.141, 1.952}, {68.924, -42.389}}), Line(visible = true, origin = {-64.851, 20.167}, points = {{-25.149, -0.167}, {12.98, -0.167}, {-0.406, 12.382}, {12.98, -0.167}, {-0.406, -11.88}}, color = {0, 255, 0}, pattern = LinePattern.Dot), Line(visible = true, origin = {-64.851, 1.88}, points = {{-25.149, -0.167}, {12.98, -0.167}, {-0.406, 12.382}, {12.98, -0.167}, {-0.406, -11.88}}, color = {0, 255, 0}, pattern = LinePattern.Dot), Line(visible = true, origin = {-64.851, -15.323}, points = {{-25.149, -0.167}, {12.98, -0.167}, {-0.406, 12.382}, {12.98, -0.167}, {-0.406, -11.88}}, color = {0, 255, 0}, pattern = LinePattern.Dot), Ellipse(visible = true, origin = {-78.086, -73.027}, fillColor = {140, 140, 140}, fillPattern = FillPattern.Sphere, extent = {{-8.086, -16.973}, {8.086, 16.973}}), Text(visible = true, origin = {-78.883, 31.334}, textColor = {255, 255, 255}, extent = {{-11.117, -11.334}, {11.117, 11.334}}, textString = "windVel", fontName = "Liberation Sans"), Text(visible = true, origin = {-78.883, -46.666}, textColor = {255, 255, 255}, extent = {{-11.117, -11.334}, {11.117, 11.334}}, textString = "heading", fontName = "Liberation Sans")}));
end SailWithMount;
