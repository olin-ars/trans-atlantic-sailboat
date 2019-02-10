within SailBoatModel.Components.Examples;

model SailTest
  inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Body attached to the sail
  // Constant input
  Modelica.Blocks.Sources.Constant xcomp(k = 1) annotation(Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant ycomp(k = 0) annotation(Placement(visible = true, transformation(origin = {-50, -35.609}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant zcomp(k = 0.01) annotation(Placement(visible = true, transformation(origin = {-50, -76.09}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Sail
  Templates.Internal.Sail sail(r_COL = {0, 0, 0}, heightDirection = {0, 1, 0}, sailWidth = 0.01, r_0.fixed = true, v_0.fixed = true, r_0.start = {0, 0, 0}, v_0.start = {0, 0, 0}, mastDiameter = 0.05) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Sensor
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity sail_v(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(Placement(visible = true, transformation(origin = {25.985, -27.609}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Wind velocity vector
  Real v_w_W[3] "Wind absolute velocity vector resolved in the world frame";
equation
  v_w_W = {xcomp.y, ycomp.y, zcomp.y};
  sail.windVel = v_w_W - sail_v.v;
  connect(sail_v.frame_a, sail.frame_a) annotation(Line(visible = true, origin = {6.026, -18.804}, points = {{9.958, -8.804}, {-1.966, -8.804}, {-1.966, 8.804}, {-6.026, 8.804}}, color = {95, 95, 95}));
end SailTest;
