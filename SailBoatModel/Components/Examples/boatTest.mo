within SailBoatModel.Components.Examples;

model boatTest
  inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Absolute wind velocity vector (parameter)
  parameter Modelica.SIunits.Velocity v_w_W[3] = {1, 0, 0.05} "Wind absolute velocity vector resolved in the world frame";
  // Damping coefficient
  parameter Real k_dw = 1000 "Rotational damping coefficient for the sail, acting at the sail's mount point" annotation(Evaluate = true, Dialog(group = "Sail Mount Parameters"));
  Modelica.Blocks.Math.Gain degToRad(k = Modelica.Constants.pi / 180) annotation(Placement(visible = true, transformation(origin = {-50, 37.054}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // spanker
  SailBoatModel.Components.Instances.MainSail mainsail(r_COL = {0, 0, 0}, heightDirection = {0, 1, 0}, sailWidth = 0.01, r_0.fixed = true, v_0.fixed = true, r_0.start = {0, 0, 0}, v_0.start = {0, 0, 0}, color = {0, 0, 0}, m = 10, sailLength = 0.6) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Sensor
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity sail_v(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) "Absolute linear velocity of the boat" annotation(Placement(visible = true, transformation(origin = {70, -34.725}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Body attached to the main sail
  Modelica.Mechanics.MultiBody.Parts.BodyBox raft(r = {0.5, 0, 0}, r_shape = {-0.5, 0, 0}, widthDirection = {0, 0, 1}, width = 1, height = 0.01, r_0.fixed = false, v_0.fixed = false, angles_fixed = false, w_0_fixed = false, z_0_fixed = false, a_0.fixed = false) annotation(Placement(visible = true, transformation(origin = {13.248, -65.94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Servo input
  Modelica.Blocks.Sources.Sine sine1(freqHz = 1, amplitude = 45, offset = 1) annotation(Placement(visible = true, transformation(origin = {-82.014, 37.353}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity sail_w(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(Placement(visible = true, transformation(origin = {70, -52.333}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque damping annotation(Placement(visible = true, transformation(origin = {-36.812, -52.333}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Instances.Spanker spanker(r_COL = {0, 0, 0}, heightDirection = {0, 1, 0}, sailWidth = 0.01, r_0.fixed = true, v_0.fixed = true, r_0.start = {0, 0, 0}, v_0.start = {0, 0, 0}, color = {0, 0, 0}, m = 10, sailLength = 0.6) annotation(Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {1, 0, 0}) annotation(Placement(visible = true, transformation(origin = {33.13, -13.558}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  mainsail.windVel = v_w_W - sail_v.v;
  spanker.windVel = v_w_W - sail_v.v;
  damping.force = zeros(3);
  damping.torque[1] = -k_dw * sail_w.w[1];
  damping.torque[2] = 0;
  damping.torque[3] = -k_dw * sail_w.w[3];
  connect(raft.frame_a, mainsail.frame_a) annotation(Line(visible = true, origin = {1.083, -47.293}, points = {{2.165, -18.647}, {-1.083, -18.647}, {-1.083, 37.293}}, color = {95, 95, 95}));
  connect(sine1.y, degToRad.u) annotation(Line(visible = true, origin = {-65.754, 37.204}, points = {{-5.26, 0.149}, {0.754, 0.149}, {0.754, -0.15}, {3.754, -0.15}}, color = {1, 37, 163}));
  connect(raft.frame_a, sail_v.frame_a) annotation(Line(visible = true, origin = {15.823, -50.332}, points = {{-12.575, -15.608}, {-15.801, -15.608}, {-15.801, 15.607}, {44.177, 15.607}}, color = {95, 95, 95}));
  connect(damping.frame_b, mainsail.frame_a) annotation(Line(visible = true, origin = {-8.937, -38.222}, points = {{-17.875, -14.111}, {8.937, -14.111}, {8.937, 28.222}}, color = {95, 95, 95}));
  connect(sail_w.frame_a, mainsail.frame_a) annotation(Line(visible = true, origin = {20, -38.222}, points = {{40, -14.111}, {-20, -14.111}, {-20, 28.222}}, color = {95, 95, 95}));
  connect(degToRad.y, spanker.heading) annotation(Line(visible = true, origin = {32.25, 14.827}, points = {{-71.25, 22.227}, {22.75, 22.227}, {22.75, -22.227}, {25.75, -22.227}}, color = {1, 37, 163}));
  connect(spanker.frame_a, fixedTranslation1.frame_b) annotation(Line(visible = true, origin = {61.043, -12.372}, points = {{8.957, 2.372}, {8.957, -1.186}, {-17.913, -1.186}}, color = {95, 95, 95}));
  connect(fixedTranslation1.frame_a, mainsail.frame_auxillary) annotation(Line(visible = true, origin = {14.789, -9.779}, points = {{8.342, -3.779}, {-1.776, -3.779}, {-1.776, 3.779}, {-4.789, 3.779}}, color = {95, 95, 95}));
  connect(degToRad.y, mainsail.heading) annotation(Line(visible = true, origin = {-20.25, 14.827}, points = {{-18.75, 22.227}, {5.25, 22.227}, {5.25, -22.227}, {8.25, -22.227}}, color = {1, 37, 163}));
end boatTest;
