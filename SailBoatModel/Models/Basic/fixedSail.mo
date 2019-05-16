within SailBoatModel.Models.Basic;

model fixedSail
  inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Absolute wind velocity vector (parameter)
  Modelica.SIunits.Velocity v_w_W[3] "Wind absolute velocity vector resolved in the world frame";
  // Damping coefficient
  parameter Real k_dw = 1000 "Rotational damping coefficient for the sail, acting at the sail's mount point" annotation(Evaluate = true, Dialog(group = "Sail Mount Parameters"));
  // spanker
  SailBoatModel.Components.Instances.Spanker spanker(r_COL = {0, 0, 0}, heightDirection = {0, 1, 0}, sailWidth = 0.01, r_0.fixed = true, v_0.fixed = true, r_0.start = {0, 0, 0}, v_0.start = {0, 0, 0}, color = {0, 0, 0}, m = 10, sailLength = 0.6) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Sensor
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity sail_v(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) "Absolute linear velocity of the boat" annotation(Placement(visible = true, transformation(origin = {70, -14.725}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Body attached to the main sail
  Modelica.Mechanics.MultiBody.Parts.BodyBox raft(r = {0.5, 0, 0}, r_shape = {-0.5, 0, 0}, widthDirection = {0, 0, 1}, width = 1, height = 0.01, r_0.fixed = false, v_0.fixed = false, angles_fixed = false, w_0_fixed = false, z_0_fixed = false, a_0.fixed = false) annotation(Placement(visible = true, transformation(origin = {13.248, -65.94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Servo input
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity sail_w(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(Placement(visible = true, transformation(origin = {70, -42.333}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque damping annotation(Placement(visible = true, transformation(origin = {-36.812, -42.333}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition sail_pos annotation(Placement(visible = true, transformation(origin = {70, 18.093}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROS_Bridge.Blocks.ROS_Sampler ros_sampler(nout = 1, samplePeriod = 0.05, nin = 1) annotation(Placement(visible = true, transformation(origin = {-80, 37.054}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 1, k = Modelica.Constants.pi / 180) annotation(Placement(visible = true, transformation(origin = {-30, 3.814}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  v_w_W[1] = 1;
  v_w_W[2] = 0;
  v_w_W[3] = 0.1;
  spanker.windVel = v_w_W - sail_v.v;
  damping.force = zeros(3);
  damping.torque[1] = -k_dw * sail_w.w[1];
  damping.torque[2] = 0;
  damping.torque[3] = -k_dw * sail_w.w[3];
  connect(raft.frame_a, spanker.frame_a) annotation(Line(visible = true, origin = {1.083, -47.293}, points = {{2.165, -18.647}, {-1.083, -18.647}, {-1.083, 37.293}}, color = {95, 95, 95}));
  connect(raft.frame_a, sail_v.frame_a) annotation(Line(visible = true, origin = {15.823, -40.332}, points = {{-12.575, -25.608}, {-15.801, -25.608}, {-15.801, 25.607}, {44.177, 25.607}}, color = {95, 95, 95}));
  connect(sail_w.frame_a, spanker.frame_a) annotation(Line(visible = true, origin = {20, -31.556}, points = {{40, -10.778}, {-20, -10.778}, {-20, 21.556}}, color = {95, 95, 95}));
  connect(damping.frame_b, spanker.frame_a) annotation(Line(visible = true, origin = {-8.937, -31.556}, points = {{-17.874, -10.778}, {8.937, -10.778}, {8.937, 21.556}}, color = {95, 95, 95}));
  connect(sail_pos.frame_a, spanker.frame_a) annotation(Line(visible = true, origin = {34.8, 0.032}, points = {{25.2, 18.061}, {22.2, 18.061}, {22.2, -13.045}, {-34.8, -13.045}, {-34.8, -10.032}}, color = {95, 95, 95}));
  connect(spanker.frame_a, world.frame_b) annotation(Line(visible = true, origin = {-46.71, -7.205}, points = {{46.71, -2.795}, {46.71, -5.808}, {-30.065, -5.808}, {-30.065, 7.205}, {-33.29, 7.205}}, color = {95, 95, 95}));
  connect(sail_v.v[1], ros_sampler.u[1]) annotation(Line(visible = true, origin = {13, 25.521}, points = {{68, -40.246}, {71, -40.246}, {71, 34.479}, {-105, 34.479}, {-105, 11.533}}, color = {1, 37, 163}));
  connect(firstOrder1.y, spanker.heading) annotation(Line(visible = true, origin = {-15.25, -1.793}, points = {{-3.75, 5.607}, {0.25, 5.607}, {0.25, -5.607}, {3.25, -5.607}}, color = {1, 37, 163}));
  connect(ros_sampler.y[1], firstOrder1.u) annotation(Line(visible = true, origin = {-50.25, 20.434}, points = {{-18.75, 16.62}, {5.25, 16.62}, {5.25, -16.62}, {8.25, -16.62}}, color = {1, 37, 163}));
  annotation(experiment(StopTime = -1, Interval = 0.01));
end fixedSail;
