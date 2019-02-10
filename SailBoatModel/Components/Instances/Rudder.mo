within SailBoatModel.Components.Instances;

model Rudder
  extends SailBoatModel.Components.Templates.SailWithMount;
equation
  connect(servoMount.frame_a, frame_a) annotation(Line(points = {{-60, -66}, {-60, -66}, {-60, -80}, {0, -80}, {0, -100}, {0, -100}}, color = {95, 95, 95}));
  connect(servo.flange, servoJoint.axis) annotation(Line(points = {{-34, -34}, {-10, -34}, {-10, -34}, {-10, -34}}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {1.116, -64.673}, fillColor = {37, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, points = {{-1.116, 44.673}, {14.78, -22.336}, {-13.665, -22.336}})}));
end Rudder;
