within SailBoatModel.Components.Instances;

model MainSail
  extends SailBoatModel.Components.Templates.SailWithMount(servoMount.n = {0, 1, 0}, servoJoint.n = {0, 1, 0}, servoJoint.useAxisFlange = false);
equation
  connect(servoMount.frame_a, frame_a) annotation(Line(visible = true, origin = {-40, -88.667}, points = {{-20, 22.667}, {-20, -11.333}, {40, -11.333}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {31.513, -12.549}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, points = {{-85.894, 67.767}, {32.907, 14.223}, {52.986, -81.989}})}));
end MainSail;
