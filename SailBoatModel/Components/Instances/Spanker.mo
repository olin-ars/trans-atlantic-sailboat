within SailBoatModel.Components.Instances;

model Spanker
  parameter Modelica.Mechanics.MultiBody.Types.Axis mastAxis = {0, 1, 0} annotation(Evaluate = true, Dialog(group = "Sail Mount Parameters"));
  extends SailBoatModel.Components.Templates.SailWithMount(servoJoint.n = mastAxis, servoJoint.useAxisFlange = true, servo.useSupport = true, servoMount.n = mastAxis);
equation
  connect(servoMount.frame_a, frame_a) annotation(Line(points = {{-60, -66}, {-60, -66}, {-60, -80}, {0, -80}, {0, -100}, {0, -100}}, color = {95, 95, 95}));
  connect(servo.flange, servoJoint.axis) annotation(Line(points = {{-34, -34}, {-10, -34}, {-10, -34}, {-10, -34}}));
end Spanker;
