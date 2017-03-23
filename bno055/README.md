For the robotic cup:
  When initialising the robot, once we know which color (and thus side) we are,
   set initial heading to :
      - alpha if color1
      - (alpha + 180) % 360 (in degrees) if color2

This way, angles will be the same in both cases.
