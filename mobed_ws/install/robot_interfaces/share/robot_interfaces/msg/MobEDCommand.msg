# 1. Driving Command
# Target linear velocity (x, y) and angular velocity (z) for the robot body
geometry_msgs/Twist twist

# 2. Posture Command
# Target posture variables for the balance controller
float64 body_height    # Target body height from the ground (m)
float64 body_roll      # Target roll angle (radians)
float64 body_pitch     # Target pitch angle (radians)
