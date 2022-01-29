import numpy as np
import math

def inRange(x, lower, upper):
    return x >= lower and x <= upper

# All distances are measured in inches
# All angles are measured in degrees

# Height of the vision tape above the ground
vision_target_height = 101.625

# Minimum expected shooting distance
min_d = 42 + 16.285
# Maximum expected shooting distance (inches)
max_d = 192.4 + 16.285

# Minimum mounting height to test
min_h = 26
# Maximum mounting height to test
max_h = 35
# Step size for range of heights
h_step = 0.5

# Minimum mounting angle to test
min_theta = 25
# Maximum mounting angle to test
max_theta = 70
# Step size for range of angles
theta_step = 0.5

# List of valid configurations
# Valid means the Limelight can see the target from the minimum distance and maximum distance
valid_configs = []

for h in np.arange(min_h, max_h, h_step):
    for t in np.arange(min_theta, max_theta, theta_step):
        # Lower height within Limelight's visibility when shooting from min distance
        lower_d_vmin = min_d * math.tan(math.radians(t - 20.5)) + h
        # Upper height within Limelight's visibility when shooting from min distance
        lower_d_vmax = min_d * math.tan(math.radians(t + 20.5)) + h

        # Lower height within Limelight's visibility when shooting from max distance
        upper_d_vmin = max_d * math.tan(math.radians(t - 20.5)) + h
        # Upper height within Limelight's visibility when shooting from max distance
        upper_d_vmax = max_d * math.tan(math.radians(t + 20.5)) + h

        if inRange(vision_target_height, lower_d_vmin, lower_d_vmax) \
            and inRange(vision_target_height, upper_d_vmin, upper_d_vmax):
                # Scoring is based on how close the target is to the center of the crosshair
                lower_d_crosshair_center = (lower_d_vmin + lower_d_vmax) / 2
                lower_d_crosshair_offset = abs(vision_target_height - lower_d_crosshair_center)

                upper_d_crosshair_center = (upper_d_vmin + upper_d_vmax) / 2
                upper_d_crosshair_offset = abs(vision_target_height - upper_d_crosshair_center)

                score = (lower_d_crosshair_offset + upper_d_crosshair_offset) / 2

                valid_configs.append({'h': h, 't': t, 'score': score})

print('Angle is the angle between the Limelight lens and the horizontal, measured in degrees')
print('Height is the height from the ground to the center of the Limelight lens, measured in inches')
print('Possible configurations (in order from best to worst)')

sorted_valid_configs = sorted(valid_configs, key = lambda i: i['score'])

for i, config in enumerate(sorted_valid_configs):
    print(str(i + 1) + " " + str(config))
