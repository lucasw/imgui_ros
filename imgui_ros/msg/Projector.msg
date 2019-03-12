bool remove  # False
# camera width and height aren't used, the texture in texture_name defines that
imgui_ros/Camera camera
# stop projecting beyond this range - TODO(lucasw) redundant with Camera far?
float32 max_range
float32 constant_attenuation
float32 linear_attenuation
float32 quadratic_attenuation
# TODO(lucasw) these would be interesting to implement
# Combine the focal_distance with aov
# to make image blur scale with distance
# float focal_distance
# float aperture ?
