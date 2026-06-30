# ==========================
# IK + FK + Arduino Mapping for 3-DOF Robot Arm
# ==========================

import math

# ==========================
# INPUTS
# ==========================
# Target point in robot frame (cm)
x_target = 11
y_target = 3.5
# Z Is really a 6th parameter for us
z_target = -8.4

# Will result in:
#Base servo: 17.65°
#Shoulder servo: 174.17°
#Elbow servo: 77.67°

# Arm link lengths (cm)
L1 = 7.75  # Shoulder → Elbow
L2 = 11  # Elbow → Wrist/Gripper

# Elbow configuration: choose "down" or "up"
elbow_config = "down"  # or "up"

# Servo zero positions (physical calibration)
BASE_ZERO = 0        # Base servo value at IK base = 0
SHOULDER_ZERO = 260  # (servo 0 is on left side so add 240 to right side!)
## WHEN WE GO TO THE LEFT SIDE WE WILL NEED TO JUST REARRANGE THIS SHOULDER MATH A BIT

ELBOW_ZERO = 160     # (straight to sky THEN subtract the angle! )


# ==========================
# IK CALCULATION
# ==========================

# 1. Base yaw angle (rotation around vertical axis)
theta_base = math.atan2(y_target, x_target)  # radians

# 2. Distance from shoulder pivot to target in r-z plane
l = math.sqrt(x_target**2 + y_target**2)  # horizontal distance
z = z_target                               # vertical distance from shoulder pivot
h = math.sqrt(l**2 + z**2)                 # straight-line distance from shoulder to target

# 3. Check reachability
if h > (L1 + L2):
    print(f"Target out of reach! h={h}, L1+L2={L1+L2}")
    #raise ValueError("Target is out of reach of the arm!")

# 4. Elbow angle (cosine law)
cos_theta_elbow = (h**2 - L1**2 - L2**2) / (2 * L1 * L2)
cos_theta_elbow = max(min(cos_theta_elbow, 1), -1)  # clamp to [-1,1]

if elbow_config == "down":
    theta_elbow = math.acos(cos_theta_elbow)
else:
    theta_elbow = -math.acos(cos_theta_elbow)

# 5. Shoulder angle
theta_shoulder = math.atan2(z, l) - math.atan2(L2 * math.sin(theta_elbow), L1 + L2 * math.cos(theta_elbow))

# Convert IK angles to degrees
theta_base_deg = math.degrees(theta_base)
theta_shoulder_deg = math.degrees(theta_shoulder)
theta_elbow_deg = math.degrees(theta_elbow)

print("=== IK RESULTS ===")
print(f"Target (x, y, z): ({x_target}, {y_target}, {z_target})")
print(f"Base yaw (theta_base): {theta_base_deg:.2f}°")
print(f"Shoulder pitch (theta_shoulder): {theta_shoulder_deg:.2f}°")
print(f"Elbow pitch (theta_elbow): {theta_elbow_deg:.2f}°")

# ==========================
# MAP IK TO ARDUINO SERVO VALUES
# ==========================

servoBase = BASE_ZERO + theta_base_deg
servoShoulder = SHOULDER_ZERO + theta_shoulder_deg
servoElbow = ELBOW_ZERO - theta_elbow_deg  # elbow often inverted

# Clamp servo angles to 0-200 (example)
# servoBase = max(0, min(200, servoBase))
# servoShoulder = max(0, min(200, servoShoulder))
# servoElbow = max(0, min(200, servoElbow))

print("\n=== ARDUINO SERVO VALUES ===")
print(f"Base servo: {servoBase:.2f}°")
print(f"Shoulder servo: {servoShoulder:.2f}°")
print(f"Elbow servo: {servoElbow:.2f}°")

# ==========================
# FK CALCULATION (to verify IK)
# ==========================

# Convert degrees back to radians for FK
theta_base_rad = math.radians(theta_base_deg)
theta_shoulder_rad = math.radians(theta_shoulder_deg)
theta_elbow_rad = math.radians(theta_elbow_deg)

# 1. Planar reach in shoulder-elbow plane
r = L1 * math.cos(theta_shoulder_rad) + L2 * math.cos(theta_shoulder_rad + theta_elbow_rad)
z_fk = L1 * math.sin(theta_shoulder_rad) + L2 * math.sin(theta_shoulder_rad + theta_elbow_rad)

# 2. Rotate into XY plane using base
x_fk = r * math.cos(theta_base_rad)
y_fk = r * math.sin(theta_base_rad)

print("\n=== FK VERIFICATION ===")
print(f"FK Result: x={x_fk:.2f}, y={y_fk:.2f}, z={z_fk:.2f}")
print(f"Target:    x={x_target}, y={y_target}, z={z_target}")

