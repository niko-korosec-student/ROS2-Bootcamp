"""
Data Types

In robotics, we often deal with different types of information:
- Strings → to store names of robots or components.
- Integers → to store whole numbers like mass or number of joints.
- Floats → to store continuous values like reach, speed, or precision.
- Booleans → to represent states (active/inactive, safe/unsafe, etc.).

This exercise demonstrates how to use different data types 
together to make decisions about a robot's task feasibility.
"""

# Robot information
robot_name = "UR5e"           # string
reach_m = 0.85                # float (meters)
mass_kg = 20                  # int (kilograms)
is_active = True              # bool (status of the robot)

# Payload calculation
payload_capacity_kg = 5       # max payload in kg
safety_factor = 0.8           # reduce to ensure safe operation
max_safe_payload = payload_capacity_kg * safety_factor

# Task requirement
task_weight = 3.5             # weight of the part to be picked in kg
can_perform = task_weight <= max_safe_payload and is_active

# Output
print(f"Robot: {robot_name}")
print(f"Reach: {reach_m} m")
print(f"Maximum safe payload: {max_safe_payload} kg")
print(f"Can perform task? {can_perform}")

# Alternative string concatenation (less safe, prone to type errors)
print(robot_name + " has a reach of: " + str(reach_m) + " m")
