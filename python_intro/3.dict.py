"""
Dictionaries

Dictionaries let us store key-value pairs. 
They are useful in robotics when we need to describe a robot, 
its configuration, or any structured data. 
Each piece of information is stored under a descriptive key, 
making it easy to access, update, or remove.
"""

# Define a robot using a dictionary
robot = {
    "name": "UR5e",
    "reach_m": 0.85,
    "mass_kg": 20,
    "active": True
}

# Modify dictionary content
robot["location"] = "Maribor"    # Add new key-value pair
robot["mass_kg"] = 22            # Update value for an existing key
del robot["active"]              # Delete a key-value pair

# Print the whole dictionary
print("Robot data:", robot)

# Print all keys and values
for key, value in robot.items():
    print(f"{key}: {value}")

# Print only keys
for key in robot.keys():
    print("Key:", key)

# Print only values
for value in robot.values():
    print("Value:", value)
