"""
Lists

In robotics, we often work with collections of items:
- Robot components (motors, sensors, controllers, batteries, etc.)
- A sequence of waypoints in a path
- A list of joint angles or sensor readings

Python lists allow us to store, modify, and iterate over such data easily.
This example shows how to manage a list of robot components.
"""

# Define initial list of robot components
components = ["motor", "sensor", "controller"]

# Modify the list
components.append("battery")         # Add element at the end
components.insert(0, "RAM")          # Insert element at the beginning
components.remove("sensor")          # Remove a specific element

# Access elements
first = components[0]                # First element
last = components[-1]                # Last element
components[1] = "new controller"     # Modify element at index 1

# Output
print(f"Components ({len(components)}):")
for c in components:                 # Iterate through all elements
    print(f"- {c}")

print(f"First: {first}, Last: {last}")
