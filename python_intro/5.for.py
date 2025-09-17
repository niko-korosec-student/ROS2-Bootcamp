"""
For Loops
For loops let us repeat actions for each element in a list or for a range of numbers.
In robotics, this is useful for:
- Iterating through a list of robot components
- Sending commands for multiple waypoints
- Reading sensor data at different indices
"""

# Example 1: Iterate over robot components
components = ["motor", "controller", "camera"]

print("Robot components:")
for comp in components:
    print(comp)

# Example 2: Iterate over a range of numbers
print("\nJoint indices (0–4):")
for i in range(5):
    print(f"Joint {i}")

# Example 3: Iterate with index and value
print("\nIndexed components:")
for index, comp in enumerate(components):
    print(f"{index}: {comp}")
