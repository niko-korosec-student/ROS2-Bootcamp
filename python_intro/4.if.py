"""
Conditional Statements (if / elif / else)

In robotics, we often make decisions based on sensor data, 
battery levels, or the state of the environment. 
Conditional statements let the program choose between different actions 
depending on the situation.

This example uses age classification for a person, 
but the same logic can be applied to robotics — for example:
- If battery < 20% → return to charging station
- If temperature > 70°C → stop the motor
- If object detected within 10 cm → stop movement
"""

# Example: Classifying a person's age group
age = 45

if age < 18:
    print("The person is a minor.")
elif 18 <= age < 30:
    print("The person is a young adult.")
elif 30 <= age < 65:
    print("The person is an adult.")
else:
    print("The person is a senior citizen.")
