"""
While Loops

Unlike for-loops (which repeat a fixed number of times), 
while-loops repeat until a condition is no longer true. 
This makes them useful in robotics for:
- Repeatedly checking sensor data until a condition is met
- Running a control loop until a stop signal is received
- Simulating "do-while" loops where the action must happen at least once
"""

print("Example: simple while loop")
i = 0
while i < 5:
    print(f"while: number is {i}")
    i += 1

print("\nExample: simulated do-while loop")
j = 0
while True:
    print(f"do-while: number is {j}")
    j += 1
    if j >= 5:
        break

print("\nExample: control loop waiting for a stop command")
while True:
    user_input = input("Enter a command (type 'stop' to end): ")
    if user_input.lower() == "stop":
        print("Loop ended.")
        break
    else:
        print(f"Command received: {user_input}")
