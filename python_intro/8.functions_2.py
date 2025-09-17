"""
Functions in Robotics (Advanced)

This example demonstrates:
- Default arguments
- Variable number of arguments (*args)
- Keyword arguments (**kwargs)
- Recursive functions

These are very useful in robotics. For example:
- Default values → optional robot parameters
- *args → handling multiple joint positions
- **kwargs → flexible robot configuration
- Recursion → algorithms like path planning or Fibonacci-based timing
"""

def greet(name, surname=""):
    if surname:
        return f"Hello, {name} {surname}!"
    return f"Hello, {name}!"

def add(*numbers):
    total = 0
    for num in numbers:
        total += num
    return total

def robot_info(name, weight, **properties):
    info = f"Robot {name}, weight {weight} kg.\n"
    for key, value in properties.items():
        info += f"- {key}: {value}\n"
    return info

def fib(n):
    if n <= 1:
        return n
    return fib(n-1) + fib(n-2)

# Usage examples
print(greet("Niko", "Korosec"))          # with surname
print(greet("Ana"))                      # without surname

print("Sum:", add(1, 3, 5, 7, 9))        # arbitrary number of arguments

print(robot_info("UR5e", 20, location="Maribor", reach="0.85 m", payload="5 kg"))

print("10th Fibonacci number is:", fib(10))  # recursive function
