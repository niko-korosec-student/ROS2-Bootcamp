"""
Functions (Intro)

Functions let us group reusable blocks of code. 
In robotics, functions are useful for tasks like:
- Greeting/logging messages
- Performing mathematical calculations
- Converting sensor data
"""

def greet(name):
    return f"Hello, {name}!"

def add(a, b):
    return a + b

# Function calls
print(greet("Niko"))
result = add(5, 7)
print(f"The sum of 5 and 7 is: {result}")
