"""
OOP Example 1: Classes and Objects

A class is a blueprint for creating objects. 
Objects combine attributes (data) and methods (functions). 
In this example, we model a circle:

- __init__ : constructor, sets the radius when a circle is created
- area()  : computes the area of the circle
- change_radius() : updates the radius
- __str__ : controls how the object is displayed with print()

This shows encapsulation: keeping data (radius) and logic (area calculation) together.
"""

import math

class Circle:
    def __init__(self, radius):
        self.radius = radius  

    def area(self):
        return math.pi * (self.radius ** 2)  

    def change_radius(self, new_radius):
        self.radius = new_radius  

    def __str__(self):
        return f"Circle with radius {self.radius:.2f}, area {self.area():.2f}"

# Example usage
circle1 = Circle(5)
print(circle1)

circle1.change_radius(10)
print(circle1)
