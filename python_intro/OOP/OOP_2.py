"""
OOP Example 2: Inheritance

Inheritance allows a class to reuse and extend another class.

- ColoredCircle inherits all attributes and methods from Circle
- super().__init__ calls the parent constructor so we don’t duplicate code
- We add a new attribute (color)
- We override __str__ to include color in the output

In robotics, this is similar to creating a generic Robot class and then extending it into 
a MobileRobot or IndustrialRobot with additional attributes.
"""

import math

class Circle:
    def __init__(self, radius):
        self.radius = radius

    def area(self):
        return math.pi * (self.radius ** 2)

    def __str__(self):
        return f"Circle with radius {self.radius:.2f}, area {self.area():.2f}"

class ColoredCircle(Circle):
    def __init__(self, radius, color):
        super().__init__(radius)  
        self.color = color       

    def __str__(self):
        base_text = super().__str__()  
        return f"{base_text}, color: {self.color}"

# Example usage
colored_circle1 = ColoredCircle(7, "blue")
print(colored_circle1)
