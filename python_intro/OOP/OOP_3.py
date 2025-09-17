"""
OOP Example 3: State and Overriding Methods

Objects can have state (attributes that change over time). 
Methods update that state, and we can override methods in subclasses.

- Car has brand, year, and speed
- accelerate() and brake() update speed
- ElectricCar inherits Car and adds battery capacity
- __str__ is overridden in ElectricCar to include battery info

This is similar to robotics, where a general Robot class 
could be extended into an ElectricRobot or SCARARobot with extra attributes.
"""

class Car:
    def __init__(self, brand, year, speed=0):
        self.brand = brand
        self.year = year
        self.speed = speed  

    def accelerate(self, amount):
        self.speed += amount
        print(f"{self.brand} accelerates to {self.speed} km/h")

    def brake(self, amount):
        self.speed = max(0, self.speed - amount)
        print(f"{self.brand} slows down to {self.speed} km/h")

    def __str__(self):
        return f"Car: {self.brand}, year: {self.year}, speed: {self.speed} km/h"

class ElectricCar(Car):
    def __init__(self, brand, year, battery_capacity, speed=0):
        super().__init__(brand, year, speed)  
        self.battery_capacity = battery_capacity  

    def __str__(self):
        base_text = super().__str__()
        return f"{base_text}, battery: {self.battery_capacity} kWh"

# Example usage
tesla = ElectricCar("Tesla Model 3", 2023, 75)
print(tesla)

tesla.accelerate(50)
tesla.brake(20)
print(tesla)
