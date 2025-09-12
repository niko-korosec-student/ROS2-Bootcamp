import math

class Krog:
    def __init__(self, radij):
        self.radij = radij  # Inicializacija lastnosti radij

    def ploscina(self):
        return math.pi * (self.radij ** 2)  # Izračun površine kroga

    def spremeni_radij(self, nov_radij):
        self.radij = nov_radij  # Posodobitev lastnosti radij

    def __str__(self):
        return f"Krog z radijem {self.radij:.2f} in površino {self.ploscina():.2f}"

# Ustvarimo objekt krog1
krog1 = Krog(5)

# Izpišemo podatke o krog1
print(krog1)

# Spremenimo radij in ponovno izpišemo podatke
krog1.spremeni_radij(10)
print(krog1)


"""Razlaga:

    import math
    Uvozimo Pythonovo knjižnico za matematične funkcije, da lahko uporabimo math.pi.

    Razred Krog
    Definiramo razred, ki predstavlja krog z lastnostjo radij.

    Metoda __init__
    To je konstruktor. Ko ustvarimo nov Krog, nastavi začetni radij (npr. 5).

    Metoda ploscina
    Izračuna površino kroga s formulo π * r². Ta metoda nima parametrov razen self, ker uporablja lastnost objekta.

    Metoda spremeni_radij
    Omogoča, da spremenimo radij kroga po kreiranju objekta.

    Metoda __str__
    Definira, kako naj se objekt predstavi, ko ga poskušamo izpisati (print). Tukaj lepo zaokroži radij in površino na 2 decimalni mesti.

    Ustvarjanje objekta krog1
    S klicem Krog(5) naredimo krog s polmerom 5.

    print(krog1)
    Samodejno kliče metodo __str__ in izpiše podrobnosti o krogu.

    Sprememba radija in ponoven izpis
    Pokličemo metodo spremeni_radij z novim radijem 10 in ponovno izpišemo stanje objekta.
    """