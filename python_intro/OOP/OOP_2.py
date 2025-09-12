import math

class Krog:
    def __init__(self, radij):
        self.radij = radij

    def ploscina(self):
        return math.pi * (self.radij ** 2)

    def __str__(self):
        return f"Krog z radijem {self.radij:.2f} in površino {self.ploscina():.2f}"

class BarvniKrog(Krog):
    def __init__(self, radij, barva):
        super().__init__(radij)  # Pokličemo konstruktor nadrazreda Krog
        self.barva = barva       # Dodamo novo lastnost barva

    def __str__(self):
        osnovni_tekst = super().__str__()  # Pokličemo __str__ iz nadrazreda
        return f"{osnovni_tekst}, barva: {self.barva}"

# Ustvarimo objekt barvni_krog1 z radijem 7 in barvo 'modra'
barvni_krog1 = BarvniKrog(7, "modra")

print(barvni_krog1)


"""
Razlaga:

    Dedovanje (inheritance)
    BarvniKrog(Krog) pomeni, da BarvniKrog podeduje vse lastnosti in metode od razreda Krog.

    super().__init__(radij)
    S tem pokličemo konstruktor nadrazreda Krog, da inicializira radij. Tako ne podvajamo kode.

    Dodana lastnost barva
    BarvniKrog razširi Krog z dodatno lastnostjo barve.

    Override metode __str__
    V BarvniKrog prilagodimo izpis, da dodamo tudi barvo, hkrati pa ohranimo izpis iz nadrazreda.

    Ustvarjanje in izpis objekta barvni_krog1
    Ustvarimo krog z radijem 7 in barvo "modra", nato ga izpišemo.

"""