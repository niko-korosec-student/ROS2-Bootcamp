"""
1. __init__ metoda in self

    __init__ je posebna metoda, ki se avtomatsko kliče, ko ustvarimo nov objekt razreda.

    self predstavlja konkretni objekt, ki ga ustvarjamo, in preko njega dostopamo do lastnosti in metod objekta.

    V __init__ nastavimo začetne vrednosti lastnosti (atributov).

2. Dodajanje lastnosti in logike

V razred lahko dodajamo lastnosti (atribute) in funkcije (metode), ki izvajajo logiko. To nam omogoča, da podatke in funkcionalnost združimo skupaj.
3. Override metode

Metode lahko v podrazredih (ali tudi v samem razredu) prepišemo (override), da spremenimo njihovo delovanje, če želimo.
"""

class Avto:
    def __init__(self, znamka, letnik, hitrost=0):
        self.znamka = znamka
        self.letnik = letnik
        self.hitrost = hitrost  # začetna hitrost

    def pospesi(self, povečava):
        self.hitrost += povečava
        print(f"Avto {self.znamka} pospeši na {self.hitrost} km/h")

    def zaviraj(self, zmanjšava):
        self.hitrost = max(0, self.hitrost - zmanjšava)
        print(f"Avto {self.znamka} upočasni na {self.hitrost} km/h")

    def __str__(self):
        return f"Avto: {self.znamka}, letnik: {self.letnik}, trenutna hitrost: {self.hitrost} km/h"

class ElektricniAvto(Avto):
    def __init__(self, znamka, letnik, baterija_kapaciteta, hitrost=0):
        super().__init__(znamka, letnik, hitrost)  # pokličemo init iz nadrazreda
        self.baterija_kapaciteta = baterija_kapaciteta  # dodamo lastnost baterije

    # Override metode __str__, da vključimo baterijo
    def __str__(self):
        osnovni_tekst = super().__str__()
        return f"{osnovni_tekst}, baterija: {self.baterija_kapaciteta} kWh"

# Uporaba:
tesla = ElektricniAvto("Tesla Model 3", 2023, 75)

print(tesla)
tesla.pospesi(50)
tesla.zaviraj(20)
print(tesla)


"""
Kaj vidimo v tem primeru?

    __init__ nastavi osnovne lastnosti avtomobila.

    self omogoča, da delo poteka znotraj objekta.

    V podrazredu ElektricniAvto dodamo novo lastnost baterija_kapaciteta.

    Override metode __str__ nam omogoči prikaz dodatnih informacij.

    Metode pospesi in zaviraj vsebujejo logiko za spreminjanje stanja objekta.
"""