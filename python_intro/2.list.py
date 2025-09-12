komponente = ["motor", "senzor", "krmilnik"]

komponente.append("baterija")     # Dodaj element na konec
komponente.insert(0, "ram")       # Vstavi element na začetek
komponente.remove("senzor")       # Odstrani določen element

prva = komponente[0]              # Prvi element
zadnja = komponente[-1]           # Zadnji element
komponente[1] = "nov krmilnik"    # Sprememba elementa na indeksu 1

print(f"Komponente ({len(komponente)}):")
for k in komponente:              # Izpiši vse elemente
    print(f"- {k}")

print(f"Prva: {prva}, Zadnja: {zadnja}")
