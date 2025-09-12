robot = {
    "ime": "UR5e",
    "doseg_m": 0.85,
    "masa_kg": 20,
    "aktiven": True
}

robot["lokacija"] = "Maribor"    # Dodajanje novega ključa in vrednosti
robot["masa_kg"] = 22            # Posodabljanje vrednosti ključa
del robot["aktiven"]             # Brisanje ključa in vrednosti

print("Robot podatki:", robot)  # Izpis celotnega slovarja

# Izpis vseh ključev in vrednosti
for ključ, vrednost in robot.items():
    print(f"{ključ}: {vrednost}")

# Izpis samo ključev
for ključ in robot.keys():
    print("Ključ:", ključ)

# Izpis samo vrednosti
for vrednost in robot.values():
    print("Vrednost:", vrednost)