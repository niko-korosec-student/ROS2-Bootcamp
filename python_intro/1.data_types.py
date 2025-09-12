# Podatki o robotu
robot_ime = "UR5e"
doseg_v_m = 0.85          # float
masa_v_kg = 20            # int
je_aktiven = True         # bool

# Izračun nosilnosti
nosilnost_v_kg = 5
varnostni_faktor = 0.8
max_varna_teza = nosilnost_v_kg * varnostni_faktor

# Odločitev glede naloge
naloga_teza = 3.5
lahko_izvede = naloga_teza <= max_varna_teza and je_aktiven

print(f"Robot: {robot_ime}")
print(f"Doseg: {doseg_v_m} m")
print(f"Največja varna teža: {max_varna_teza} kg")
print(f"Lahko izvede nalogo? {lahko_izvede}") #f omogoči lepšo sintakso
print(str(robot_ime) + " ima doseg: " + str(doseg_v_m)) # več možnosti za napako
