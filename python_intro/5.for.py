sadje = ['jabolko', 'banana', 'češnja']

print("Izpis sadja:")
for sadez in sadje:
    print(sadez)

print("\nŠtevilke od 0 do 4:")
for i in range(5):
    print(i)

print("\nIndeksi in sadeži:")
for indeks, sadez in enumerate(sadje):
    print(f"{indeks}: {sadez}")