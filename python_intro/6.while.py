print("Primer while zanke:")
i = 0
while i < 5:
    print(f"while: Število je {i}")
    i += 1

print("\nPrimer simulacije do-while zanke:")
j = 0
while True:
    print(f"do-while: Število je {j}")
    j += 1
    if j >= 5:
        break

print("\nVpiši 'stop' za konec programa.")
while True:
    user_input = input("Vnesi nekaj: ")
    if user_input.lower() == "stop":
        print("Konec zanke.")
        break
    else:
        print(f"Vnesel si: {user_input}")
