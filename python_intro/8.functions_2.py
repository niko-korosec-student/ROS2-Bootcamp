def pozdrav(ime, priimek=""):
    if priimek:
        return f"Živjo, {ime} {priimek}!"
    return f"Živjo, {ime}!"

def vsota(*stevilke):
    total = 0
    for stevilo in stevilke:
        total += stevilo
    return total

def podatki_o_osebi(ime, starost, **lastnosti):
    info = f"Oseba {ime}, star {starost} let.\n"
    for kljuc, vrednost in lastnosti.items():
        info += f"- {kljuc}: {vrednost}\n"
    return info

def fib(n):
    if n <= 1:
        return n
    return fib(n-1) + fib(n-2)

# Primeri uporabe
print(pozdrav("Niko", "Korošec"))       # z priimkom
print(pozdrav("Ana"))                    # brez priimka

print("Vsota:", vsota(1, 3, 5, 7, 9))  # vsota poljubnega števila argumentov

print(podatki_o_osebi("Marko", 30, poklic="inženir", mesto="Ljubljana", hobi="kolesarjenje"))

print("10. Fibonacci člen je:", fib(10))  # rekurzivna funkcija