from time import sleep
def test():
    i = 0
    while i < 10:
        yield(i)
        sleep(0.5)
        i += 1

for k in test():
    print(k)