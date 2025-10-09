import subprocess

def pwmsetup(wPin, frecuencia=1000, cycleregister = 2000):
    subprocess.run(['gpio', 'pwmr', str(wPin), str(cycleregister)])
    subprocess.run(['gpio', 'pwmTone', str(wPin), str(frecuencia)])
    
def pwmdutty(wPin, dutty):
    subprocess.run(['gpio', 'pwm', str(wPin), str(dutty*10)])
