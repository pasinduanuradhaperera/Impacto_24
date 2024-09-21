import time
from colorama import init, Fore

def frame_rate(instance):
    etime = time.time() - instance.start_time if hasattr(instance, 'start_time') else 0
    frame_rate = 1 / (etime + 1e-6)  # avoid division by zero
    print('Frame rate is: %.2f fps' % frame_rate)
    instance.start_time = time.time()

def printProgressBar(precent):
    if precent < 101 and precent > -1:       
        bar_length = 50
        filled_length = int(bar_length * precent // 100)
        bar = '█' * filled_length + '-' * (bar_length - filled_length)
        print(Fore.GREEN + f'\rLoading: |{bar}| {round(precent,1)}% Completed', end='\r')
        if precent == 100:
            print()

