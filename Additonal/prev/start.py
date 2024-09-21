import pyfiglet 
import os
import inquirer as inq
from colorama import init, Fore
from utils import printProgressBar
import rotation as rad

class Start:
   
   def __init__(self):
      # Initialize colorama
      init(autoreset=True) 
      ForeColor = Fore.GREEN
      os.system('cls' if os.name == 'nt' else 'clear')

      print(ForeColor + pyfiglet.figlet_format("Team Automatus", font='slant', justify='center'))
      print(ForeColor + '<<<Eastern University>>>\n\n')

   
if __name__ == '__main__':
   start = Start()

