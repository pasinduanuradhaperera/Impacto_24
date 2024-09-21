import inquirer as inq
import os
from game import Game


class SecondMenu: 
    def __init__(self):
        #  os.system('cls' if os.name == 'nt' else 'clear')          
        self.question2 = [
               inq.List('Option2', message="Choose What to do: ", choices=["Full Test", "Test Motors", "Test Accelerometer","Test Encoder Distance " ,"Start Program"]),
            ]
        self.choice = ''
        self.choice = inq.prompt(self.question2)['Option2']

if __name__ == '__main__':
    second_menu = SecondMenu()
