from start import Start
from second_menu import SecondMenu
from game import Game
import rotation as rad 

# initial setup 
start = Start()


print('Lets Go...')
rad_o = rad.Accelerations()

menu = SecondMenu()
   
def full_test():
    pass

def motor_test():
    pass

def acce_test():
    pass

def gamestart():    
    game = Game(rad_o)        
    game.start()
    
if menu.choice == "Full Test":
    full_test()
elif menu.choice == "Test Motors":
    motor_test()
elif menu.choice == "Test Accelerometer":
    acce_test()
elif menu.choice == "Start Program":
    gamestart()