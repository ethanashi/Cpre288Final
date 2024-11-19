# Author: Phillip Jones
# Date: 10/30/2023
# Description: Client starter code that combines: 1) Simple GUI, 2) creation of a thread for
#              running the Client socket in parallel with the GUI, and 3) Simple recieven of mock sensor 
#              data for a server/cybot.for collecting data from the cybot.

# General Python tutorials (W3schools):  https://www.w3schools.com/python/

# Serial library:  https://pyserial.readthedocs.io/en/latest/shortintro.html 
import serial 
import time # Time library   
# Socket library:  https://realpython.com/python-sockets/  
# See: Background, Socket API Overview, and TCP Sockets  
import socket
import tkinter as tk # Tkinter GUI library
# Thread library: https://www.geeksforgeeks.org/how-to-use-thread-in-tkinter-python/
import threading
import os  # import function for finding absolute path to this python script

#import CyBot_Plot_Sensor_Scan_Values_From_File
import subprocess 
##### START Define Functions  ########

# Main: Mostly used for setting up, and starting the GUI
def main():

        global window  # Made global so quit function (send_quit) can access
        window = tk.Tk() # Create a Tk GUI Window
        window.minsize(500, 500)
        window.configure(bg = "#F85E8C")

        # Last command label  
        global Last_command_Label  # Made global so that Client function (socket_thread) can modify
        Last_command_Label = tk.Label(text="Last Command Sent: ") # Creat a Label
        Last_command_Label.pack() # Pack the label into the window for display

        # Quit command Button
        quit_command_Button = tk.Button(text ="Press to Quit", command = send_quit, 
                relief = "raised", 
                borderwidth = 10, 
                bg = "dark blue", 
                fg = "white")
        quit_command_Button.pack()  # Pack the button into the window for display

        # Cybot Scan command Button
        scan_command_Button = tk.Button(text ="Press to Scan", command = send_scan, 
                relief = "raised", 
                borderwidth = 10, 
                bg = "dark blue", 
                fg = "white")
        scan_command_Button.pack() # Pack the button into the window for display


        # Create a Thread that will run a fuction assocated with a user defined "target" function.
        # In this case, the target function is the Client socket code
        my_thread = threading.Thread(target=socket_thread) # Creat the thread
        my_thread.start() # Start the thread


        frame1 = tk.Frame(
                master=window, 
                bg = "dark blue", 
                relief = "raise", 
                borderwidth = 10 


        )
        frame1.pack(side = "left") 
        move_label1 = tk.Label (
                master=frame1, 
                text = "^", 
                font = ("Arial", 30), 
                bg = "dark blue", 
                fg = "white"
        )
        move_label1.pack()
        move_label2 = tk.Label(
                master = frame1, 
                text = ">", 
                font = ("Arial", 30),
                bg = "dark blue", 
                fg = "white"
        ) 
        move_label2.pack(side = "right")
        move_label3 = tk.Label(
                master = frame1, 
                text = "<", 
                font = ("Arial", 30),
                bg = "dark blue", 
                fg = "white"
        )
        move_label3.pack(side = "left")
        move_label4 = tk.Label(
                master = frame1,
                text = "v", 
                font = ("Arial", 30), 
                bg = "dark blue", 
                fg = "white"
        )
        move_label4.pack(side = "bottom") 

        movement_label = tk.Label( 
                master=frame1, 
                text="w to move forward\n s to move backward\n a to rotate left\n d to rotate right", 
                bg = "dark blue", 
                fg = "white"
        )
        movement_label.pack()

        autonomous_button = tk.Button(
                text = "start the bot.\n turn the bot back to autonomous mode", 
                command = start_bot, 
                relief = "raised", 
                borderwidth = 10, 
                bg = "dark blue", 
                fg = "white"
        )
        autonomous_button.pack(side = "right")

        window.bind('<w>', forward) 
        window.bind('<s>', backward)
        window.bind('<a>', left)
        window.bind('<d>', right)

        # Start event loop so the GUI can detect events such as button clicks, key presses, etc.
        window.mainloop()

def right(event): 
        global gui_send_message
        gui_send_message = "R\n"
def left(event): 
        global gui_send_message
        gui_send_message = "L\n"
def forward(event): 
        global gui_send_message
        gui_send_message = "F\n"
def backward(event): 
        global gui_send_message
        gui_send_message = "B\n"

def start_bot():
        global gui_send_message
        gui_send_message = "A\n"

# Quit Button action.  Tells the client to send a Quit request to the Cybot, and exit the GUI
def send_quit():
        global gui_send_message # Command that the GUI has requested be sent to the Cybot
        global window  # Main GUI window
        
        gui_send_message = "quit\n"   # Update the message for the Client to send
        time.sleep(1)
        window.destroy() # Exit GUI

def keyPressFunction(event): 
        print("key pressed", event.char)


# Scan Button action.  Tells the client to send a scan request to the Cybot
def send_scan():
        global gui_send_message # Command that the GUI has requested sent to the Cybot
        
        gui_send_message = "M\n"   # Update the message for the Client to send

        #CyBot_Plot_Sensor_Scan_Values_From_File.absolute_path
        subprocess.run(["python", "CyBot_Plot_Sensor_Scan_Values_From_File.py"])





# Client socket code (Run by a thread created in main)
def socket_thread():
        # Define Globals
        global Last_command_Label # GUI label for displaying the last command sent to the Cybot
        global gui_send_message   # Command that the GUI has requested be sent to the Cybot

        # A little python magic to make it more convient for you to adjust where you want the data file to live
        # Link for more info: https://towardsthecloud.com/get-relative-path-python 
        absolute_path = os.path.dirname(__file__) # Absoult path to this python script
        relative_path = "./"   # Path to sensor data file relative to this python script (./ means data file is in the same directory as this python script)
        full_path = os.path.join(absolute_path, relative_path) # Full path to sensor data file
        filename = 'sensor-scan.txt' # Name of file you want to store sensor data from your sensor scan command

        # Choose to create either a UART or TCP port socket to communicate with Cybot (Not both!!)
        # UART BEGIN
        # cybot = serial.Serial('COM100', 115200)  # UART (Make sure you are using the correct COM port and Baud rate!!)
        # UART END

        # TCP Socket BEGIN (See Echo Client example): https://realpython.com/python-sockets/#echo-client-and-server
        HOST = "127.0.0.1"  # The server's hostname or IP address
        PORT = 65432        # The port used by the server
        cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a socket object
        cybot_socket.connect((HOST, PORT))   # Connect to the socket  (Note: Server must first be running)
                      
        cybot = cybot_socket.makefile("rbw", buffering=0)  # makefile creates a file object out of a socket:  https://pythontic.com/modules/socket/makefile
        # TCP Socket END

        # Send some text: Either 1) Choose "Hello" or 2) have the user enter text to send
        send_message = "Hello\n"                            # 1) Hard code message to "Hello", or
        # send_message = input("Enter a message:") + '\n'   # 2) Have user enter text
        gui_send_message = "wait\n"  # Initialize GUI command message to wait                                

        cybot.write(send_message.encode()) # Convert String to bytes (i.e., encode), and send data to the server

        print("Sent to server: " + send_message) 

        # Send messges to server until user sends "quit"
        while send_message != 'quit\n':
                
                # Update the GUI to display command being sent to the CyBot
                command_display = "Last Command Sent:\t" + send_message
                Last_command_Label.config(text = command_display)  
        
                # Check if a sensor scan command has been sent
                if (send_message == "M\n") or (send_message == "m\n"):

                        print("Requested Sensor scan from Cybot:\n")
                        rx_message = bytearray(1) # Initialize a byte array

                        # Create or overwrite existing sensor scan data file
                        file_object = open(full_path + filename,'w') # Open the file: file_object is just a variable for the file "handler" returned by open()

                        while (rx_message.decode() != "END\n"): # Collect sensor data until "END" recieved
                                rx_message = cybot.readline()   # Wait for sensor response, readline expects message to end with "\n"
                                file_object.write(rx_message.decode())  # Write a line of sensor data to the file
                                print(rx_message.decode()) # Convert message from bytes to String (i.e., decode), then print

                        file_object.close() # Important to close file once you are done with it!!                

                else:                
                        print("Waiting for server reply\n")
                        rx_message = cybot.readline()      # Wait for a message, readline expects message to end with "\n"
                        print("Got a message from server: " + rx_message.decode() + "\n") # Convert message from bytes to String (i.e., decode)


                # Choose either: 1) Idle wait, or 2) Request a periodic status update from the Cybot
                # 1) Idle wait: for gui_send_message to be updated by the GUI
                while gui_send_message == "wait\n": 
                        time.sleep(.1)  # Sleep for .1 seconds
                send_message = gui_send_message

                # 2) Request a periodic Status update from the Cybot:
                # every .1 seconds if GUI has not requested to send a new command
                #time.sleep(.1)
                #if(gui_send_message == "wait\n"):   # GUI has not requested a new command
                #        send_message = "status\n"   # Request a status update from the Cybot
                #else:
                #        send_message = gui_send_message  # GUI has requested a new command

                gui_send_message = "wait\n"  # Reset gui command message request to wait                        

                cybot.write(send_message.encode()) # Convert String to bytes (i.e., encode), and send data to the server
                
        print("Client exiting, and closing file descriptor, and/or network socket\n")
        time.sleep(2) # Sleep for 2 seconds
        cybot.close() # Close file object associated with the socket or UART
        cybot_socket.close()  # Close the socket (NOTE: comment out if using UART interface, only use for network socket option)

##### END Define Functions  #########


### Run main ###
main()