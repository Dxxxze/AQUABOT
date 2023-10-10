# Copyright 2017, Digi International Inc.
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# Referenced XBee python library example communication code. 


# Author:   Siwen Wang
# School:   University of Arizona
# Course:   ENGR 498 2022 - 2023
# Project:  AQUABOT C3

from digi.xbee.devices import XBeeDevice
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import csv
import time

# Constant Global Variables 
device = XBeeDevice("COM4", 9600) # The XBee connected to the computer
droneAddr = ["0013A2004201562E", "0013A200420155E4", "0013A20042015602", 
             "0013A20042015613", "0013A200420155A0", "0013A20042015611", 
             "0013A20042015616", "0013A20042015632"] # address of all XBee on the drones

drones = [1] # available drones

# This class contains all the function that will be handling file changes
# Specifically, the command.txt file in the current directory
class Handler(FileSystemEventHandler):

    def sendCommand(self, num, command):
        # Global variables
        global remote_devices   # list of remote devices, which will be all available drones
        if remote_devices[num - 1] is None:
            print("Error: drone %d is not connected", num)
            return
        msg = "Drone " + str(num) + " " + command
        print("Sending data [%s] to drone %d..." % (msg, num))
        while True:
            try:
                device.send_data(remote_devices[num - 1], msg)
                break
            except:
                print("message not sent successfully, resending now")

    def broadcast(self, com):
        command = com[0]
        com.pop(0)
        while len(com) > 0: 
            droneNum = int(com.pop(0))
            if droneNum not in drones: continue
            self.sendCommand(droneNum, command)

    def on_modified(self, event):
        # Global variables
        global lastCommand      # cached the last command sent
        global lastTime         # the time that last command was sent
        global remote_devices   # list of remote devices, which will be all available drones
        # detected change on the command.txt file
        if event.src_path == "C:\\Users\\Draco\\OneDrive\\AQUABOT\\command.txt":
            newTime = time.time()
            commandFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\command.txt", "r")
            newCommand = commandFile.readline()
            commandFile.close()
            com = newCommand.split(' ')
            if (newTime - lastTime >= 32 or newCommand != lastCommand) or lastTime == 0 : 
                # regular command
                if com[0] == "START" or com[0] == "STOP" or com[0] == "CANCEL": 
                    self.broadcast(com)
                # Other Command
                elif com[0] == "SensorFrequency" or com[0] == "DroneDistance":
                    self.broadcast(com[0] + " " + com[1])
                # Movement Command
                else:
                    while len(com) > 1:
                        droneNum = com.pop(0)
                        gps = com.pop(0)
                        if int(droneNum) not in drones: 
                            continue
                        self.sendCommand(int(droneNum), gps)
            lastTime = newTime
            lastCommand = newCommand

def connect():
    # Global variables
    global remote_devices

    # Obtain the remote XBee devices from the XBee network
    print("Connecting... ")

    while True:
        xbee_network = device.get_network()
        allDevices = xbee_network.discover_devices("REMOTE")    # temp list
        numDevices = xbee_network.get_number_devices() 
        # put each devices in their corresponding slots
        for i in range (0, numDevices):
            index = droneAddr.index(str(allDevices[i].get_64bit_addr()))
            remote_devices[index] = allDevices[i]
        # check if all available drones are connected
        flag = True
        for i in drones:
            if remote_devices[i - 1] is None: flag = False
        if flag == True: break
        else: print("Connection failed. Retrying now... ")


    # let all drones know what other drones are in the swarm

    # msg = "A: "
    # for i in drones:    
    #     msg += str(i)
    #     msg += " "
    # for i in drones:
    #     device.send_data(remote_devices[i - 1], msg)

    print("Connected")
    print("Waiting for data...\n")


def main():
    # Global variables
    global remote_devices       # list of remote devices, which will be all available drones

    print(" +-----------------------------------------+")
    print(" |               XBee Started              |")
    print(" +-----------------------------------------+\n")

    try:       
        # open the device
        device.open()
        device.flush_queues()

        connect()

        # start receiving data
        while True:
            xbee_message = device.read_data()
            if xbee_message is not None:
                msg = xbee_message.data.decode()
                print("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                        xbee_message.data.decode()))
                # drop invalid data
                data = msg.split()
                
                # if data[1][6] == '0': 
                #     print("skipped")
                #     continue

                # store the new message in the data.txt file of the corresponding drone
                while True:
                    try: 
                        dataFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + data[3] + "\\data.txt", 'w')
                        dataFile.write(msg)
                        dataFile.close()
                        break
                    except: print("prob opened in GUI, try again")
                # store the sensors data
                if (data[4] != "-"): 
                    try: 
                        pHFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + data[3] + "\\pH.csv", 'a', newline = '')
                        csv.writer(pHFile).writerow([data[0], data[1], data[2], data[3], data[4]])
                        pHFile.close()
                    except: print("pH data file opened failed, drone %s data lost" % data[3])
                if (data[5] != "-"): 
                    try:
                        tempFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + data[3] + "\\Temperature.csv", 'a', newline = '')
                        csv.writer(tempFile).writerow([data[0], data[1], data[2], data[3], data[5]])
                        tempFile.close()
                    except: print("temperature data file opened failed, drone %s data lost" % data[3])
                if (data[6] != "-"): 
                    try:
                        salinityFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + data[3] + "\\Salinity.csv", 'a', newline = '')
                        csv.writer(salinityFile).writerow([data[0], data[1], data[2], data[3], data[6]])
                        salinityFile.close()
                    except: print("salinity data file opened failed, drone %s data lost" % data[3])
                # store the GPS data
                try:
                    gpsFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + data[3] + "\\gps.csv", 'a', newline = '')
                    gps = data[7].split(',')
                    csv.writer(gpsFile).writerow([data[0], data[1], data[2], data[3], gps[0], gps[1], data[8]])
                    gpsFile.close()
                except: print("gps data file not opened, drone %s data lost\n" % data[3])
                    
    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    # Global variables
    global lastCommand
    global lastTime
    lastCommand = ""
    lastTime = 0
    global remote_devices       # list of remote devices, which will be all available drones
    remote_devices = [None] * 8
    # initialize file watcher
    observer = Observer()
    observer.schedule(Handler(), "C:\\Users\\Draco\\OneDrive\\AQUABOT") # watch the local directory
    observer.start()
    # start
    try: 
        main()
    finally:
        observer.stop()
        observer.join()