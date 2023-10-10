# Copyright 2017, Digi International Inc.
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from digi.xbee.devices import XBeeDevice
import random
import time
import schedule
from datetime import datetime

global device
global remote_device

gpsFrequency = 1
sensorsFrequency = 3 # unit in second
drones = [1, 2, 3]

def generateData(count):
    date = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
    data = date

    for i in drones:
        data += " Drone " + str(i)
        latitude = round(random.uniform(-90, 90), 6)
        longitude = round(random.uniform(-180, 180), 6)
        if count % sensorsFrequency == 0:
            pH = round(random.uniform(0, 14), 1)
            temp = round(random.uniform(-5, 50), 1)
            cond = round(random.uniform(3, 6), 1)
            data += " " + str(pH) + " " + str(temp) + " " + str(cond) + " "
        else:
            data += " - - - "
        data += str(latitude) + "," + str(longitude)

    return data

def main():
    print(" +--------------------------------------+")
    print(" | XBee Python Library Send Data Sample |")
    print(" +--------------------------------------+\n")

    device = XBeeDevice("COM5", 9600)

    try:
        device.open()
        device.flush_queues()
        print("Waiting for data...\n")

        # Obtain the remote XBee device from the XBee network.
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device("ALPHA")
        while remote_device is None:
            print("try again")
            remote_device = xbee_network.discover_device("ALPHA")
        print("connected")

        count = 0
        oldTime = time.time()
        while True:
            currTime = time.time()
            if (currTime - oldTime >= 1):
                oldTime = currTime
                data = generateData(count)
                print("Sending data to %s..." % (remote_device.get_64bit_addr()))
                device.send_data(remote_device, data)
                print("Success")
                count += 1

                break

            xbee_message = device.read_data()
            if xbee_message is not None:
                print("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                         xbee_message.data.decode()))
            
            
            time.sleep(0.9)

    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    main()