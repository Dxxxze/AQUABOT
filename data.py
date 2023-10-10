import random
import time
from datetime import datetime

drones = [1, 2, 3, 4, 5, 6, 7, 8]

# lat = 32.209873
# lon = -110.922644
# lats = [32.209873, 32.209873, 32.209873, 32.209873, 
#         32.209973, 32.209973, 32.209973, 32.209973]
# lons = [-110.922644, -110.922744, -110.922844, -110.922944, 
#         -110.922644, -110.932644, -110.942644, -110.952644]
lats = [32.209873, 32.210873, 32.211873, 32.212873, 32.213873, 32.214873, 32.215873, 32.216873]
lons = [-110.922644, -110.922644, -110.922644, -110.922644, -110.922644, -110.922644, -110.922644, -110.922644]

for i in range(2000):
    for j in drones: 
        # pHFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + str(j) + "\\pH.txt", 'a')
        # tempFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + str(j) + "\\Temperature.txt", 'a')
        # salFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + str(j) + "\\Salinity.txt", 'a')
        # gpsFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + str(j) + "\\GPS.txt", 'a')
        

        date = "MST " + datetime.now().strftime("%Y/%m/%d %H:%M:%S") + " "
        # MST YYYY/MM/DD droneNum HH:mm:SS pH 0.0
        pH = round(random.uniform(0, 14), 1)
        # pHToWrite = date + str(j) + " pH " + str(pH) + "\n"
        # # MST YYYY/MM/DD droneNum HH:mm:SS Temperature 0.0
        temp = round(random.uniform(-5, 50), 1)
        # tempToWrite = date + str(j) + " Temperature " + str(temp) + "\n"
        # # MST YYYY/MM/DD droneNum HH:mm:SS Salinity 0.0
        sal = round(random.uniform(0, 1000), 1)
        # salToWrite = date + str(j) + " Salinity " + str(sal) + "\n"
        # # MST YYYY/MM/DD droneNum HH:mm:SS GPS Coordinate 0.000000,0.000000 Bearing 0
        
        # lats[j - 1] += 0.000001
        # lons[j - 1] += 0.000001
        bearing = 45

        # # bearing = random.randint(0, 360)
        # gpsToWrite = date + str(j) + " GPS Coordinate " + str(lats[j - 1]) + "," 
        # gpsToWrite += str(lons[j - 1]) + " Bearing " + str(bearing) + "\n"
        # MST YYYY/MM/DD droneNum HH:mm:SS pH.0 temp.0 sal.0 lat.0000000,lon.000000 bearing
        dataToWrite = date + str(j) + " " + str(pH) + " " + str(temp) + " "
        dataToWrite += str(sal) + " " + str(lats[j - 1]) + "," + str(lons[j - 1]) + " " + str(bearing)

        # print(pHToWrite)
        # pHFile.write(pHToWrite)
        # tempFile.write(tempToWrite)
        # salFile.write(salToWrite)
        # gpsFile.write(gpsToWrite)
        while True:
            try:
                dataFile = open("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + str(j) + "\\data.txt", 'w')
                dataFile.write(dataToWrite)
                dataFile.close()
                break
            except: print("prob opened in GUI, try again")

        # pHFile.close()
        # tempFile.close()
        # salFile.close()
        # gpsFile.close()
        

    time.sleep(3)







