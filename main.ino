/* ----------------------------------------------------------------------------
    PROJECT:    AQUABOT C3
    AUTHORS:    SIDHANT GULATI
                GARRET FENDERSON
                SIWEN WANG
    
    SCHOOL:     UNIVERSITY OF ARIZONA
    COURSE:     ENGR 498 2022 - 2023
 * ------------------------------------------------------------------------- */

/* ------------------------------------------------------------ Header Files */
#include <Arduino.h>
#include <Servo.h>                      // Propellor and Rudder
#include <OneWire.h>                    // Temperature Sensor
#include <DallasTemperature.h>          // Temperature Sensor
#include <TinyGPSPlus.h>                // GPS
#include <QMC5883LCompass.h>            // Magnetometer Sensor
/* ------------------------------------------------------------------------- */

/* --------------------------------------------------------------- Constants */
#define SERIAL_BAUD 9600

// Drone Control
#define NUMBER_OF_DRONES  8
#define DRONE_NUMBER      6
#define MINIMUM_DISTANCE  3

// Rudder
#define RUDDER_PIN        4
#define MAX_RUDDER_ANGLE  170
#define MIN_RUDDER_ANGLE  10
#define DEF_RUDDER_ANGLE  90

// Propellor / Motor
#define MOTOR_PIN         5
#define MAX_MOTOR_SPEED   1520
#define MIN_MOTOR_SPEED   1490
#define ARM_MOTOR_SPEED   1450

// PID Constants
#define kp_rud 3.5
#define ki_rud 0.01
#define kd_rud 1.00

#define kp_mot 1.00
#define ki_mot 0.00
#define kd_mot 0.50

// Magnetometer
#define MAG_MEAN_SIZE     5
// #define MAG_OFFSET        103           // drone 1, 2
#define MAG_OFFSET        90             // drone 6
      
// pH Sensor
#define pHSensorPin       A2 

// Temperature Sensor
#define ONE_WIRE_BUS      A0  

// Salinity Sensor
#define TdsSensorPin      A1 
#define VREF              5.0           // analog reference voltage of the ADC
#define SCOUNT            5            // sum of sample point. Sidhant changed it from 1

// GPS
#define GPS_BAUD          9600

/* ------------------------------------------------------------------------- */

/* -------------------------------------------------------- Global Variables */
// Time
unsigned long time;
unsigned long previous_time;
unsigned long elapsed_time;

// Propellor
Servo motor;
float motor_value;
float previous_error_mot;
float error_mot;

// Rudder
Servo rudder;
float rudder_value;
float previous_error_rud;
float error_rud;

// PID values
float pid_p_rud;
float pid_i_rud;
float pid_d_rud;

float pid_p_mot;
float pid_i_mot;
float pid_d_mot;

// pH Sensor
unsigned long int avgValue;             // Store the average value of the sensor feedback
float b;
int buf[10];
int temp;

// Temperature Sensor  
OneWire oneWire(ONE_WIRE_BUS);          // Setup a oneWire instance to commmunicate with OneWire Device
DallasTemperature sensors(&oneWire);    // Pass oneWire reference to DallasTemperature library

// Salinity Sensor
int analogBuffer[SCOUNT];               // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float averageValue = 0;
float tdsValue = 0;
float temperature = 16;                 // current temperature for compensation

// GPS
TinyGPSPlus gps;
float cur_lat;
float cur_lon;
float des_lat;
float des_lon;

// Magnetometer Sensor
QMC5883LCompass mag;
float des_angle;
float cur_angle;
float running_val[MAG_MEAN_SIZE];

// XBee Radio
byte computerAddr[] = {0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x56, 0x31};
byte droneAddrs[8][8] = {{0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x56, 0x2E}, 
                         {0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x55, 0xE4}, 
                         {0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x56, 0x02},
                         {0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x56, 0x13},
                         {0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x55, 0xA0},
                         {0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x56, 0x11},
                         {0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x56, 0x16},
                         {0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x56, 0x32}};
bool packetDetected;                    // flag
int gpsFrequency;                       // in second
unsigned long gpsLastTime;
int broadcastFrequency;
unsigned long broadcastLastTime;
// the following can be changed based on command
int sensorFrequency;                    // in second
unsigned long sensorLastTime;
int availableDrones[8] = {-1, -1, -1, -1, -1, -1, -1, -1};

int total = 1;
int rank = 0;

// PID Controller
bool droneOn;
bool collisionFlag = true;
bool droneCollision[NUMBER_OF_DRONES] = {true, true, true, true, true, true, true, true};

/* ------------------------------------------------------------------------- */

/* ---------------------------------------------------------- Main functions */

/*
 * Initial set up
 */ 
void setup() {
    Serial.begin(SERIAL_BAUD);

    // Setting up Rudder
    rudderSetup();

    // Setting up Motor
    motorSetup();
  
    // Setting up the pH sensor
    pHSensorSetup();

    // Setting up the temperature Sensor
    temperatureSensorSetup();

    // Setting up the salinity sensor
    salinitySensorSetup();

    // Setting up the GPS sensor
    gpsSensorSetup();

    // Setting up the Magnetometer
    magnetometerSetup();

    // Setting up Xbee
    radioSetup();
    
    // Setting up LED
    pinMode(11, OUTPUT);
  
    // initialize
    gpsFrequency = 3;
    sensorFrequency = 3;
    broadcastFrequency = 3;    
    time = millis();
    sensorLastTime = time;
    gpsLastTime = time;

    Serial.println("Set up complete");    
}

void loop() {
    // update GPS data
    getGPS();
    // update time
    previous_time = time;
    time = millis();
    elapsed_time = time - previous_time;
    // grab data
    float pH, temp, ppm;
    cur_angle = magnetometerHeading();
    // pH = -1; 
    
    // timer to send gps data 
    if ((time - gpsLastTime) / 1000 >= gpsFrequency) {
        // timer to grab sensors data 
        if ((time - sensorLastTime) / 1000 >= sensorFrequency) {
            pH = pHSensorRead();
            temp = temperatureSensorRead();
            ppm = salinitySensorRead();
            sensorLastTime = time;
        } else {
            pH = -1;
            temp = -1;
            ppm = -1;
        }
        // grab GPS data
        getLatLon();
        String gpsTime = String("GMT ") + gps.date.year() + "/" + gps.date.month() + "/" + gps.date.day();
        gpsTime = String(gpsTime) + " " + gps.time.hour() + ":" + gps.time.minute() + ":" + gps.time.second();
        // send data to computer
        sendMsgToComputer(gpsTime, pH, temp, ppm, cur_lat, cur_lon, cur_angle);
        // timer to broadcast
//        if ((time - broadcastLastTime) / 1000 >= broadcastFrequency && gps.time.second() % 3 == 0) {
//
//Serial.println("broadcasting");
//          
//            broadcastGPSMsg(); 
//            broadcastLastTime = time; 
//        }       
        gpsLastTime = time;
    }


    if ((time - broadcastLastTime) / 1000 >= broadcastFrequency && gps.time.second() % total == rank && total > 1) {

        // Serial.println("broadcasting");
          
        broadcastGPSMsg(); 
        broadcastLastTime = time; 
    }      
    
    // receive message from XBee
    receiveMsg();
    
    if(collisionFlag)
        digitalWrite(11, LOW);
    else
        digitalWrite(11, HIGH);
            
    if (droneOn && collisionFlag) {
        Compute_PID();
        motor.writeMicroseconds(motor_value);
        rudder.write(rudder_value); 
        //Serial.println();             
    } else {
        motor.writeMicroseconds(ARM_MOTOR_SPEED);
        rudder.write(DEF_RUDDER_ANGLE);
    }
}

/* ------------------------------------------------------------------------- */

/* ------------------------------------------ Propeller and Rudder Functions */

/*
 * 
 * @para:       
 * @return:     
 */
void motorSetup() { 
  motor.attach(MOTOR_PIN); 
  motor.writeMicroseconds(ARM_MOTOR_SPEED); 
  delay(300);
  //motor.writeMicroseconds(MIN_MOTOR_SPEED);  
}

/*
 * 
 * @para:       
 * @return:     
 */
void rudderSetup() { 
  droneOn = false;
  collisionFlag = true;
  
  rudder.attach(RUDDER_PIN);
  rudder.write(DEF_RUDDER_ANGLE);
}

/* ------------------------------------------------------------------------- */

/* ----------------------------------------------------- pH Sensor Functions */

/*
 * 
 * @para:       
 * @return:     
 */
void pHSensorSetup() { pinMode(pHSensorPin, INPUT); }

/*
 * 
 * @para:       
 * @return:     
 */
float pHSensorRead() {
    int i, j;
    float phValue;
    //Get 10 sample value from the sensor for smooth the value
    for(i=0; i<10; i++) { 
        buf[i]=analogRead(pHSensorPin);
        delay(10);                                      // Change this???
    }
    //sort the analog from small to large
    for(i=0; i<9; i++) {
        for(j=i+1; j<10; j++) {
            if(buf[i]>buf[j]) {
                temp=buf[i];
                buf[i]=buf[j];
                buf[j]=temp;
            }
        }
    }
    avgValue = 0;
    // take the average value of 6 center sample
    for(i = 2; i < 8; i++) avgValue += buf[i];
    phValue = (float) avgValue*5.0/1024/6; //convert the analog into millivolt
    phValue = 3.5 * phValue;
    return phValue;
}

/* ------------------------------------------------------------------------- */

/* -------------------------------------------- Temperature Sensor Functions */

/*
 * 
 * @para:       
 * @return:     
 */
void temperatureSensorSetup() {
    sensors.begin();
}

/*
 * 
 * @para:       
 * @return:     
 */
float temperatureSensorRead() {
    sensors.requestTemperatures();
    return sensors.getTempCByIndex(0);
}

/* ------------------------------------------------------------------------- */

/* ------------------------------------------------Salinity Sensor Functions */

/*
 * 
 * @para:       
 * @return:     
 */
void salinitySensorSetup() { pinMode(TdsSensorPin, INPUT); }

/*
 * median filtering algorithm
 * @para:       
 * @return:     
 */
int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    for (byte i = 0; i<iFilterLen; i++)
    bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0) bTemp = bTab[(iFilterLen - 1) / 2];
    else bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
}

/*
 * 
 * @para:       
 * @return:     
 */
float salinitySensorRead() {
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT) analogBufferIndex = 0;

    int bArray[SCOUNT] = {};
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
        // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0;
        // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
        float compensationCoefficient = 1.0+0.02*(temperature-25.0);
        // temperature compensation
        float compensationVoltage=averageVoltage/compensationCoefficient;
        // convert voltage value to tds value
        tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
        // store the value
        bArray[copyIndex] = tdsValue;
    }
    return getMedianNum(bArray, SCOUNT);
}

/* ------------------------------------------------------------GPS Functions */

/*
 * 
 * @para:       N/A
 * @return:     N/A
 */
void gpsSensorSetup() { 
    Serial1.begin(GPS_BAUD);
}

int getGPS() {
    while(true) { // loop indefintely until gps oject is updated
      if(Serial1.available()) { // check for available serial data
        if(gps.encode(Serial1.read())) { // check package is complete and read it
          //gps.f_get_position(&lat,&lon); // get latitude and longitude
          return 1; // exite indefinite loop after updating gps object
          }
      }
    }
    return 0;  
}

void getLatLon() { 
  if(gps.location.isValid())
  {
    cur_lat = gps.location.lat();
    cur_lon = gps.location.lng();    
  } 
}


/*
 * 
 * @para:       
 * 
 * @return:     true, 
 *              false, 
 */
// float gpsDistance(float Lat, float Long)
// {
//     return gps.distance_between(Lat, Long, latitude, longitude);  
// }

// float gpsCourseChange(float Lat, float Long, float Heading)
// {
//     return gps.f_course();
// }

/* ------------------------------------------------------------------------- */

/* ------------------------------------------- Magnetometer Sensor Functions */

/*
 * 
 * @para:       
 * 
 * @return:     true, 
 *              false, 
 */
void magnetometerSetup() { 
  mag.init(); 
  //mag.setCalibration(-640, 720, -841, 561, -988, 0); // drone 1
  // mag.setCalibration(-826, 573, -740, 701, -1097, 0); // drone 2
  mag.setCalibration(-972, 443, -1085, 370, -1335, 0); // drone 6
}

/*
 * 
 * @para:       
 * 
 * @return:     true, 
 *              false, 
 */
float magnetometerHeading()
{
  mag.read();
  if((mag.getAzimuth() + MAG_OFFSET) < 0)
  {
    return mag.getAzimuth() + MAG_OFFSET + 360;
  }
  else if((mag.getAzimuth() + MAG_OFFSET) > 360)
  {
    return mag.getAzimuth() + MAG_OFFSET - 360;
  }
  return mag.getAzimuth() + MAG_OFFSET;
}

void Compute_PID()
{
  des_angle = gps.courseTo(cur_lat, cur_lon, des_lat, des_lon);


  
  // Serial.print("des_angle ");Serial.print(des_angle);Serial.print(" ");
  // Serial.print("cur_angle ");Serial.print(cur_angle);Serial.print(" ");
  // Serial.print(des_lat, 6);Serial.print(","); Serial.print(des_lon, 6);



  
  previous_error_rud = error_rud;
  previous_error_mot = error_mot;
  error_rud = des_angle - cur_angle;
  error_mot = gps.distanceBetween(cur_lat, cur_lon, des_lat, des_lon);

  if(error_rud > 180)
  {
    error_rud = error_rud - 360.00;
  }
  else if(error_rud < -180)
  {
    error_rud = error_rud + 360.00;
  }  

  // Serial.print("error_rud ");Serial.print(error_rud);Serial.print(" ");
  // Serial.print("error_mot ");Serial.print(error_mot);Serial.print(" ");

  pid_p_rud = kp_rud * error_rud;
  pid_p_mot = kp_mot * error_mot;

  pid_i_mot = pid_i_mot + (error_mot * ki_mot);
  pid_i_rud = pid_i_rud + (error_rud * ki_rud);

  pid_d_rud = kd_rud * ((error_rud - previous_error_rud) / elapsed_time);
  pid_d_mot = kd_mot * ((error_mot - previous_error_mot) / elapsed_time);

  if(abs(error_rud) <= 0.01)
  {
    pid_i_rud = 0.00;
  }
  if(abs(error_mot) <= 2)
  {
    pid_i_mot = 0.00;
  }

  rudder_value = pid_p_rud + pid_i_rud - pid_d_rud;   // Add derivitive term or subtract???
  motor_value = pid_p_mot + pid_i_mot - pid_d_mot;

  rudder_value = DEF_RUDDER_ANGLE - rudder_value;
  motor_value = MIN_MOTOR_SPEED + motor_value;

  if(rudder_value > MAX_RUDDER_ANGLE) 
  {
    rudder_value = MAX_RUDDER_ANGLE;
  }
  else if(rudder_value < MIN_RUDDER_ANGLE) 
  {
    rudder_value = MIN_RUDDER_ANGLE;  
  }

  if(motor_value > MAX_MOTOR_SPEED)
  {
    motor_value = MAX_MOTOR_SPEED;
  }
  else if(motor_value < MIN_MOTOR_SPEED)
  {
    motor_value = MIN_MOTOR_SPEED;
  }
}

/* ------------------------------------------------------------------------- */

/* ---------------------------------------------------- XBee Radio Functions */

/*
 * XBee Module initial setup
 */
void radioSetup() {
    packetDetected = false; /* Initialize variables */
    Serial2.begin(9600);
}

/*
 * Send data to the computer
 */
void sendMsgToComputer(String gpsTime, float pH, float temp, float ppm, float lat, float lon, float heading) {
  String msg = gpsTime + " ";
  msg += String(DRONE_NUMBER);
  msg += " ";
  if (pH == -1) msg += "-";
  else msg += String(pH, 2);
  msg += " ";
  if (temp == -1) msg += "-";
  else msg += String(temp, 2);
  msg += " ";
  if (ppm == -1) msg += "-";
  else msg += String(ppm, 2);
  msg += " ";
  msg += String(lat, 6);
  msg += ",";
  msg += String(lon, 6);
  msg += " ";
  msg += String(heading, 2);  

  // Serial.println(msg);
    
  sendMsg(&computerAddr[0], msg);
}

void broadcastGPSMsg() {
    String msg = String(DRONE_NUMBER) + " ";
    msg += String(cur_lat, 6);
    msg += ",";
    msg += String(cur_lon, 6);
    if (droneOn) msg += " started";
    //Serial.println("I am broadcasting");

    for (int i = 0; i < NUMBER_OF_DRONES; i++) {
      if (i + 1 > DRONE_NUMBER && availableDrones[i] != -1) 
        sendMsg(&droneAddrs[i][0], msg);
    }
}

void sendMsg(byte *addr, String msg) {
    byte payload[255];
    payload[0] = 0x7E;                            // Start delimiter
    payload[1] = 0x00;                            // length of the msg would not exceed 255
    payload[2] = 14 + msg.length();               // length
    payload[3] = 0x10;                            // frame type
    payload[4] = 0x01;                            // frame ID
    for (int i = 0; i < 8; i++) 
        payload[5 + i] = addr[i];                 // 64-bit dest. address
    payload[13] = 0xFF;                           // 16-bit dest. address
    payload[14] = 0xFE;
    payload[15] = 0x00;                           // Broadcast radius
    payload[16] = 0x00;                           // Options
    msg.getBytes(&payload[17], msg.length() + 1); // msg
    payload[17 + msg.length()] = getChecksum(&payload[3], payload[2]); // checksum

    Serial2.write(payload, sizeof(payload));
}

/* Receive data from other XBee module */
void receiveMsg() {
    if (Serial2.available() && Serial2.read() == 0x7E && packetDetected == false) {      
        /* start to receive a message */
        packetDetected = true;
        // Serial.println("I received a massage");
        // grab the length of the message
        int count = 0;
        byte length = 0;
        while (count < 2) {
            if (Serial2.available()) {
                length = Serial2.read();
                count++;
            }
        }
        /* store the message */
        count = 0;
        byte msg[length] = {};
        while (count < length) {
            if (Serial2.available()) {
                msg[count] = Serial2.read();
                count++;
            }
        }
        /* get the checksum */
        byte checksum = -1;
        while (true) {
            if (Serial2.available()) {
                checksum = Serial2.read();
                break;
            }
        }
        /* check data consistency using checksum */
        if (!checkConsistency(msg, length, checksum)) {
            Serial.print("Corrupted message: ");
            Serial.println(parseMsg(msg, length));
            packetDetected = false;
            return;
        } 
        /* If the message is from computer, send receive message */
        if (isFromComputer(msg)) {
          Serial.println("Received massage from computer");
          processComputerCommand(parseMsg(msg, length));
        }
        /* If the message is from other drones */
        else {
            //Serial.println("Message not from computer");
            int droneIndex = fromWhichDrone(msg);

            if (droneIndex == DRONE_NUMBER) {
              Serial.println("received message from myself");
            }
            
            if (droneIndex != DRONE_NUMBER && droneIndex != -1) {
                Serial.print("Message from Drone ");
                Serial.print(droneIndex);
                Serial.print(": ");
                Serial.println(parseMsg(msg, length));

                processDroneInfo(droneIndex, parseMsg(msg, length));
            } 
        }
        /* reset the flag to prepare for the next message */
        packetDetected = false;
    }
}

/*
 * Get the checksum of the payload. First get the sum of all bytes in the 
 * packet, delimiter and the length excluded. Then get the last two digits of
 * the sum, and subtrcted it from 0xFF.
 * @para:      payload, the byte array of the data;
 *              length, the length of the message.
 * @return:   checksum, the calculated checksum  .
 */
int getChecksum(byte *payload, int length) {
  int sum = 0;
  for (int i = 0; i < length; i++) sum += payload[i];
  int remain = sum & 0xff;
  int checksum = 0xff - remain;
  return checksum;
}

/*
 * Check weather the received message is corrupted or not.
 * @para:       msg, an byte array of the received message;
 *              length, the length of the message;
 *              checksum, an int as the received checksum.
 * @return:     true, if the data is not corrupted;
 *              false, otherwise.
 */
bool checkConsistency(byte *msg, int length, int checksum) {
  if (getChecksum(msg,length) != checksum) return false;
  return true;
}

/*
 * Check the address of the received message and whether it's
 * from the computer.
 * @para:       msg, an byte array of the received message.
 * @returnL     true, if the address match computer's address;
 *              false, otherwise.
 */
bool isFromComputer(byte *msg) {
  for (int i = 0; i < 8; i++)
    if (msg[i + 1] != computerAddr[i]) return false;
  return true;
}

int fromWhichDrone(byte *msg) {
  for (int i = 0; i < NUMBER_OF_DRONES; i++) {
    bool flag = true;
    for (int j = 0; j < 8; j++) {
      if (msg[j + 1] != droneAddrs[i][j]) {
        flag = false;
        continue;
      }
    }
    if (flag) return i + 1;
  }
  return -1;
}

void processComputerCommand(String msg) {
    // sendMsg(&computerAddr[0], String(DRONE_NUMBER) + " S");
    Serial.print("Command: ");
    Serial.println(msg);
    // if initial configuration command
    if (msg.substring(0, 2).equals("A:")) {
      total = 0;
      rank = 0;
    
        Serial.println(msg);
        for (int i = 0; i < 8; i++) availableDrones[i] = -1;
        for(int i = 3; i < msg.length(); i++) {
            int droneNum = msg[i] - 48;
            availableDrones[droneNum - 1] = 1;
            total++;
            if (droneNum < DRONE_NUMBER) rank++;
            i++;
        }
        if (rank > 0) rank++;

        Serial.print("total: ");
        Serial.print(total);
        Serial.print(", rank: ");
        Serial.println(rank);

        broadcastFrequency = total;
    
        Serial.println("availableDrones: ");
        for (int i = 0; i < 8; i++) {

               
          
            Serial.print(availableDrones[i]);
            Serial.print(" "); 
            Serial.print(droneCollision[i]);
            Serial.print(" ");
        }
        Serial.println();


      // if drone control command
    } else if (msg.substring(0, 5).equals("Drone")) {
        // check 
        if (msg.substring(6, 7).toInt() != DRONE_NUMBER) {
            Serial.println("error, this drone should not receive this message");
            return;
        }
        // parse command
        String command = msg.substring(8);
        // START
        if (command.equals("START")) droneOn = true;
        // STOP
        else if (command.equals("STOP")) droneOn = false;
        // CANCEL
        else if (command.equals("CANCEL")) {
            des_lat = cur_lat;
            des_lon = cur_lon;
            droneOn = true;
        // SENSOR FREQUENCY
        } else if (command.substring(0, 15).equals("SF")) {
            sensorFrequency = command.substring(16).toInt();

            
            Serial.print("New sensor frequency: ");
            Serial.println(sensorFrequency);

            
        // DRONE DISTANCE
        // LAT,LON
        } else {
            int comma = command.indexOf(",");
            des_lat = command.substring(0, comma).toFloat();
            des_lon = command.substring(comma + 1).toFloat();
    
            Serial.print("des_lat: ");
            Serial.print(des_lat);
            Serial.print(", des_lon: ");
            Serial.println(des_lon);
            
        }
    }
}

void processDroneInfo(int droneIndex, String msg) {
    bool flag = true;
    droneCollision[droneIndex-1] = true;
  
    if(!gps.location.isValid()) {           //DO WE NEED THIS HERE
        return;
    }
  
    String command = msg.substring(2);        
    int comma = command.indexOf(",");
    float lat = command.substring(0, comma).toFloat();
    float lon = command.substring(comma + 1).toFloat();
    /*
    Serial.print("Drone: ");
    Serial.print(droneIndex);
    Serial.print(" lat: ");
    Serial.print(lat);
    Serial.print(" lon: ");
    Serial.println(lon);
    Serial.print("Drone: ");
    Serial.print(DRONE_NUMBER);
    Serial.print(" lat: ");
    Serial.print(cur_lat);
    Serial.print(" lon: ");
    Serial.println(cur_lon);
    Serial.print("Distance: ");
    Serial.println(gps.distanceBetween(lat, lon, cur_lat, cur_lon));
    */
    
    if(gps.distanceBetween(lat, lon, cur_lat, cur_lon) < MINIMUM_DISTANCE) {
        droneCollision[droneIndex-1] = false;
    }
    else {
        droneCollision[droneIndex-1] = true;
    }
    for(int i = 0; i < DRONE_NUMBER; i++) {
        flag = flag && droneCollision[i];
    }
    collisionFlag = flag;

}

/*
 * Parse the command received from computer and return it
 * @para:       msg, an byte array of the received message;
 *              length, the length of the message.
 * @returnL     command, as a String
 */
String parseMsg(byte *msg, int length) {
  String message = "";
  for (int i = 12; i < length; i++) {
    char temp = msg[i];
    message += temp;
  }
  return message;
}

/* ------------------------------------------------------------------------- */
