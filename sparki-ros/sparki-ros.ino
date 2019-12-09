/* ***** Based on Sparki_Myro originally created by Jeremy Eglen ***** */


// these defines make the code ugly in a lot of places, but have been put in to save space (in Sparki's memory) when possible
#define NO_DEBUGS	// save more memory by excluding any DEBUG_DEBUG lines
#define COMPACT_2  // remove certain LCD functions (there was once a COMPACT)
#define STATUS_ACK // if this is defined, the status light will be lit when the Sparki is processing a command

#include <Sparki.h>

/* ########### CONSTANTS ########### */
const char TERMINATOR = (char)23;      // this terminates every transmission from python
const char SYNC = (char)22;            // send this when in the command loop waiting for instructions


/* ***** COMMAND CHARACTER CODES ***** */
const char COMMAND_BEEP = 'b';          // requires 2 arguments: int freq and int time; returns nothing
const char COMMAND_COMPASS = 'c';       // no arguments; returns float heading
const char COMMAND_GET_ACCEL = 'f';     // no arguments; returns array of 3 floats with values of x, y, and z
const char COMMAND_GET_LIGHT = 'k';     // no arguments; returns array of 3 ints with values of left, center & right light sensor                                    
const char COMMAND_GET_LINE = 'm';      // no arguments; returns array of 5 ints with values of left edge, left, center, right & right edge line sensor
const char COMMAND_GET_MAG = 'o';       // no arguments; returns array of 3 floats with values of x, y, and z
const char COMMAND_GRIPPER_CLOSE_DIS = 'v'; // requires 1 argument: float distance to close the gripper; returns nothing
const char COMMAND_GRIPPER_OPEN_DIS = 'x';  // requires 1 argument: float distance to open the gripper; returns nothing
const char COMMAND_GRIPPER_STOP = 'y';  // no arguments; returns nothing
const char COMMAND_INIT = 'z';          // no arguments; confirms communication between computer and robot
const char COMMAND_LCD_CLEAR = '0';     // no arguments; returns nothing
const char COMMAND_LCD_DRAW_CIRCLE = '1';   // requires 4 arguments: int x&y, int radius, and int filled (1 is filled); returns nothing
const char COMMAND_LCD_DRAW_RECT = '4'; // requires 5 arguments: int x&y for start point, ints width & height, and int filled (1 is filled); returns nothing 

const char COMMAND_LCD_DRAW_LINE = '2'; // requires 4 arguments ints x&y for start point and x1&y1 for end points; returns nothing
const char COMMAND_LCD_DRAW_PIXEL = '3';    // requires 2 arguments: int x&y; returns nothing
const char COMMAND_LCD_DRAW_STRING = '5';   // requires 3 arguments: int x (column), int line_number, and char* string; returns nothing
const char COMMAND_LCD_PRINT = '6';     // requires 1 argument: char* string; returns nothing
const char COMMAND_LCD_PRINTLN = '7';   // requires 1 argument: char* string; returns nothing
const char COMMAND_LCD_READ_PIXEL = '8';    // requires 2 arguments: int x&y; returns int color of pixel at that point
const char COMMAND_LCD_SET_COLOR = 'T'; // requires 1 argument: int color; returns nothing
const char COMMAND_LCD_UPDATE = '9';    // no arguments; returns nothing
const char COMMAND_MOTORS = 'A';        // requires 3 arguments: int left_speed (-100 to 100), int right_speed (-100 to 100), & float time
                                        // if time < 0, motors will begin immediately and will not stop; returns nothing
const char COMMAND_BACKWARD_CM = 'B';   // requires 1 argument: int cm to move backward; returns nothing
const char COMMAND_FORWARD_CM = 'C';    // requires 1 argument: int cm to move forward; returns nothing
const char COMMAND_PING = 'D';          // no arguments; returns ping at current servo position
const char COMMAND_RECEIVE_IR = 'E';    // no arguments; returns int IR receiver value (-1 indicates no data available)
const char COMMAND_SEND_IR = 'F';       // requires 1 argument: int data; returns nothing
const char COMMAND_SERVO = 'G';         // requires 1 argument: int servo position; returns nothing

const char COMMAND_SET_RGB_LED = 'I';   // requires 3 arguments: int red, int green, int blue; returns nothing; note hardware limitations 
                                        // will prevent some values of red, green, and blue from showing up
const char COMMAND_SET_STATUS_LED = 'J';    // requires 1 argument: int brightness of LED; returns nothing
const char COMMAND_STOP = 'K';          // no arguments; returns nothing
const char COMMAND_TURN_BY = 'L';       // requires 1 argument: float degrees to turn - if degrees is positive, turn clockwise,
                                        // if degrees is negative, turn counterclockwise; returns nothing
const char COMMAND_NOOP = 'Z';  // no arguments; returns nothing; does nothing; added 1.1.3; can be used to prevent a timeout in communication with the robot
/* ***** END OF COMMAND CHARACTER CODES ***** */


/* ***** SENSOR POSITION CONSTANTS ***** */
/* LINE SENSORS */
const int LINE_EDGE_RIGHT = 4;
const int LINE_MID_RIGHT = 3;
const int LINE_MID = 2;
const int LINE_MID_LEFT = 1;
const int LINE_EDGE_LEFT = 0;

/* LIGHT SENSORS */
const int LIGHT_SENS_RIGHT = 2;
const int LIGHT_SENS_MID = 1;
const int LIGHT_SENS_LEFT = 0;


/* ***** SERIAL PORT FOR BLUETOOTH ***** */
#ifdef USE_BLUETOOTH
#define serial Serial1
#else
#define serial Serial
#endif
/* ########### END OF CONSTANTS ########### */



/* ########## FUNCTION PROTOTYPES ########### */
/* ***** SPARKI UTILITY FUNCTIONS ***** */
/* These functions are used internally, and are not called by the computer */
char getSerialChar();  // gets a char from the serial port; BLOCKING
float getSerialFloat();  // gets a float from the serial port; BLOCKING
int getSerialInt();  // gets an int from the serial port; BLOCKING

int getSerialBytes(char* buf, int size); // gets bytes from the serial port; BLOCKING

void motors(int left_speed, int right_speed);  // starts the motors at the speed indicated

void sendSerial(char c);
void sendSerial(char* message);
void sendSerial(float f);
void sendSerial(float* floats, int size);
void sendSerial(int i);
void sendSerial(int* ints, int size);


/* ***** SPARKI COMMANDS ***** */
void getAccel();
float getBattery();
void getLight();
void getLine();
void getMag();
void initSparki();  // confirms communication with Python

// LCD functions are generally handled via direct calls
void LCDdrawCircle(int center_x, int center_y, int radius, int filled);
void LCDdrawRect(int corner_x, int corner_y, int width, int height, int filled);

void LCDdrawString(int x, int y);
void LCDprint();
void LCDsetColor(int color);
void setDebugLevel(int level);
void setStatusLED(int brightness);
void stop();
void turnBy(float deg);
/* ########## END OF FUNCTION PROTOTYPES ########### */


/* ########### FUNCTIONS ########### */
/* ***** SERIAL FUNCTIONS ***** */
// gets a char from the serial port
// blocks until one is available
char getSerialChar() {
  int size = 5;
  char buf[size];
  int result = getSerialBytes(buf, size);
  return buf[0];
}

// gets a float from the serial port
// blocks until one is available
// floats are sent to Sparki as char* in order to eliminate conversion issues
// we then convert the string to a float and return that
// there is likely to be some loss of precision
float getSerialFloat() {
  int size = 20;
  char buf[size];
  int result = getSerialBytes(buf, size);
  return atof( buf );
}

// gets an int from the serial port
// blocks until one is available
// ints are sent to Sparki as char* in order to eliminate conversion issues
// we then convert the string to an int and return that
int getSerialInt() {
  int size = 20;
  char buf[size];
  int result = getSerialBytes(buf, size);
  return atoi( buf );
}

// gets bytes from the serial port
// the byte stream should be terminated by TERMINATOR
// buf will be overwritten entirely
// way too much work went into getting this function right - it probably could be more efficient
// returns the number of bytes read
int getSerialBytes(char* buf, int size) {
  char inByte = -1;
  int maxChars = size; 
  int count = 0;
  
  // zero out the buffer
  for( int i = 0; i < maxChars; i++ ) {
    buf[i] = '\0';
  }

  while ((inByte != TERMINATOR) && (count < maxChars)) {
    if(serial.available()) {
      inByte = serial.read();
   
      if ((inByte != TERMINATOR) && (inByte >= 0)) {
        buf[count++] = (char)inByte;
      }
    }
  }
  
  // flush extra characters
  while ((inByte != TERMINATOR) && (serial.available())) {
      inByte = serial.read();
  }

  return count;
}


/* These functions send data from Sparki to the computer over Bluetooth */
void sendSerial(char c) {
  serial.print(c); 
  serial.print(TERMINATOR); 
}

void sendSerial(char* message) {
  serial.print(message); 
  serial.print(TERMINATOR); 
}

void sendSerial(float f) {
  serial.print(f); 
  serial.print(TERMINATOR); 
}

void sendSerial(float* floats, int size) {
  for(int j = 0; j < size; j++) {  
    sendSerial(floats[j]);
  }
}

void sendSerial(int i) {
  serial.print(i); 
  serial.print(TERMINATOR); 
}

void sendSerial(int* ints, int size) {
  for(int j = 0; j < size; j++) {  
    sendSerial(ints[j]);
  }
}

void sendSync() {
  serial.print(SYNC);
  serial.flush();
}
/* ***** END OF SERIAL FUNCTIONS ***** */

/* ***** SPARKI FUNCTIONS ***** */
// void getAccel()
// sends an array with the values of the X, Y, and Z accelerometers
void getAccel() { 
  sparki.readAccelData(); // it's faster to get the values this way than call them individually
  float values[3] = { 
    -sparki.xAxisAccel*9.8, 
    -sparki.yAxisAccel*9.8, 
    -sparki.zAxisAccel*9.8   };
  
  sendSerial( values, 3 );
} // end getAccel()


// void getLight()
// sends an array with the values of the left, center and right light sensors
void getLight() { 
  int values[3] = { 
    sparki.lightLeft(),
    sparki.lightCenter(),
    sparki.lightRight()   };  
  sendSerial( values, 3 );
} // end getLight()


// void getLine()
// sends an array with the values of the left edge, left, center, right, and right edge line sensors
void getLine() { 
  int values[5] = { 
    sparki.edgeLeft(),
    sparki.lineLeft(),
    sparki.lineCenter(),
    sparki.lineRight(),
    sparki.edgeRight()   };
  
  sendSerial( values, 5 );
} // end getLine()

// void getMag()
// sends an array with the values of the X, Y, and Z magnometers
void getMag() { 
  sparki.readMag(); // it's faster to get the values this way than call them individually
  float values[3] = { 
    sparki.xAxisMag, 
    sparki.yAxisMag, 
    sparki.zAxisMag   };

  sendSerial( values, 3 );
} // end getMag()

// return version to prove communication
void initSparki() {
  sparki.clearLCD();
  sparki.print("Init Received");
  sparki.updateLCD();
	
  sendSerial("Sparki ROS Connected");
} 


// LCDdrawCircle(int, int, int, int)
// draw a circle with a center at center_x, center_y
// with a radius of radius
// if filled > 0, fill it in
void LCDdrawCircle(int center_x, int center_y, int radius, int filled) {
  if (filled > 0) {
    sparki.drawCircleFilled(center_x, center_y, radius);
  } else {
    sparki.drawCircle(center_x, center_y, radius);
  }
} // end LCDdrawCircle(int, int, int, int)


// LCDdrawRect(int, int, int, int, int)
// draw a circle with a corner at corner_x, corner_y
// with a width of width and a height of height
// if filled > 0, fill it in
void LCDdrawRect(int corner_x, int corner_y, int width, int height, int filled) {
  if (filled > 0) {
    sparki.drawRectFilled(corner_x, corner_y, width, height);
  } else {
    sparki.drawRect(corner_x, corner_y, width, height);
  }
} // end LCDdrawRect(int, int, int, int, int)


// LCDdrawString(int, int)
// gets a line number and position, then gets a string from the serial port and prints it
void LCDdrawString(int x, int y) {
  int maxChars = 20;
  char buf[maxChars];
 
  getSerialBytes( buf, maxChars );

  sparki.drawString( x, y, buf);  
} // end LCDdrawString(int, int)


// LCDprint()
// gets data from the serial port and prints it
void LCDprint() {
  int maxChars = 20;
  char buf[maxChars];
  
  getSerialBytes( buf, maxChars );

  sparki.print(buf);  
} // end LCDprint()


// LCDsetColor(int)
// sets the drawing color where the color is a constant defined in Sparki.h
void LCDsetColor(int color) {
  sparki.setPixelColor( color );  // WHITE is defined in Sparki.h
} // end LCDsetColor(int)


// motors(int,int)
// moves Sparki's left wheel at left_speed and right wheel at right_speed
// speed should be a number from -100 to 100 indicating the percentage of power used
// if the speed is positive, that indicates forward motion on that wheel
void motors(int left_speed, int right_speed) { 
  if (left_speed >= 0) {
    sparki.motorRotate(MOTOR_LEFT, DIR_CCW, left_speed);
  } 
  else if (left_speed < 0) {
    sparki.motorRotate(MOTOR_LEFT, DIR_CW, -left_speed);
  }

  if (right_speed >= 0) {
    sparki.motorRotate(MOTOR_RIGHT, DIR_CW, right_speed);
  } 
  else if (right_speed < 0) {
    sparki.motorRotate(MOTOR_RIGHT, DIR_CCW, -right_speed);
  }
} // end motors(int,int)


// setStatusLED(int)
// sets the status LED to brightness -- brightness should be between 0 and 100 (as a percentage)
void setStatusLED(int brightness) {
  // if brightness is 0 or 100, we'll use the digital write
  if (brightness <= 0) {
    digitalWrite(STATUS_LED, LOW); // will turn the LED off
    delay(5);
  } else if (brightness >= 100) {
    digitalWrite(STATUS_LED, HIGH); // will turn the LED on
    delay(5);
  } else { // otherwise, we'll use the analog write
    analogWrite(STATUS_LED, brightness * 2.55); // the maximum brightness is 255, so we convert the percentage to that absolute
    delay(5);
  }
} // end setStatusLED(int)


// stop()
// stop the gripper and the movement motor
void stop() {
  sparki.gripperStop();
  sparki.moveStop();
} // end stop()


// turnBy(float)
// turn the robot by deg degress -- positive will turn right and negative will turn left
void turnBy(float deg) {
  if (deg < 0) {
    sparki.moveLeft(-deg);
  } else if (deg > 0) {
    sparki.moveRight(deg);
  }
} // end turnBy(int)


// setup - start robot
void setup() {
  setStatusLED(50);
  sparki.servo(SERVO_CENTER); // rotate the servo to its 0 degree postion (forward)  

  serial.begin(9600);
 
  sparki.RGB( 0, 0, 0 );

  sparki.println("SparkiROS Online");
  sparki.updateLCD();
} // end setup()


// main loop()
void loop() {
#ifdef STATUS_ACK
  setStatusLED(0);                      // turn off the LED
#endif // STATUS_ACK

  if (serial.available()) {
    char inByte = getSerialChar();
    setStatusLED(100);                  // turn on the LED while we're processing a command

    /* For any and all commands which have multiple arguments, we must first
     * get those arguments and then pass them to the function. It appears that
     * the compiler does not do the getSerial___()'s in the order that they're
     * written, but the reverse of that -- it's unclear if that's this particular
     * version of the compiler, or if it's simply that you're not guaranteed
     * as to the order of calls
     */

    switch (inByte) {
    case COMMAND_BEEP:                // int, int; returns nothing
      {
      int freq = getSerialInt();
      int duration = getSerialInt();
      sparki.beep( freq, duration );
      break;
      }
    case COMMAND_COMPASS:             // no args; returns float
      sendSerial( sparki.compass() );
      break;
    case COMMAND_GET_ACCEL:           // no args; returns array of 3 floats
      getAccel();                     // sendSerial is done in the function
      break;
  	case COMMAND_GET_LIGHT:           // no args; returns array of 3 ints
      getLight();                     // sendSerial is done in the function
      break;
    case COMMAND_GET_LINE:            // no args; returns array of 5 ints
      getLine();                      // sendSerial is done in the function
      break;
    case COMMAND_GET_MAG:             // no args; returns array of 3 floats
      getMag();                       // sendSerial is done in the function
      break;
    case COMMAND_GRIPPER_CLOSE_DIS:   // float; returns nothing
      sparki.gripperClose( getSerialFloat() );
      break;
    case COMMAND_GRIPPER_OPEN_DIS:    // float; returns nothing
      sparki.gripperOpen( getSerialFloat() );
      break;
    case COMMAND_GRIPPER_STOP:        // no args; returns nothing
      sparki.gripperStop();
      break;
    case COMMAND_INIT:                // no args; returns a char
      initSparki();                   // sendSerial is done in the function
      break;
    case COMMAND_LCD_CLEAR:           // no args; returns nothing
      sparki.clearLCD();
      break;
    case COMMAND_LCD_DRAW_CIRCLE:     // int, int, int, int; returns nothing
      {
      int center_x = getSerialInt();
      int center_y = getSerialInt();
      int radius = getSerialInt();
      int filled = getSerialInt();
      LCDdrawCircle( center_x, center_y, radius, filled );
      break;
      }
    case COMMAND_LCD_DRAW_RECT:       // int, int, int, int, int; returns nothing
      {
      int corner_x = getSerialInt();
      int corner_y = getSerialInt();
      int width = getSerialInt();
      int height = getSerialInt();
      int filled = getSerialInt();
      LCDdrawRect( corner_x, corner_y, width, height, filled );
      break;
      }
    case COMMAND_LCD_DRAW_PIXEL:      // int, int; returns nothing
      {
      int x = getSerialInt();
      int y = getSerialInt();
      sparki.drawPixel( x, y );
      break;
      }
	case COMMAND_LCD_DRAW_STRING:     // int, int, char*; returns nothing
      {
      int x = getSerialInt();
      int y = getSerialInt();
      LCDdrawString( x, y );
      break;
      }
    case COMMAND_LCD_SET_COLOR:      // int; returns nothing
      LCDsetColor( getSerialInt() );
      break;
    case COMMAND_LCD_PRINT:           // char*; returns nothing
      LCDprint();					  // gets char* in function
      break;
    case COMMAND_LCD_PRINTLN:         // char*; returns nothing
      LCDprint();					  // gets char* in function
      sparki.println(' ');
      break;
    case COMMAND_LCD_READ_PIXEL:      // int, int; returns nothing
      {
      int x = getSerialInt();
      int y = getSerialInt();
      sendSerial( sparki.readPixel( x, y ) );
      break;
      }
    case COMMAND_LCD_UPDATE:          // no args; returns nothing
      sparki.updateLCD();
      break;
    case COMMAND_MOTORS:              // int, int, float; returns nothing
      { 
      int left_speed = getSerialInt();
      int right_speed = getSerialInt();
      motors( left_speed, right_speed );
      break;
      } // end COMMAND_MOTORS
    case COMMAND_BACKWARD_CM:         // float; returns nothing
      sparki.moveBackward( getSerialFloat() );
      break;
    case COMMAND_FORWARD_CM:          // float; returns nothing
      sparki.moveForward( getSerialFloat() );
      break;
    case COMMAND_PING:                // no args; returns int
      sendSerial( sparki.ping() );
      break;
    case COMMAND_RECEIVE_IR:          // no args; returns int
      sendSerial( sparki.readIR() );
      break;
    case COMMAND_SEND_IR:             // int; returns nothing
      sparki.sendIR( getSerialInt() );
      break;
    case COMMAND_SERVO:               // int; returns nothing
      sparki.servo( getSerialInt() );
      break;
    case COMMAND_SET_RGB_LED:         // int, int, int; returns nothing
      {
	    /* arcbotics states that underlying hardware limitations can cause colors not to appear as intended
	       specifically, the LEDs draw different voltages, red more than green and green more than blue
	       as a side effect, if you have a situation where red == green == blue, only red will show */
		 
	    /* arcbotics recommends the following values for the specified colors:
         blue   0,   0,   100
         green  0,   100, 0
         indigo 20,  0,   100
         orange 90,  100, 0
         pink   90,  0,   100
         red    100, 0,   0
         violet 60,  0,   100
         white  60,  100, 90
         yellow 60,  100, 0
         off    0,   0,   0 */

      int red = getSerialInt();
      int green = getSerialInt();
      int blue = getSerialInt();
      sparki.RGB( red, green, blue );
      break;
      }
    case COMMAND_SET_STATUS_LED:      // int; returns nothing
      setStatusLED( getSerialInt() );
      break;
    case COMMAND_STOP:                // no args; returns nothing
      stop();
      break;
    case COMMAND_TURN_BY:             // float; returns nothing
      turnBy( getSerialFloat() );
      break;
    case COMMAND_NOOP:             // no args; returns nothing; does nothing
      // can be used to prevent a timeout in communication with the robot
      break;

    default:
      sparki.print("Bad input");
      sparki.updateLCD();
      stop();
      sparki.beep();
      serial.print(TERMINATOR); 
      break;
    } // end switch ((char)inByte)
  } // end if (serial.available())
  
  sendSync();    // we send the sync every time rather than a more complex handshake to save space in the program
} // end loop()
/* ########### END OF FUNCTIONS ########### */
