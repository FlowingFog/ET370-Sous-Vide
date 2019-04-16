#include <OneWire.h>
#include <Dimmer.h>

//Setup the button Variables to each individual pin. Using Analog pins so no need to declare the pins as outputs

#define left A0
#define right A3
#define up A1
#define down A2
#define res A4

int buttonPress; //Contains the pressed button

//Will trigger buttons on rising edge, ie, when button is pressed

int buttonLast = 0;
int buttonNow = 0;

/*Running 16x2 LCD screen. Used as a menu.
 * Rs is on pin 13
 * E is on pin 12
 * D7 is on pin 7
 * D6 is on pin 6
 * D5 is on pin 5
 * D4 is on pin 4
*/
#include <LiquidCrystal.h>

LiquidCrystal lcd(13, 12, 7, 6, 5, 4);

int screen = 1; //Used to show decide which screen displayed by the LCD

/*Dimmer controls the AC power. PID controls the PWM going to the Dimmer. 
 * ZeroCurrent Cross is pin 2
 * PWM is pin 3
 */
OneWire ds(10);

int setpoint = 130; //Sp is temperature setpoint, intitial setting is 145.
double error;
double tempIn;
double errSum;
double output;
double outputAdj;
double proBand = 100;
double Sp = setpoint;
double pidProportional = 100/proBand;
double kp = pidProportional; // Constant of Proportionality is based upon the proportional band
double intRes = 10;
double pidIntegral = 1/intRes;
double ki = pidIntegral; // Constant of Integral Term is based upon the integral adjustment
unsigned long changeTime = 5000;
unsigned long lastTime ;
unsigned long timedifference;
int errorDiffMax = 15;
double slope = 100/errorDiffMax;
Dimmer dimmer(3, DIMMER_RAMP, 1.5, 60);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.print("Welcome to");
  lcd.setCursor(0, 1);
  lcd.print("Sous Vide");
  delay(2000);
  lcd.clear();
  lcd.home();
  lcd.print("Set Temperature");
}

//Function that reads buttons, should return value that corresponds with button press
int buttonRead() {
  int buttonReading = 0;
  if (analogRead(left) >= 500) {
    buttonNow = 1;
  }
  if (analogRead(right) >= 500) {
    buttonNow = 2;
  }
  if (analogRead(up) >= 500) {
    buttonNow = 3;
  }
  if (analogRead(down) >= 500) {
    buttonNow = 4;
  }
  if (analogRead(res) >= 500) {
    buttonNow = 5;
  }
  int  buttonSum = analogRead(left) + analogRead(right) + analogRead(up) + analogRead(down) + analogRead(res);
  if (buttonSum <= 400) {
    buttonNow = 0;
  }
  if (buttonLast == 0 && buttonNow == 1) {
    Serial.println("Left is Pressed");
    buttonReading = 1;
  }
  if (buttonLast == 0 && buttonNow == 2) {
    Serial.println("Right is Pressed");
    buttonReading = 2;
  }
  if (buttonLast == 0 && buttonNow == 3) {
    Serial.println("Up is Pressed");
    buttonReading = 3;
  }
  if (buttonLast == 0 && buttonNow == 4) {
    Serial.println("Down is Pressed");
    buttonReading = 4;
  }
  if (buttonLast == 0 && buttonNow == 5) {
    Serial.println("Reset is Pressed");
    buttonReading = 5;
  }
  buttonLast = buttonNow;
  return buttonReading;
}

void screenSp() {
  //This screen is for setting temperature setpoint
  lcd.setCursor(0, 1);
  lcd.print(Sp);
  buttonPress = buttonRead();
  if (buttonPress == 1) {
    //Left is pressed, do nothing
  }
  if (buttonPress == 2) {
    //Right is pressed, move to set Kp
    screen = 2;
    lcd.clear();
    lcd.home();
    lcd.print("Set Kp");
    return;
  }
  if (buttonPress == 3) {
    //Up is pressed, increment Setpoint up
    Sp++;
  }
  if (buttonPress == 4) {
    //Down is pressed, increment Setpoint down
    Sp--;
  }
  if (buttonPress == 5) {
    //Reset is pressed, reset Setpoint to original value
    Sp = setpoint;
  }
}

void screenKp() {
  //This screen is for setting temperature setpoint
  lcd.setCursor(0, 1);
  lcd.print(kp);
  buttonPress = buttonRead();
  if (buttonPress == 1) {
    //Left is pressed, go back to set Sp
    screen = 1;
    lcd.clear();
    lcd.home();
    lcd.print("Set Temperature");
    return;
  }
  if (buttonPress == 2) {
    //Right is pressed, move to set Kp
    screen = 3;
    lcd.clear();
    lcd.home();
    lcd.print("Set Ki");
    return;
  }
  if (buttonPress == 3) {
    //Up is pressed, increment Setpoint up
    kp++;
  }
  if (buttonPress == 4) {
    //Down is pressed, increment Setpoint down
    kp--;
  }
  if (buttonPress == 5) {
    //Reset is pressed, reset Setpoint to original value
    kp = pidProportional;
  }
}

void screenKi() {
  //This screen is for setting temperature setpoint
  lcd.setCursor(0, 1);
  lcd.print(ki);
  buttonPress = buttonRead();
  if (buttonPress == 1) {
    //Left is pressed, go back to set Sp
    screen = 2;
    lcd.clear();
    lcd.home();
    lcd.print("Set Kp");
    return;
  }
  if (buttonPress == 2) {
    //Right is pressed, move to set Kp
    screen = 4;
    lcd.clear();
    lcd.home();
    lcd.print("Ready to Run");
    lcd.setCursor(0, 1);
    lcd.print("Press Right");
    return;
  }
  if (buttonPress == 3) {
    //Up is pressed, increment Setpoint up
    ki++;
  }
  if (buttonPress == 4) {
    //Down is pressed, increment Setpoint down
    ki--;
  }
  if (buttonPress == 5) {
    //Reset is pressed, reset Setpoint to original value
    ki = pidIntegral;
  }
}

void screenRun() {
  //This screen is for setting temperature setpoint
  buttonPress = buttonRead();
  if (buttonPress == 1) {
    //Left is pressed, go back to set Sp
    screen = 3;
    lcd.clear();
    lcd.home();
    lcd.print("Set Ki");
    return;
  }
  if (buttonPress == 2) {
    //Right is pressed, Start PID and run.
    screen = 5;
    lcd.clear();
    lcd.home();
    return;
  }
  if (buttonPress == 3) {
    //Up is pressed, Do nothing

  }
  if (buttonPress == 4) {
    //Down is pressed, Do nothing

  }
  if (buttonPress == 5) {
    //Reset is pressed, go back to set Sp
    lcd.clear();
      lcd.home();
  lcd.print("Set Temperature");
    screen = 1;
    return;
  }
}

void screenRunning() {
  //This screen is for setting temperature setpoint
  pidController();
  lcd.clear();
  lcd.home();
  lcd.print("Setpoint = ");
  lcd.print(Sp);
  lcd.setCursor(0, 1);
  lcd.print("Sensed = ");
  //  lcd.print(tempIn);
  buttonPress = buttonRead();
  if (buttonPress == 1) {
    //Left is pressed, Do nothing
  }
  if (buttonPress == 2) {
    //Right is pressed, Do nothing
  }
  if (buttonPress == 3) {
    //Up is pressed, Do nothing

  }
  if (buttonPress == 4) {
    //Down is pressed, Do nothing

  }
  if (buttonPress == 5) {
    //Reset is pressed, go back to set Sp
    lcd.clear();
      lcd.home();
  lcd.print("Set Temperature");
    screen = 1;
    return;
  }
}

double getTemperature() {
top:
  byte i;
  byte present = 0;
  byte type_s = 0;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

  if ( !ds.search(addr)) {
    //    Serial.println("No more addresses.");
    //    Serial.println();
    ds.reset_search();
    delay(250);
    //return;
    goto top;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  //ds.depower();
  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //    Serial.print(data[i], HEX);
    //    Serial.print(" ");
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  return fahrenheit;
  //  Serial.print("  Temperature = ");
  //  Serial.print(celsius);
  //  Serial.print(" Celsius, ");
  //  Serial.print(fahrenheit);
  //  Serial.println(" Fahrenheit");

}

void pidController(){
   tempIn = getTemperature(); //Running off of F, can run off of Celsius, just has to change variable
  //  Serial.print(" tempIn ");
  //  Serial.println(tempIn);
  error = setpoint - tempIn;  //Gets error each time the loop is run
//  if (error >= -1 && error <= 1) {
//    if (tempMaintain == 0) {
//      tempMaintainStart = millis();
//      tempMaintain = 1;
//    }
//    tempMaintainDiff = (millis() - tempMaintainStart)/1000;
//    if (tempMaintainDiff >= 900) {
//      digitalWrite(led,HIGH);
//    }
//      }
  if (error < 1 && error > -1) { //Use Proportional and Integral control within 15 degrees of the setpoint
  errSum += ki*(error * (changeTime / 1000)); //convert changetime from milliseconds to seconds in order to properly calculate the integral value
  output = kp * error + errSum; //output is some value  
  }
  if (error >= errorDiffMax) {    //Use only proportional control when the error is outside of 15 degrees of the setpoint
    output = kp * error;  
  }
  
  //only changes pwm value if the correct time difference has passed.
  timedifference = millis() - lastTime;
  //  Serial.print ( "timedifference = ");
  //  Serial.println (timedifference);
  if (timedifference >= changeTime) {
    lastTime = millis();
    outputAdj = output * slope; // convert pid output to dimmer percentage
    if (outputAdj >= 100) {
      outputAdj = 100;
    }
    if (outputAdj <= 0 || error <= 0) {
      outputAdj = 0;
    }
    dimmer.set(outputAdj);
    //    Serial.print("Error = ");
    //    Serial.println(error);
    //    Serial.print("Dimmer Setting = ");
    //    Serial.println(dimmer);
    //    dimmer.setPower(output); //output ranges from 0-100%
    Serial.print(millis());
    Serial.print(" , ");
    Serial.print(setpoint);
    Serial.print(" , ");
    Serial.print(tempIn);
    Serial.print(" , ");
    Serial.print(error);
    Serial.print(" , ");
    Serial.print(output);
    Serial.print(" , ");
    Serial.println(outputAdj);

  }
}


void loop() {
  switch (screen) {
    case 1:
      screenSp();
      break;
    case 2:
      screenKp();
      break;
    case 3:
      screenKi();
      break;
    case 4:
      screenRun();
      break;
    case 5:
      screenRunning();
      break;
  }

  buttonRead();
}
