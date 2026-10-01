#include <Arduino.h>
#include <math.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>


Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

void setReports();
#define BNO8X_RESET -1 // Reset pin not wired

// Timing & filter variables
bool firstRead = true;
unsigned long curTime = 0;
unsigned long prevTime = 0;
float cutFreq = 3.5;      // Higher freq oscillation removed
float cutTimeConst = 0;

// Pins
const int XLEDPIN = 9;    
const int YLEDPIN = 10;   
const int BUTTONPIN = 1;  

// Angle limits
const float maxAngle = 45.0; // Slow blink angle
const float offLevel = 1.0;  // Solid led angle

// Blink timing
const unsigned long fastBlink = 50;
const unsigned long slowBlink = 1000;

// Button timing
const unsigned long dTapMax = 400;       // Max tap gap
const unsigned long dTapMin = 30;        // Min tap gap
const unsigned long longPressTime = 800; // long press hold time

// Screen refresh interval
const unsigned long screenRefresh = 100;


enum AxisMode {
  Both,
  X,
  Y,
  mCount
};

AxisMode axisMode = Both;
bool rawMode = false;   

// Raw data variables
float xRaw = 0;
float yRaw = 0;
float zRaw = 0;
float magRaw = 0;

// Smoothed data variables
float xs = 0;
float ys = 0;
float zs = 0;

// Angles (degrees)
float xAngle = 0;
float yAngle = 0;

// LED struct
struct Blinker {
  int pin;
  bool on;
  unsigned long toggle;
};

Blinker xled = {XLEDPIN, false, 0};
Blinker yled = {YLEDPIN, false, 0};

// Button state
bool debounceButton = false;  // Last debounced button state
unsigned long lastBounce = 0; // When debounce last changed
unsigned long startPress = 0; // When current press began
bool longPress = false;       // Long press handling
int taps = 0;                 // Tap count
unsigned long lastTap = 0;    // Tap release time


unsigned long lastScreen = 0; 


Adafruit_BNO08x bno08x(BNO8X_RESET);
sh2_SensorValue_t sensorValue;


// FUNCTIONS ///////////////////////////////////////////////////////////

// Inputs tilt angle outputs blink period
unsigned long angleBlink(float deg) {
  deg = fabs(deg);
  if (deg < offLevel) return 0;
  if (deg > maxAngle) deg = maxAngle;
  return fastBlink + (unsigned long)((slowBlink - fastBlink) * (deg / maxAngle));
}


void updateLed(Blinker &led, unsigned long period, bool enabled) {
  if (!enabled) {       // raw mode
    led.on = false;
    digitalWrite(led.pin, LOW);
    return;
  }
  if (period == 0) {    // level surface
    led.on = true;
    digitalWrite(led.pin, HIGH);
    return;
  }
  unsigned long now = millis();
  if (now - led.toggle >= period / 2) {
    led.on = !led.on;
    digitalWrite(led.pin, led.on ? HIGH : LOW);
    led.toggle = now;
  }
}

// Checks button for press variation
void buttonHandler() {
  bool btn = (digitalRead(BUTTONPIN) == HIGH); 
  unsigned long now = millis();

  // 1. Debounced edge detection
  if (btn != debounceButton && now - lastBounce > dTapMin) {
    lastBounce = now;
    debounceButton = btn;

    if (btn) {
      // determine if long press
      startPress = now;
      longPress = false;
    } else if (!longPress) {
      // Taps
      if (taps == 1 && now - lastTap < dTapMax) {
        // Double tap detected
        axisMode = (AxisMode)(((int)axisMode + 1) % (int)mCount);
        taps = 0;
      } else {
        // First/one tap
        taps = 1;
        lastTap = now;
      }
    }
  }

  // Long press
  if (debounceButton && !longPress && now - startPress > longPressTime) {
    rawMode = !rawMode;
    longPress = true; 
    taps = 0;
  }

  // One Tap
  if (taps == 1 && now - lastTap > dTapMax) {
    taps = 0;
  }
}


// SETUP //////////////////////////////////////////////////////////////////////
void setup() {

  // Display setup
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  display.init(135, 240);
  display.setRotation(1);
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, 1);
  display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  delay(1000);
  canvas.setTextColor(ST77XX_GREEN);
  canvas.setTextSize(2);
  Serial.begin(115200);

  Serial.println("Adafruit BNO08x test!");
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");
  setReports();

  pinMode(BUTTONPIN, INPUT_PULLDOWN);

  pinMode(XLEDPIN, OUTPUT);
  pinMode(YLEDPIN, OUTPUT);
  digitalWrite(XLEDPIN, LOW);
  digitalWrite(YLEDPIN, LOW);
}


// LOOP ///////////////////////////////////////////////////////////////////////
void loop() {

  buttonHandler();
  updateLed(xled, angleBlink(xAngle), !rawMode && (axisMode == Both || axisMode == X));
  updateLed(yled, angleBlink(yAngle), !rawMode && (axisMode == Both || axisMode == Y));

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  // Data collection
  float x = sensorValue.un.accelerometer.x;
  float y = sensorValue.un.accelerometer.y;
  float z = sensorValue.un.accelerometer.z;

  // Magnitude and time stamping
  curTime = millis();
  float accel = sqrt(x*x + y*y + z*z);         // Magnitude
  float deltaT = (curTime - prevTime) / 1000.0; // Time delta (sec)
  prevTime = curTime;

  // Raw data
  xRaw = x;
  yRaw = y;
  zRaw = z;
  magRaw = accel;

  // Store first reading to start data smoothing
  if (firstRead) {
    xs = x;
    ys = y;
    zs = z;
    firstRead = false;
    return;
  }

  // Data smoothing
  cutTimeConst = deltaT / ((1 / (2*M_PI*cutFreq)) + deltaT);
  xs = xs + cutTimeConst * (x - xs);
  ys = ys + cutTimeConst * (y - ys);
  zs = zs + cutTimeConst * (z - zs);

  // Tilt angle computation
  xAngle = atan2(xs, sqrt(ys*ys + zs*zs)) * 180.0 / M_PI;
  yAngle = atan2(ys, sqrt(xs*xs + zs*zs)) * 180.0 / M_PI;


  Serial.print("X: ");
  Serial.print(xs);
  Serial.print("  Y: ");
  Serial.print(ys);
  Serial.print("  Z: ");
  Serial.print(zs);
  Serial.print("  X ANGLE: ");
  Serial.print(xAngle);
  Serial.print("  Y ANGLE: ");
  Serial.println(yAngle);

// Screen printing 
  if (curTime - lastScreen >= screenRefresh) {
    lastScreen = curTime;

    canvas.fillScreen(ST77XX_BLUE);
    canvas.setCursor(0, 20);

    if (rawMode) {
      // Raw accelerometer screen
      canvas.print("MENU: ");
      canvas.println("RAW ACCEL");

      canvas.print("X: ");
      canvas.println(xRaw);
      canvas.print("Y: ");
      canvas.println(yRaw);
      canvas.print("Z: ");
      canvas.println(zRaw);
      canvas.print("MAG: ");
      canvas.println(magRaw);
    } else {
      // Level screen: mode and active angle(s)
      canvas.print("MENU: ");
      canvas.println("LEVEL");

      canvas.print("MODE: ");
      switch (axisMode) {
        case Both: canvas.println("X + Y");  break;
        case X:    canvas.println("X ONLY"); break;
        case Y:    canvas.println("Y ONLY"); break;
        default: break;
      }

      if (axisMode == Both || axisMode == X) {
        canvas.print("X: ");
        canvas.print(xAngle);
        canvas.println(" deg");
      }
      if (axisMode == Both || axisMode == Y) {
        canvas.print("Y: ");
        canvas.print(yAngle);
        canvas.println(" deg");
      }
    }

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());
  }
}


void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  } else {
    Serial.println("Set accelerometer report... success!");
  }
}
