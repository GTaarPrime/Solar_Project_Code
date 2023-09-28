#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Time.h>
#include <Time.h>
#include <Wire.h>


const char * ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -8 * 3600;
const int daylightOffset_sec = 3600;

const char * ssid[] = {"SM-G930W83609","SillyPeaHead","Pixel_4922"};
const char * password[] = {"ntrl1215","EarlierTudors1485","jameshotspot"};

const uint8_t wifi_timeout = 20;


//these define which pins control the stepper motors direction and pulse inputs
//these pins were changed when the circuit board was transferred
//these pins changed again to match the motor controller layout
#define pin_azimuth_direction 32
#define pin_azimuth_pulse 33
#define pin_azimuth_activate 25

#define pin_elevation_pulse 26
#define pin_elevation_direction 27
#define pin_elevation_activate 13

//these define the pins used for the manual operation buttons for controlling the stepper motors
//these pins were changed when the circuit board was transferred
#define button_azimuth_right 34
#define button_azimth_left 35
#define button_elevation_up 36
#define button_elevation_down 39

//this defines the pin which controls whether the code is operating in manual or automated operation
#define pin_Manual 18

//these define the pins used for the calibration hall sensors
//these pins were changed when the circuit board was transferred
#define pin_Hall90 16
#define pin_Hall180 17

#define pin_elevation_axis_direction 19
#define pin_azimthal_axis_direction 23

#define status_mode 1
#define status_elev_direction 2
#define status_azim_direction 3

uint8_t status = 0;

#define panel_longitude -123.12722631  
#define panel_latitude 49.17491793 

#define timezone -7
// Local solar time meridian = 15*timezone = -105
// #define LSTM -105.0
// Longitude correction for time = longitude - LSTM
#define longTC -18.12722631
// sin and cos of latitude (used in position calculation)
#define sinLat 0.756707593
#define cosLat 0.653753485
// Number of degrees the Earth moves around the sun in 1 day (360/365.25)
#define degs_per_day 0.98562628336
#define CMPS12_ADDRESS 192


#define default_elevation panel_latitude
#define default_azimuth 180

#define elevation_tolerance 1
#define azimuth_tolerance 1

//#define panel_longitude -2.148975
//#define panel_latitude 0.8582642



//these define the length of delay between each pulse to the stepper motor(minimum value of 5 micro seconds)
#define pwm_freq 
#define pwm_duty_cycle 0.5


struct tm current_time;
double last_solar_update; // In Julian Days
double solar_update_interval = 0.01041667; // In days -> 15min = 0.25 hours = 0.25 / 24 = 1/96


ESP32Time rtc(gmtOffset_sec);


enum pvMode {
  manual,
  tracking,
  waiting,
  sleeping,
  starting,
  calibrating
};

#define FALSE false
#define TRUE true

#define EL_UP FALSE
#define EL_DOWN TRUE

#define AZ_LEFT FALSE
#define AZ_RIGHT TRUE


pvMode mode;

bool ready;

bool elevation_Direction = EL_UP;
bool azimuth_Direction = AZ_LEFT;

double* target_elevation;
double* target_azimuth;

double sun_elevation;
double sun_azimuth;

double manual_elevation;
double manual_azimuth;

double elevation;
double azimuth;

// Start of debuging functions


// setting PWM properties
const int azimuthFreq = 1;
const int azimuthChannel = 0;
const int elevationFreq = 1;
const int elevationChannel = 1;
const int pwm_resolution = 16;


void setupPWM(){
  // configure LED PWM functionalitites
  int sixteenBitHalfDuty = 32767;
  ledcSetup(azimuthChannel, azimuthFreq, pwm_resolution);
  ledcSetup(elevationChannel, elevationFreq, pwm_resolution);
  Serial.println("PWM Setup");
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pin_azimuth_pulse, azimuthChannel);
  ledcAttachPin(pin_elevation_pulse, elevationChannel);
  Serial.println("Channel Setup");

  // activate PWM for pins 33 and 27 with 50% duty cycle
  ledcWrite(azimuthChannel, sixteenBitHalfDuty);
  ledcWrite(elevationChannel, sixteenBitHalfDuty);
  Serial.println("Activating pulses");

  // Perform test if motors are running
  
  digitalWrite(pin_azimuth_direction,0);
  digitalWrite(pin_azimuth_activate,0);
  digitalWrite(pin_elevation_direction,0);
  digitalWrite(pin_elevation_activate,1);
  Serial.println("Running elevation up");
  delay(1000);

  digitalWrite(pin_elevation_direction,1);
  Serial.println("Running elevation down");
  delay(1000);

  digitalWrite(pin_elevation_direction,0);
  digitalWrite(pin_elevation_activate,0); 
  digitalWrite(pin_azimuth_direction,0); 
  digitalWrite(pin_azimuth_activate,1);
  Serial.println("Running azimuth up");
  delay(1000);

  digitalWrite(pin_azimuth_direction,1);
  Serial.println("Running azimuth down");
  delay(1000);
 
  digitalWrite(pin_azimuth_direction,0); 
  digitalWrite(pin_azimuth_activate,0);


}
//end of debugging functions
uint8_t connectToWiFi(uint8_t wifi_option = 0)
{
  if(wifi_option > 2) return 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid[wifi_option], password[wifi_option]);
  Serial.println("Connecting to WiFi ..");
  uint8_t try_counter = wifi_timeout;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    try_counter--;
    if(try_counter==0) break;
    delay(1000);
  }
  if(WiFi.status() == WL_CONNECTED)
  {
    Serial.println(ssid[wifi_option]);
    Serial.println(WiFi.localIP());
    return 1;
  }
  disconnectWiFi();
  wifi_option++;
  return connectToWiFi(wifi_option);
}

// Synchronize the RTC with the NTP server - if it fails we just keep assuming the RTC is correct
void synchronizeTime()
{
  Serial.println("Synchronizing time");
  struct tm timeinfo;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    //status |= (1<<ERR_NTP_FAILED);
  }
  else
  {
    //status &= ~(1<<ERR_NTP_FAILED);
    Serial.println(&timeinfo, "Got time: %A, %B %d %Y %H:%M:%S");
    rtc.setTimeStruct(timeinfo);
  }
}

void disconnectWiFi()
{
  Serial.println("Disconnecting");
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}




void update_panel_position() {

  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(1);                     //Sends the register we wish to start reading from
  Wire.endTransmission();  

  Wire.requestFrom(CMPS12_ADDRESS, 5);       
  
  while(Wire.available() < 5);        // Wait for all bytes to come back (TODO: What if it doesn't?)
  
  uint8_t angle8, high_byte, low_byte, pitchAngle, rollAngle;
  uint16_t compassDirection;

  angle8 = Wire.read();               // Read back the 5 bytes
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitchAngle = Wire.read();
  rollAngle = Wire.read();
  
  compassDirection = high_byte;                 // Calculate 16 bit angle
  compassDirection <<= 8;
  compassDirection += low_byte;

  azimuth = compassDirection;
  elevation = 90.0 - pitchAngle;

}



double get_current_time()
{
  current_time = rtc.getTimeStruct();
  return julian_day(current_time);
}

double julian_day(struct tm& thetime)
{
	time_t epoch_time = mktime(&thetime);
	double epoch_days = floor(epoch_time / 86400.0);
	return epoch_days + 2440587.5 + day_fraction(thetime) - timezone / 24.0;
}

double day_fraction(struct tm& thetime)
{
  return (thetime.tm_hour * 3600.0 + thetime.tm_min * 60.0 + thetime.tm_sec) / 86400.0;
}

double days_since_year_start(struct tm& thetime)
{

  struct tm year_start;
  year_start.tm_mday = 1;
  year_start.tm_mon = 0;
  year_start.tm_hour = 0;
  year_start.tm_min = 0;
  year_start.tm_sec = 0;

  return julian_day(thetime) - julian_day(year_start);

}

void solar_position(double* el, double* az, struct tm* t)
	{

    double LT = 24.0 * day_fraction(*t); // Local clock time
    double d = days_since_year_start(*t);
		double B = degs_per_day * (d-81); // Angle of orbit since autumn equinox

		double sindB = sin(radians(B));
		double cosdB = cos(radians(B));

		double EoT = 19.74 * sindB * cosdB - 7.53 * cosdB - 1.5 * sindB;
		double LST = LT + (longTC + EoT) / 60.0;  // Local solar time
		double hour_angle = 15.0 * (LST - 12.0);

		double declination = 23.45 * sindB;

		double cosdHRA = cos(radians(hour_angle));
		double sindDec = sin(radians(declination));
		double cosdDec = cos(radians(declination));

		*el = degrees(asin(sindDec * sinLat + cosdDec * cosLat * cosdHRA));
		*az = degrees(acos((sindDec * cosLat - cosdDec * sinLat * cosdHRA)/cos(radians(elevation))));
  }

void update_solar_position()
{
  last_solar_update = get_current_time();
  solar_position(&sun_elevation,&sun_azimuth,&current_time);
}

void limit_target()
{
  if(*target_elevation  < 0)    *target_elevation = 0;
  if(*target_elevation  > 90)   *target_elevation = 90;
  if(*target_azimuth    < 90)   *target_azimuth = 80;
  if(*target_azimuth    > 270)  *target_azimuth = 280;
}

void poll_target_switch()
{
  if(digitalRead(pin_Manual))
  {
    if(mode != manual)
    {
      // If we just switched to manual, make the current position of the panel be the target
      manual_elevation = elevation;
      manual_azimuth = azimuth;
    }
    
    mode = manual;
    target_elevation = &manual_elevation;
    target_azimuth = &manual_azimuth;
  }
  else
  {
    mode = tracking;
    target_elevation = &sun_elevation;
    target_azimuth = &sun_azimuth;
  }
}

void track()
{
  // Limit target angle

  // Determine error
  double elevation_error;
  double azimuth_error;

  elevation_error = *target_elevation - elevation;  // -20 - -50 = 30
  azimuth_error = *target_azimuth - azimuth;

  // Determine direction of motion required

  elevation_Direction ^= (elevation_error < 0);
  azimuth_Direction ^= (azimuth_error < 0);

  // Set direction

  digitalWrite(pin_elevation_direction,elevation_Direction);
  digitalWrite(pin_azimuth_direction,azimuth_Direction);

  // Activate or deactivate when in tolerance

  digitalWrite(pin_elevation_activate,fabs(elevation_error) > elevation_tolerance);
  digitalWrite(pin_azimuth_activate,fabs(azimuth_error) > azimuth_tolerance);
}

void setup()
{
  Serial.begin(115200);
  mode = starting;
  setupPWM();
  synchronizeTime();
  //update_panel_position();
  //update_solar_position();
  //last_solar_update = get_current_time();
  //poll_target_switch();
  ready = TRUE; 
}


void loop() {
  // put your main code here, to run repeatedly:
  if(ready)
  {

  //Start debugging
  //End debbuging

    poll_target_switch();
    //poll_limit_switches();

    double now = get_current_time();
    update_panel_position();

    if(now - last_solar_update > solar_update_interval)
    {
      update_solar_position();
    }
    
    track();
  }
}
