
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

// pin layout
#define P_START_BUTTON 4
#define P_SETTING_BUTTON 5
#define P_CURRENT 0
#define P_VOL_POT 1
#define P_PRESS_POT 2
#define P_RATE_POT 3
#define P_RED_LED 6
#define P_GREEN_LED 8
#define P_SERVO 10

// array indices
#define VOL 0
#define PRES 1
#define RATE 2

// measured physically, was stated to be 270 though
#define MAX_SERVO_POS 135 //250 
#define START_POS 0

// these speeds correspond to delays
// i.e higher delay, slower speed
#define MIN_SPEED 10     //
#define MAX_SPEED 0       //0

// current sensor constants
#define VIN A7
#define VCC 5.0
#define MODEL 2
#define QOV 0.5 * VCC

// potentiometer offset values
#define POT_PIN_OFFSET 1
#define POT_LCD_OFFSET 1

// x-coordinate of numeric values on LCD
#define LCD_H_SPACING 13











// Map from 0, 1024 to -radius, radius
// indices are volume, pressure, rate radii
const int8_t pot_map_radius[3] = {5, 5, 5};

// array shapes are [age group][parameter]
// order is adult, child, infant and volume, pressure, rate
const uint16_t nom_vals[3][3] = {{500, 300, 25}, {5, 5, 5}, {12, 25, 50}};
const uint8_t val_inc[3][3] = {{10, 5, 1}, {1, 1, 1}, {1, 1, 2}};

// base timing for adult, child, infant breaths
const uint16_t base_delays[3] = {950, 550, 200}; //950 550 200 

// constant strings for LCD
const char clear_value[] = "        ";
const char *setting_titles[3] = {"Volume: ", "Pressure: ", "Breath Rate: "};
const char *age_titles[3] = {"Adult", "Child", "Infant"};
const char *unit[3] = {" mL", "/10", " br/m"};

// Globals
bool ventilating = false;
bool time_to_squeeze = true;
bool time_to_release = true;
long squeeze_time = 0; //-100
long release_time = 0;
bool compression_state = 0;
bool halt = false;
bool warn_user = false;

bool can_read_age_button = true;
long age_button_time = -100;       // time age button was clicked -100
bool can_read_start_button = true;
long start_button_time = -100;     // time start button was clicked -100

int8_t pot_vals[3] = {0, 0, 0};
uint8_t age_state = 0;             // 0, 1, 2 -> adult, child, infant

//////////////////////////////////////////////////////////
/*
Sensitivity array is holding the sensitivy of the  ACS712
current sensor models. 
Do not change. All values are from page 5  of data sheet          
*/
double sensitivity[] ={
          0.185,// for ACS712ELCTR-05B-T
          0.100,// for ACS712ELCTR-20A-T
          0.066// for ACS712ELCTR-30A-T
         }; 
double voltage;
double current_sum = 0;
/////////////////////////////////////////////////////////////////////////////
Servo servo;
LiquidCrystal_I2C lcd(0x27, 20, 4);


///// FUNCTIONS /////


int map_pot_val(uint16_t unmapped_pot_val, int8_t pot_index)
{
  // map the potentiometer value to the desired radius
  return map(unmapped_pot_val, 0, 1024, pot_map_radius[pot_index], -pot_map_radius[pot_index]);
}

int16_t map_servo_pos(int16_t pos)
{
  
  return map(pos, 0, MAX_SERVO_POS, 0, 180);    // 180, 0
}

void lcd_update(uint8_t pot_index)
{
  // update lcd
  lcd.setCursor(LCD_H_SPACING - 1, pot_index + POT_LCD_OFFSET);
  lcd.print(clear_value);
  lcd.setCursor(LCD_H_SPACING, pot_index + POT_LCD_OFFSET);
  int16_t val = nom_vals[pot_index][age_state] + val_inc[pot_index][age_state] * pot_vals[pot_index];
  lcd.print(val);
  lcd.print(unit[pot_index]);
}


void lcd_startup()
{
  // lcd startup 
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Final Proyect");
  lcd.setCursor(6,2);
  lcd.print("ACTUATORS");
  lcd.setCursor(0,3);
  lcd.print("Romo, Reyes, Torres");
}

void lcd_init_titles()
{
  // initialize the lcd titles
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Volume:");
  lcd.setCursor(0, 2);
  lcd.print("Pressure:");
  lcd.setCursor(0, 3);
  lcd.print("Breath Rate:");
}

void lcd_init()
{
  // initialize the entire lcd
  lcd_init_titles();
 // lcd_update_age();
}
/////////////////////////nooooooooooooooooooooooooo
void lcd_warn()
{
  // show the warning message on the lcd
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("BLOCKAGE DETECTED");
  lcd.setCursor(0, 1);
  lcd.print("Press button");
  
  
}
/////////////////////////////////////////////////////////////////
void check_pot_vals()
{
  // check potentiometer values
  // if one has changed, update the lcd
  for (uint8_t i = 0; i < 3; ++i)
  {
    if (map_pot_val(analogRead(i + POT_PIN_OFFSET), i) != pot_vals[i]) 
    {
      pot_vals[i] = map_pot_val(analogRead(i + POT_PIN_OFFSET), i);
      lcd_update(i);
    }
  }
}

void check_age_setting()
{
  // check the age button, update age_state if changed
  if (!digitalRead(P_SETTING_BUTTON) && can_read_age_button) 
  {
    age_button_time = millis();
    can_read_age_button = false;
    age_state = (age_state + 1) % 3;
  //  lcd_update_age();
  }
}

void check_start_stop()
{
  // check if the start/stop button was clicked
  if (!digitalRead(P_START_BUTTON) && can_read_start_button) 
  {
    // cancel the warning if in place
    if (warn_user)
    {
    
      warn_user = false;
      lcd_init();
    }
    else
    {
      // start or stop ventilation
      start_button_time = millis();
      can_read_start_button = false;
      ventilating = !ventilating;
  
      green_led(ventilating);
    }
  }
}

bool check_halt()
{
  // check if operation was halted
  if (ventilating && warn_user)
  {
    return true;
  }
  bool was_ventilating = ventilating;
  check_start_stop();
  if (was_ventilating && !ventilating)
  {
    return true;
  }
  return false;
}

uint8_t get_servo_delay()
{
  // get the servo delay based on potentiometer value
  uint16_t des_delay = nom_vals[PRES][age_state];         ///PRES
  des_delay += val_inc[PRES][age_state] * pot_vals[VOL];
  des_delay = map(des_delay, nom_vals[PRES][age_state] - pot_map_radius[PRES] * val_inc[PRES][age_state], nom_vals[PRES][age_state] + pot_map_radius[PRES] * val_inc[PRES][age_state], 0, 10);
  return des_delay;
}

void drive_servo(uint16_t desired_pos)
{
  // drive the servo to its desired position
  uint16_t servo_pos = servo.read();

  // bag release direction
  if (servo_pos < desired_pos)
  {
    for (int16_t pos = servo_pos; pos < desired_pos; pos += 1) {
      halt = check_halt();
      if (halt)
      {
        break;
      }
      servo.write(pos);  
      delay(6);
    }
  }
  
  // bag squeeze direction
  else 
  {
    for (int16_t pos = servo_pos; pos > desired_pos; pos -= 1) {
      halt = check_halt();
      if (halt)
      {
        break;
      }
      servo.write(pos);
      update_current();
      uint8_t del = get_servo_delay(); 
      delay(del);
    }
  }
}

uint16_t get_ms_per_breath()
{
  // get breath rate given potentiometer value
  uint8_t breath_per_min = nom_vals[RATE][age_state] + val_inc[RATE][age_state] * pot_vals[RATE];
  return round(1000 / (breath_per_min / 60.0));
}

uint16_t vol_to_ang(uint16_t vol)
{
  // desired volume to servo angle formula, empirically determined
  return 4.89427e-7 * pow(vol, 3) - 8.40105e-4 * pow(vol, 2) + 0.64294*vol + 28.072327; //4.89427e-7 * pow(vol, 3) - 8.40105e-4 * pow(vol, 2) + 0.64294*vol + 28.072327;
}


void green_led(bool on)
{
  // turn on/off green LED (non PWM pin)
  digitalWrite(P_GREEN_LED, on ? HIGH : LOW);
}



double predict_current()
{
  // predict current given volume, pressure parameters
  // empirically determined formula
  uint16_t v = nom_vals[0][age_state] + val_inc[0][age_state] * pot_vals[0];
  uint8_t p = nom_vals[1][age_state] + val_inc[1][age_state] * pot_vals[1];
  return -1.041041e3 + 2.35386*v + 2.316309e-3*pow(v, 2) - 3.887507e1*p + 7.7198644*pow(p, 2) - 4.2099326e-2*v*p;
}

void update_current()
{
  // read the current value from the sensor, add it to the total sum
  uint16_t potentiometer_raw = analogRead(P_CURRENT);
  double voltage_raw =   (5.0 / 1023.0)* analogRead(VIN);// Read the voltage from sensor
  voltage =  voltage_raw - QOV + 0.012 ;// 0.000 is a value to make voltage zero when there is no current
  double current = voltage / sensitivity[MODEL] * -1; // sensor is flipped
  current_sum += current;
}

void setup() {
  // Setup code
  
  pinMode(P_RED_LED, OUTPUT);
  pinMode(P_GREEN_LED, OUTPUT);
  pinMode(P_SETTING_BUTTON, INPUT_PULLUP);
  pinMode(P_START_BUTTON, INPUT_PULLUP);
  servo.attach(P_SERVO);
  servo.write(map_servo_pos(START_POS)); // home the servo
  lcd.init();
  lcd.backlight();
  lcd_startup();
  green_led(true);
  delay(1000);
  lcd_init();
  green_led(false);
}


///////////////////////////// MAIN /////////////////////////////


void loop() {
  // Just some button debouncing here
  if (!can_read_age_button && millis() - age_button_time >= 500) {
    can_read_age_button = true;
  }
  if (!can_read_start_button && millis() - start_button_time >= 500) {
    can_read_start_button = true;
  }
  
  // If the user isn't being warned, update potentiometer values
  // If ventilation isn't occuring, update age setting
  if (!warn_user)
  {
    if (!ventilating)
    {
      check_age_setting();
    }
    check_pot_vals();
  }
  
  // if halted during a servo movement
  if (halt)
  {
    compression_state = 0;
    ventilating = false;
    drive_servo(map_servo_pos(START_POS));
    release_time = millis();
  }
  halt = check_halt();
  
  if (ventilating)
  {
    uint16_t ms_per_breath = get_ms_per_breath();
    int16_t post_breath_delay = ms_per_breath - base_delays[age_state]-2000;
   /* if (post_breath_delay < 0)
    {
      // just in case
      post_breath_delay = 1000;
    }
    */
    if (time_to_squeeze && !compression_state) 
    {
      // squeeze bag
      compression_state = 1;
      uint16_t des_vol = nom_vals[VOL][age_state];
      des_vol += val_inc[VOL][age_state] * pot_vals[VOL];
      uint16_t dest = vol_to_ang(des_vol);
      if (age_state == 0)
      {
       
        dest = map(dest, 100, 208, 0, 50);
      }
      drive_servo(map_servo_pos(dest));
      squeeze_time = millis();
    }
    else if (time_to_release && compression_state)
    {
      
      {
        compression_state = 0;
        drive_servo(map_servo_pos(START_POS));
        release_time = millis();
      }
      current_sum = 0;
    }
    
    if (compression_state)
    {
      update_current();
    }
    
    time_to_squeeze = millis() - release_time > post_breath_delay;
    time_to_release = millis() - squeeze_time > base_delays[age_state];
  }  }
  
