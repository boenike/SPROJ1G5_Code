// Include the library for the display
#include <LiquidCrystal_I2C.h>

// Defined Constants
const byte total_modes = 6 , LCD_ADDR = 0x27 , LCD_ROWS = 20 , LCD_COLS = 4 , resistanceRanges = 5 ;
const unsigned int buzzerTone = 1000 ;
const unsigned long adcDelay = 4 , bounceDelay = 170 ;
const float voltageFactor = 5.0 / 1023.0 ;
const char ohmSymbol = 0xF4 ;

// Button Pinouts
// SDA - Pin A4 | SCL - Pin A5
const byte prevButtonPin = 2 , nextButtonPin = 3 , buzzerPin = 6 , freqPin = 5 , relay2Pin = 7 , resistanceSelectPins [ 5 ] = { 8 , 9 , 10 , 11 , 12 } ;
const byte voltSel = 13 , relay1Pin = A0 , temperatureAdcPin = A1 , currentAdcPin = A2 , batteryAdcPin = A3 , resistanceAdcPin = A6 , voltageAdcPin = A7 ;

// Global Variables
int resistanceMeasuredValues [ 5 ] ;
byte mode_select ;
volatile bool next , prev ;
bool once = true;

// Declaring the LCD Display object
LiquidCrystal_I2C LCD ( LCD_ADDR , LCD_ROWS , LCD_COLS ) ;

void selectRelayCombination ( byte selector ) {
  // This function manipulates the output pins that connect to the relays in order to switch the input to the correct circuit
  switch ( selector ) {
    case 0 : digitalWrite ( relay2Pin , LOW )  ; digitalWrite ( relay1Pin , LOW )  ; break ; //VOLTAGE
    case 1 : digitalWrite ( relay2Pin , HIGH ) ; digitalWrite ( relay1Pin , LOW )  ; break ; //FREQUENCY
    case 2 : digitalWrite ( relay2Pin , LOW )  ; digitalWrite ( relay1Pin , HIGH ) ; break ; //RESTISTANCE
    default : break ;
  }
}

void setResistancePin ( byte onPin ) {
  // This function selects the specified MOSFET to be active, while switching off the others
  byte g ;
  for ( g = 0 ; g < resistanceRanges ; g++ ) {
    if ( g == onPin ) digitalWrite ( resistanceSelectPins [ g ] , LOW ) ;
    else digitalWrite ( resistanceSelectPins [ g ] , HIGH ) ;
  }
}

int getAbsVal ( int num  ) {
  // This function returns the absolute value of the input parameter
  if ( num >= 0 ) return num ;
  return - num ;
}

byte getClosestToHalf ( int * array , const byte size ) {
  // This function iterates through an array to find the index of the element that is closest to the value 512 and returns this index
  const int half = 512 ;
  int minValue = getAbsVal ( array [ 0 ] - half ) ;
  int currentAbsVal ;
  byte minIndex = 0 , i ;

  for ( i = 1 ; i < size ; i++ ) {
    currentAbsVal = getAbsVal ( array [ i ] - half ) ;
    if ( currentAbsVal < minValue ) {
      minValue = currentAbsVal ;
      minIndex = i ;
    }
  }
  return minIndex ;
}

bool checkDisconnectedRes ( void ) {
  // This function checks whether there is something connected to the terminals
  byte count = 0 ;
  byte g ;
  const int trigger = 80 ;
  const int full = 1023 ;
  for ( g = 0 ; g < resistanceRanges ; g++ ) {
    if ( getAbsVal ( full - resistanceMeasuredValues [ g ] ) <= trigger ) count++ ;
  }
  return count == resistanceRanges ;
}

void resistanceTest ( void ) {

  float resistance = 0.0 , resMultiplyFactor , resDivider = 1.0 ;
  const float resistanceFactors [ 5 ] = { 100.0 , 996.0 , 9930.0 , 100000.0 , 1000000.0 } ;
  const int full = 1023 ;
  byte samples = 40 , j , i , resHalfIndex ;
  String resUnit , resRange ;

  // Iterate through the loop to get multiple measurements
  for ( j = 0 ; j < samples ; j++ ) {

    // Select the specified range and save the measured ADC value to the corresponding array element
    for ( i = 0 ; i < resistanceRanges ; i++ ) {
      setResistancePin ( i ) ;
      delay ( adcDelay ) ;
      resistanceMeasuredValues [ i ] = analogRead ( resistanceAdcPin ) ;
    }

    // Switch off the last MOSFET after taking the final measurement
    digitalWrite ( resistanceSelectPins [ 4 ] , HIGH ) ;

    // Get the index of the range whose measured ADC value is closest to 512, i.e. the measured voltage is closest to 2.5V
    resHalfIndex = getClosestToHalf ( resistanceMeasuredValues , resistanceRanges ) ;

    // Adjust variables according to the selected range
    switch ( resHalfIndex ) {
      case 0  : resMultiplyFactor = resistanceFactors [ 0 ] ; resUnit = " "   ; resRange = "100R" ; resDivider = 1.0    ; break ;
      case 1  : resMultiplyFactor = resistanceFactors [ 1 ] ; resUnit = " "   ; resRange = "1K"   ; resDivider = 1.0    ; break ;
      case 2  : resMultiplyFactor = resistanceFactors [ 2 ] ; resUnit = " k"  ; resRange = "10K"  ; resDivider = 1000.0 ; break ;
      case 3  : resMultiplyFactor = resistanceFactors [ 3 ] ; resUnit = " k"  ; resRange = "100K" ; resDivider = 1000.0 ; break ;
      case 4  : resMultiplyFactor = resistanceFactors [ 4 ] ; resUnit = " k"  ; resRange = "1M"   ; resDivider = 1000.0 ; break ;
      default : break ;
    }

    // Calculate the resistance value and add it to the variable
    resistance += ( ( float ) resistanceMeasuredValues [ resHalfIndex ] * resMultiplyFactor ) / (float) ( full - resistanceMeasuredValues [ resHalfIndex ] ) ;
  }

  // Take the average value
  resistance /= (float) samples * resDivider ;

  // Check whether there is nothing connected to the input terminals
  if ( checkDisconnectedRes ( ) ) {
    LCD.setCursor ( 0 , 1 ) ;
    LCD.print ( "                    " ) ;
    LCD.setCursor ( 0 , 2 ) ;
    LCD.print ( "      Overload      " ) ;
  }

  else {
    // Display resistance in Mega Ohms
    if ( resHalfIndex == 4 && resistance > 1000.0 ) {
      resistance /= 1000.0 ;
      resUnit = " M" ;
    }
    // Print out the automatically selected measurement range and the resistance value
    LCD.setCursor ( 0 , 1 ) ;
    LCD.print ( "     Range: " + resRange + "    " ) ;
    LCD.setCursor ( 0 , 2 ) ;
    LCD.print ( "      " + String ( resistance ) + resUnit + ohmSymbol + "    " ) ;
  }
}

void voltageTest ( void ) {
  // Reset the variables
  const float lowerBound = 0.33 , upperCalibFactor = 3.96 ;
  float voltage = 0.0 , value ;
  byte samples = 100 , h ;

  // Take measurement samples and adjust the range according to the input selection pin
  for ( h = 0 ; h < samples ; h++ ) {
    value = (float) analogRead ( voltageAdcPin ) ;
    if ( digitalRead ( voltSel ) ) value *= upperCalibFactor ;
    value *= voltageFactor ;
    voltage += value ;
  }
  // Take the average value
  voltage /= (float) samples ;
  LCD.setCursor( 0 , 2 ) ;
  // Print out the voltage value to the LCD display
  if ( voltage >= lowerBound ) LCD.print ( "       " + String ( voltage ) + " V        " ) ;
  else LCD.print ( "                    " ) ;
}

// Interrupt handlers
void nextButtonPressed ( void ) {
  next = true ;
}

void prevButtonPressed ( void ) {
  prev = true ;
}

void currentTest ( void ) {
  float value = 0.0 , current = 0.0 , shuntVoltage , Vref ;
  const float currFactor = 1000.0 / 400.0 , unitValue = ( 5.0 / 1023.0 ) * 1000 ; // 1000mA per 400mV | (5V/1023)*1000 ~= 4.887 mV
  byte samples = 200 , idx ;

  if ( once ) {
    Vref = analogRead ( currentAdcPin ) ;
    once = false ;
  }

  for ( idx = 0 ; idx < samples ; idx++ ) {
    value += (float) analogRead ( currentAdcPin ) ;
    delay ( adcDelay ) ;
  }

  value /= (float) samples ;
  shuntVoltage = unitValue * value ;
  current = ( shuntVoltage - Vref ) * currFactor ;

  LCD.setCursor ( 0 , 2 ) ;

  if ( current > 0.0 ) LCD.print ( "     " + String ( current ) + " mA        " ) ;
  else LCD.print ( "         OL         " ) ;
}

void frequencyTest ( void ) {
  // 25Hz -> Lower-Bound Frequency | ~100 kHz -> Upper-Bound Frequency (+/- 5% error)

  float frequency = 0.0 ;
  const float millisecs = 1000000.0 , freqCorrectionLimit = 30000.0 , freqCalibFactor = 1.056 ;
  byte samples = 30 , i ;
  String freqUnit ;
  const unsigned long timeOut = 100000 ;
  unsigned long highPeriod , lowPeriod ;

  // Sample measurements
  for ( i = 0 ; i < samples ; i++  ) {
    highPeriod = pulseIn ( freqPin , HIGH , timeOut ) ;
    lowPeriod = pulseIn ( freqPin , LOW , timeOut ) ;
    frequency += millisecs / (float) ( highPeriod + lowPeriod ) ;
  }

  // Take the average for a more stable measurement
  frequency /= (float) samples ;

  // If the frequency is above the adjusting limit, calibrate the frequency with a certain mulitplier
  if ( frequency >= freqCorrectionLimit ) frequency *= freqCalibFactor ;

  if ( frequency >= 1000.0 ) {
    frequency /= 1000.0 ;
    freqUnit = " kHz" ;
  }

  else freqUnit = " Hz" ;

  LCD.setCursor ( 0 , 2 ) ;
  LCD.print ( "    " + String ( frequency ) + freqUnit + "        " ) ;
}

void continuityTest ( void ) {

  const float contCutoff = 10.0 , factor = 100.0 ;
  const int full = 1023 ;
  float value ;

  // Select the first range (100R) and get the resistance value
  setResistancePin ( 0 ) ;
  value = (float) analogRead ( resistanceAdcPin ) ;
  value = ( value * factor ) / (float) getAbsVal ( full - (int) value ) ;

  // If the measured resistance is below the threshold value, beep the buzzer and show the resistance value on the display
  if ( value <= contCutoff ) {
    tone ( buzzerPin , buzzerTone ) ;
    LCD.setCursor ( 0 , 1 ) ;
    LCD.print ( "      CONNECTED     " ) ;
    LCD.setCursor ( 0 , 2 ) ;
    LCD.print ( "      " + String ( value ) + " " + ohmSymbol + "         " ) ;
  }

  // Stop the beeping and show OL
  else {
    noTone ( buzzerPin ) ;
    LCD.setCursor ( 0 , 1 ) ;
    LCD.print ( "                    " ) ;
    LCD.setCursor ( 0 , 2 ) ;
    LCD.print ( "     Open Loop      " ) ;
  }
}

void batteryTest ( void ) {
  // This function checks whether the voltage of the supply 9V battery is the threshold voltage,
  // which is the safe minimum input voltage of the 7805 linear voltage regulator IC.
  const float batHalfThreshVolt = 3.5 ;
  float value = (float) analogRead ( batteryAdcPin ) * voltageFactor ;

  if ( value <= batHalfThreshVolt ) {
    LCD.setCursor ( 0 , 3 ) ;
    LCD.print ( "  Change Battery!   " ) ;
  }
}

void temperatureTest ( void ) {
  // This function gets the temperature value in degrees Celsius
  // Reset variables
  float celsius = 0.0 ;
  const float factor = ( ( 5.0 / 1023.0 ) / 0.01 ) ;
  const char degreeSymbol = 0xDF ;
  byte samples = 30 , j ;

  // Take multiple measurements
  for ( j = 0 ; j < samples ; j++ ) {
    celsius += analogRead ( temperatureAdcPin ) * factor ;
    delay ( adcDelay ) ;
  }

  // Take the average value
  celsius /= (float) samples ;

  // Print out the result
  LCD.setCursor ( 0 , 2 ) ;
  LCD.print ( "     " + String ( celsius ) + " " + degreeSymbol + "C       " ) ;
}

void setup ( ) {
  // Setting up initial values
  next = true ;
  prev = false ;
  byte h , mode_select = 0 ; ;

  for ( h = 0 ; h < resistanceRanges ; h++ ) {
    pinMode ( resistanceSelectPins [ h ] , OUTPUT ) ;
    digitalWrite ( resistanceSelectPins [ h ] , HIGH ) ;
  }

  // Button Pins
  pinMode ( nextButtonPin , INPUT_PULLUP ) ;
  pinMode ( prevButtonPin , INPUT_PULLUP ) ;

  // ADC Pins
  pinMode ( resistanceAdcPin  , INPUT ) ;
  pinMode ( currentAdcPin     , INPUT ) ;
  pinMode ( voltageAdcPin     , INPUT ) ;
  pinMode ( temperatureAdcPin , INPUT ) ;
  pinMode ( batteryAdcPin     , INPUT ) ;

  // Input Pins
  pinMode ( freqPin , INPUT ) ;
  pinMode ( voltSel , INPUT ) ;

  // Output Pins
  pinMode ( buzzerPin , OUTPUT ) ;
  pinMode ( relay1Pin , OUTPUT ) ;
  pinMode ( relay2Pin , OUTPUT ) ;

  // Interrupt handlers for switching modes
  attachInterrupt ( digitalPinToInterrupt ( nextButtonPin ) , nextButtonPressed , FALLING ) ;
  attachInterrupt ( digitalPinToInterrupt ( prevButtonPin ) , prevButtonPressed , FALLING ) ;

  // Initializing the LCD Display
  LCD.init ( ) ;
  LCD.backlight ( ) ;
  LCD.clear ( ) ;

  batteryTest ( ) ;
}

void loop ( ) {
  byte k ;
  // Cycle measurement modes accordingly when a button is pressed
  if ( next || prev ) {
    delay ( bounceDelay ) ;

    if ( next ) {
      mode_select++ ;
      next = false ;
    }

    if ( prev ) {
      mode_select-- ;
      prev = false ;
    }

    for ( k = 0 ; k < resistanceRanges ; k++ ) {
      digitalWrite ( resistanceSelectPins [ k ] , HIGH ) ;
    }

    LCD.clear ( ) ;
    noTone ( buzzerPin ) ;
    batteryTest ( ) ;

    if ( mode_select == total_modes + 1 ) mode_select = 1 ;
    if ( mode_select == 0 ) mode_select = total_modes ;

    LCD.setCursor ( 0 , 0 ) ;

    switch ( mode_select ) {
      case 1 : LCD.print ( "      Voltmeter     " ) ; selectRelayCombination ( 0 ) ; break ;
      case 2 : LCD.print ( "     Ampermeter     " ) ; selectRelayCombination ( 0 ) ; break ;
      case 3 : LCD.print ( "     Ohm-meter      " ) ; selectRelayCombination ( 2 ) ; break ;
      case 4 : LCD.print ( "   Frequency-meter  " ) ; selectRelayCombination ( 1 ) ; break ;
      case 5 : LCD.print ( "  Continuity test   " ) ; selectRelayCombination ( 2 ) ; break ;
      case 6 : LCD.print ( "  Temperature test  " ) ; selectRelayCombination ( 0 ) ; break ;
      default : break ;
    }
  }

  // Switch to the next measurement mode
  switch ( mode_select ) {
    case 1 : voltageTest     ( ) ; break ;
    case 2 : currentTest     ( ) ; break ;
    case 3 : resistanceTest  ( ) ; break ;
    case 4 : frequencyTest   ( ) ; break ;
    case 5 : continuityTest  ( ) ; break ;
    case 6 : temperatureTest ( ) ; break ;
    default : break ;
  }
}