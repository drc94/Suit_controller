#include <SoftwareSerial.h>

SoftwareSerial HC05 (2,3); // Rx -> pin 2  /  Tx -> pin 3
const int sensorPin = A0;
const int ledPin = 13;
const int pumpPin = 5;
int sensorValue = 0;
bool reading = false;
byte targetBuffer[3];
unsigned int count = 0;
//int pwm_aux = 0;
int pwm = 0;
int targetInt = 0;

/* PID VARIABLES */
double previous_error = 0;
int integral = 0; 
unsigned long last_time = 0;
double Kp = 15;
double Kd = 0.0001;
double Ki = 0.02;

void init_timer1()
{
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 4hz increments
  OCR1A = 7812;// = (32*10^6) / (1*256) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 bits for 256 prescaler
  TCCR1B |= (1 << CS12);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect){
//generates pulse wave of frequency 4Hz
  /*Serial.print("Target: ");
  Serial.println(targetInt);
  Serial.print("Lectura sensor: ");
  Serial.println(sensorValue);*/
  sensorValue = analogRead(sensorPin);
  pwm = pid_output(targetInt, sensorValue);
 // Serial.println(pwm);
  if(pwm < 50) pwm = 0;
  else if(pwm > 255) pwm = 255;
  analogWrite(pumpPin, pwm);
}

int pid_output(int setpoint, int measured_value)
{
  int output = 0;
  int error = setpoint - measured_value;
  if(error > 0)
  {
    unsigned long dt = millis() - last_time;
    last_time = millis();
    integral = integral + error * dt;
    double derivative = (error - previous_error) / dt;
    output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
  }
  else 
  {
    output = 0;
    integral = 0;
    previous_error = 0;
  }
  return (int)output;
}

void setup()
{
  delay(500);
  //Serial.begin(9600);
  HC05.begin(9600);
 // Serial.write("\nHC-05 bluetooth test\n");
  pinMode(ledPin, OUTPUT); //Establecer el pin 13 como salida
  pinMode(pumpPin, OUTPUT); //Establecer el pin 5 como activación de la bomba

  cli(); // Stops interruptions
  init_timer1();
  sei(); // Enables interruptions
}

void loop(){
  if(HC05.available()>=1)
   {
      char btInput = HC05.read(); //Leer un caracter
 
      if(btInput == 'h' or btInput == 'H') //Si es 'H', encender el LED
      {
         //digitalWrite(ledPin, HIGH);
         //digitalWrite(pumpPin, HIGH);
         //Serial.println("Bomba encendida");
         HC05.write("Pump On ");
      }
      
      else if(btInput == 'l' or btInput == 'L') //Si es 'L', apagar el LED
      {
         digitalWrite(ledPin, LOW);
         analogWrite(pumpPin, 0);
         targetInt = 0;
        // Serial.println("Bomba apagada");
         HC05.write("Pump Off");
      }

      else if(btInput == 'r' or btInput == 'R')
      {
        byte byteToSend[2];
        byteToSend[0] = sensorValue & 0xFF;
        byteToSend[1] = (sensorValue>>8) & 0xFF;
        
        HC05.write(byteToSend[0]);
        HC05.write(byteToSend[1]);
      }

      else if((btInput == 't' or btInput == 'T')||(reading)) //set target
      {
        reading = true;
        targetBuffer[count] = btInput;
        count++;
        if(count >= 3)
        {
          unsigned int aux = targetBuffer[2];
          aux = aux << 8;
          aux |= targetBuffer[1];

          targetInt = aux;
          reading = false;
          count = 0;
        }
      }
   }
}
