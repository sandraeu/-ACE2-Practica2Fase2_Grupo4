#include <SoftwareSerial.h>
#include <Servo.h>

//-------------------------------------------------------------------

//Communication
SoftwareSerial btSerial(11, 12); // RX, TX
String serialData = "";

int data0 = A0;
int data1 = A1;
int data2 = A2;

//Movement
char movementCommand = 'p';
char movementDirection = 'p';

//Start/Stop Button
int sButton = 10;
boolean sButtonPressed = false;

//Servo
Servo servo_1;
int servoOutput = 9;
boolean servoActivated = false;

//Line Sensors
int lineSensorLeft = 6;
int lineSensorRight = 7;
boolean lineMode = false;
int estadoLine = 0;
int antizq = 2;
int antder = 2;
int valorizq = 0;
int valorder = 0;

//Ultrasonic


const int TriggerPin = 4; 
const int EchoPin = 5;
int TriggerEstado = LOW; 
int Intervalo = 1; 
int Intervalo2 = 1000; 
int PrintEstado = LOW; 
unsigned long previousMillis = 0;
int distanciaMin = 20;
//-------------------------------------------------------------------
int count_objetos =0;
int cprueba =0;
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  //--------------------------------------------------------------------------
  //Communication
  Serial.begin(9600);
  while (!Serial) { ; }
  btSerial.begin(9600);
  
  pinMode(data0, OUTPUT);
  pinMode(data1, OUTPUT);
  pinMode(data2, OUTPUT);
  
  digitalWrite(data0, LOW);
  digitalWrite(data1, LOW);
  digitalWrite(data2, LOW);
  
  //Start/Stop Button
  pinMode(sButton, INPUT_PULLUP);

  //Servo
  servo_1.attach(servoOutput);
  servo_1.write(0);

  //Line Sensors
  pinMode(lineSensorLeft, INPUT);
  pinMode(lineSensorRight, INPUT);

  //Ultrasonic
  pinMode(TriggerPin,OUTPUT); 
  pinMode(EchoPin,INPUT);
  
  //--------------------------------------------------------------------------
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*
 * Códigos
 * serialData = 
 * mx   Balancear sin moverse
 * mw   Mover adelante
 * ms   Mover atrás
 * ma   Giro izquierda
 * md   Giro derecha
 * mp   Detener
 * sm   Mover servomotor
 */


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{
  //---------------------------Serial Communication---------------------------
  serialData = "";
  while(btSerial.available())
  {
    serialData.concat(char(btSerial.read()));
  }
  if(serialData.length() > 0)
  {
    //btSerial.print("command - ");
    //btSerial.println(serialData);
  }
  //--------------------------------------------------------------------------

  
  //Código sensor línea
  if(estadoLine == 1){
    valorizq = digitalRead(lineSensorLeft);
    valorder = digitalRead(lineSensorRight);

    if(valorizq != antizq || valorder != antder){
        if(valorizq == HIGH && valorder == LOW){ // el sensor esta en linea negra y el otro en blanca
          movementCommand = 'd';
          antizq = 1;
          antder = 0;
          //btSerial.println(movementDirection);
        }else if(valorizq == LOW && valorder == HIGH){ //el sensor esta en linea blanca
          movementCommand = 'a';
          antizq = 0;
          antder = 1;
         // btSerial.println(movementDirection);
        }else if( valorizq == HIGH && valorder == HIGH){

          movementCommand = 'x';
          antizq = 1;
          antder = 1;
         // btSerial.println(movementDirection);
        }else if(valorizq == LOW && valorder == LOW){
          estadoLine =0;
          antizq = 0;
          antder = 0;
          movementCommand = 'w';
         // btSerial.println(movementDirection);
        }
      }


      
  }
  

  
  //-----------------------------------Servo----------------------------------
  if(serialData.equals("sm"))
  {
    if(!servoActivated)
    {
      servo_1.write(90);
      movementCommand = 'p';
      servoActivated = true;
    }
    else
    {
      servo_1.write(0);
      movementCommand = 'x';
      servoActivated = false;
    }
  }
  //--------------------------------------------------------------------------

//Código sensor ultrasónico'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

  unsigned long currentMillis = millis(); 
  //millis captura
  if (currentMillis-previousMillis >= Intervalo) 
  { 
    previousMillis = currentMillis;
    if (TriggerEstado == LOW)
    {
      (TriggerEstado = HIGH);
    }
    else {
      (TriggerEstado = LOW);
    }
  }
  // chequear si se puede pintar o no
  if (currentMillis-previousMillis >= 50) 
  { 
    previousMillis = currentMillis;
    if (PrintEstado == LOW)
    {
      (PrintEstado = HIGH);
    }
    else 
    {
      (PrintEstado = LOW);
    }
  }
  digitalWrite(TriggerPin,TriggerEstado);
  int Duracion, Distancia; //variables
  Duracion = pulseIn(EchoPin,HIGH);
  Distancia = (Duracion/2) / 29.1;

  //adecuado pintar


  //Serial.println(Distancia);
  if (Distancia > 0)
  {
    Serial.print(Distancia);
    Serial.println("cm");
  }

  if (Distancia > 0 && Distancia <= distanciaMin )
  {
    Serial.println("se detecto objeto");
    
      if(estadoLine == 1){
        cprueba++;
        movementCommand='x';
        estadoLine=0;
      }
  }
  
  //''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
  //---------------------esquivar-------------
if(Distancia == 35){
  
}
  //-------------------------------------------------------------------------

  //-----------------------------Start/Stop Button----------------------------
  if(digitalRead(sButton) == LOW)
  {
    if(!sButtonPressed)
    {
      if(movementDirection == 'p')
      {
        movementCommand = 'x';
      }
      else
      {
        movementCommand = 'p';
      }
      sButtonPressed = true;
    }
  }
  else
  {
    sButtonPressed = false;
  }
  //--------------------------------------------------------------------------

  //----------------------------send data bt----------------------------------
  count_objetos ++;
if(count_objetos == 5){
  
  btSerial.print(cprueba);
  btSerial.print(":");
  btSerial.print(movementDirection);
  btSerial.print(":");
  btSerial.println(estadoLine);
  count_objetos =0;
  Serial.println(movementDirection);
}
  //--------------------------------------------------------------------------
  //-----------------------------Movement Control-----------------------------
  if(serialData.startsWith("m"))
  {
    movementCommand = serialData.charAt(1);
  }
  if(movementCommand != '\0')
  {
    movementDirection = movementCommand;
    switch(movementDirection)
    {
      case 'p':
        digitalWrite(data0, LOW);
        digitalWrite(data1, LOW);
        digitalWrite(data2, LOW);
        Serial.println("mando p");
        break;
      case 'w':
        digitalWrite(data0, HIGH);
        digitalWrite(data1, LOW);
        digitalWrite(data2, LOW);
        Serial.println("mando w");
        break;
      case 's':
        digitalWrite(data0, LOW);
        digitalWrite(data1, HIGH);
        digitalWrite(data2, LOW);
        Serial.println("mando s");
        break;
      case 'a':
        digitalWrite(data0, HIGH);
        digitalWrite(data1, HIGH);
        digitalWrite(data2, LOW);
        Serial.println("mando a");
        break;
      case 'd':
        digitalWrite(data0, LOW);
        digitalWrite(data1, LOW);
        digitalWrite(data2, HIGH);
        Serial.println("mando d");
        break;
      case 'z':
        digitalWrite(data0, HIGH);
        digitalWrite(data1, LOW);
        digitalWrite(data2, HIGH);
        Serial.println("mando z");
        break;
      case 'c':
        digitalWrite(data0, LOW);
        digitalWrite(data1, HIGH);
        digitalWrite(data2, HIGH);
        Serial.println("mando c");
        break;
      case 'x':
        digitalWrite(data0, HIGH);
        digitalWrite(data1, HIGH);
        digitalWrite(data2, HIGH);
        Serial.println("mando x");
        break;
      case 'i':
        if(estadoLine==0){
          estadoLine=1;
          digitalWrite(data0, HIGH);
         digitalWrite(data1, LOW);
         digitalWrite(data2, LOW);
          movementDirection ='w';
        }else{
          estadoLine=0;
          movementDirection = 'x';
        }
        Serial.println("mando i");
        break;
    }
    movementCommand = '\0';
   // btSerial.print("move -> ");
   //btSerial.println(movementDirection);
  }
  //--------------------------------------------------------------------------


}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
