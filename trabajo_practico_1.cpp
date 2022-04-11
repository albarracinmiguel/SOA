#include <Keypad.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// SECTOR PINES:
#define LED_TIRA_PIN 13
#define PIR_PIN 12
#define LED_RGB_ROJO_PIN 11
#define LED_RGB_VERDE_PIN 10
#define SERVO_PIN 3
#define LED_VERDE_PIN 2
#define FOTOSENSOR_PIN A0

#define SENSOR_LUZ_MINIMO 713  // valor minimo del sensor de luz de ambiente
#define SENSOR_LUZ_MAXIMO 1023 // valor maximo del sensor de luz de ambiente
#define BRILLO_MINIMO 0        // brillo minimo de la para led RGB pwm
#define BRILLO_MAXIMO 255      // brillo maximo de la para led RGB pwm
Servo servo;                   // variable para el servo

// SECTOR LEDS
#define LED_TIRA_CANT 4 // NUMERO DE PIXELES TOTALES
Adafruit_NeoPixel tiraLED = Adafruit_NeoPixel(LED_TIRA_CANT, LED_TIRA_PIN, NEO_RGB + NEO_KHZ800); // objeto para el led de la tira
int previaLecturaLuz = -1;

// SECTOR TECLADO
const byte FILAS = 4;    // define numero de filas
const byte COLUMNAS = 3; // define numero de columnas
char keys[FILAS][COLUMNAS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

byte pinesFilas[FILAS] = {9, 8, 7, 0};    // pines correspondientes a las filas
byte pinesColumnas[COLUMNAS] = {6, 5, 4}; // pines correspondientes a las columnas
Keypad teclado = Keypad(makeKeymap(keys), pinesFilas, pinesColumnas, FILAS, COLUMNAS); // crea objeto teclado con las configuraciones usando Keypad Library
char TECLA;                     // almacena la tecla presionada
char CLAVE[5];                  // almacena en un array 9digitos ingresados
char CLAVE_MAESTRA[5] = "1234"; // almacena en un array la contraseña maestra
byte INDICE = 0;                // indice del array

// Setea la luminosidad de los leds
void setBrilloTira(int oscuridadAmbiental)
{
  int brillo = map(oscuridadAmbiental, SENSOR_LUZ_MINIMO, SENSOR_LUZ_MAXIMO, BRILLO_MINIMO, BRILLO_MAXIMO); // mapea la brillo deacuerdo a la oscuridad del ambiente
  for (int i = 0; i < LED_TIRA_CANT; i++)                                                                   // recorre todos los leds
  {
    tiraLED.setPixelColor(i, tiraLED.Color(brillo, BRILLO_MINIMO, BRILLO_MINIMO)); // setea el color verde con brillo el brillo adecuado
  }
  tiraLED.show(); // muestra los cambios
}

// Modo de espera led rgb en amarillo verdoso
// TODO: todavia no se como hacer la maquina de estados pero se me ocurren 3 estados: en modo de espera (amarillo), auto pasando hasta el PIR O TIMEOUT (verde), contraseña incorrecta (rojo)
void modoDeEspera()
{
  digitalWrite(LED_RGB_VERDE_PIN, HIGH); // seteo el brillo del verde en digital
  analogWrite(LED_RGB_ROJO_PIN, 240);    // seteo el brillo del rojo en analogico
  delayMicroseconds(1000);               // espero 1ms para que se note la animacion
}

void setup()
{
  tiraLED.begin();         // inicializa la libreria
  tiraLED.show();          // inicializa todos los pines en 'off'
  servo.attach(SERVO_PIN); // inicializa el servo
  Serial.begin(9600);
  pinMode(PIR_PIN, INPUT);            // inicializa el pin del PIR
  pinMode(LED_VERDE_PIN, OUTPUT);     // inicializa el pin del led verde
  pinMode(LED_RGB_ROJO_PIN, OUTPUT);  // inicializa el pin del led RGB rojo
  pinMode(LED_RGB_VERDE_PIN, OUTPUT); // inicializa el pin del led RGB verde
}

void loop()
{
  if (previaLecturaLuz != analogRead(FOTOSENSOR_PIN)) // TODO: remover esta condicion y variable previaLecturaLuz y usar interrupcion
  {
    previaLecturaLuz = analogRead(FOTOSENSOR_PIN); // TODO: remover esta variable previaLecturaLuz y usar interrupcion
    setBrilloTira(analogRead(FOTOSENSOR_PIN));     // cambia color de la tira de leds
  }

  modoDeEspera(); // modo de espera para la el led RGB

  int PIRvalue = digitalRead(PIR_PIN); // leo el valor del PIR
  if (PIRvalue == HIGH)                // si el valor del pin es HIGH
  {
    digitalWrite(LED_VERDE_PIN, HIGH); // seteo el led verde en HIGH
    servo.write(90);                   // abre la puerta
  }
  else
  {
    digitalWrite(LED_VERDE_PIN, LOW); // seteo el led verde en LOW
    servo.write(0);                   // cierra la puerta
  }

  TECLA = teclado.getKey(); // obtiene tecla presionada y asigna una variable
  if (TECLA)                // comprueba que se haya presionado una tecla
  {
    CLAVE[INDICE] = TECLA; // almacena en array la tecla presionada
    INDICE++;              // incrementa indice en uno
    Serial.print(TECLA);   // envia un monitor serial la tecla presionada
  }

  if (INDICE == 4) // si ya se almacenaron los 6 digitos
  {
    if (!strcmp(CLAVE, CLAVE_MAESTRA)) // compara clave ingresada con clave maestra
      Serial.println(" Correcta");     // imprime en monitor serial que es correcta la clave mas
    else
      Serial.println(" Incorrecto"); // imprime en monitor serial que es incorrecta la clave

    INDICE = 0; // resetea el indice para guardar una nueva clave
  }
}
