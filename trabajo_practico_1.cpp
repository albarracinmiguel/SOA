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

Servo servo;

// SECTOR LEDS
#define LED_TIRA_CANT 4 // NUMERO DE PIXELES TOTALES
Adafruit_NeoPixel tiraLED = Adafruit_NeoPixel(LED_TIRA_CANT, LED_TIRA_PIN, NEO_RGB + NEO_KHZ800);
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

Keypad teclado = Keypad(makeKeymap(keys), pinesFilas, pinesColumnas, FILAS, COLUMNAS); // crea objeto

char TECLA;                     // almacena la tecla presionada
char CLAVE[5];                  // almacena en un array 9digitos ingresados
char CLAVE_MAESTRA[5] = "1234"; // almacena en un array la contraseña maestra
byte INDICE = 0;                // indice del array

// Setea la luminosidad de los leds
void setBrilloTira(int oscuridadAmbiental)
{
  int brillo = map(oscuridadAmbiental, 713, 1023, 0, 255); // mapea la brillo deacuerdo a la oscuridad del ambiente
  for (int i = 0; i < LED_TIRA_CANT; i++)                  // recorre todos los leds
  {
    tiraLED.setPixelColor(i, tiraLED.Color(brillo, 0, 0)); // setea el color verde con brillo
  }
  tiraLED.show(); // muestra los cambios
}

// Modo de espera led rgb titilando en amarillo
// TODO: todavia no se como hacer la maquina de estados pero se me ocurren 3 estados: en modo de espera (amarillo), auto pasando hasta el PIR O TIMEOUT (verde), contraseña incorrecta (rojo)
void modoDeEspera()
{

  digitalWrite(LED_RGB_VERDE_PIN, HIGH); // seteo el brillo del verde
  analogWrite(LED_RGB_ROJO_PIN, 240);    // seteo el brillo del rojo
  delayMicroseconds(1000);               // espero 1ms para que se note la animacion
}

void setup()
{
  tiraLED.begin();         // inicializa la libreria
  tiraLED.show();          // inicializa todos los pines en 'off'
  servo.attach(SERVO_PIN); // inicializa el servo
  Serial.begin(9600);
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_VERDE_PIN, OUTPUT);
  pinMode(LED_RGB_ROJO_PIN, OUTPUT);
  pinMode(LED_RGB_VERDE_PIN, OUTPUT);
}

void loop()
{
  if (previaLecturaLuz != analogRead(FOTOSENSOR_PIN)) // TODO: remover esta condicion y variable previaLecturaLuz y usar interrupcion
  {
    previaLecturaLuz = analogRead(FOTOSENSOR_PIN);
    setBrilloTira(analogRead(FOTOSENSOR_PIN));
  }

  modoDeEspera();

  int PIRvalue = digitalRead(PIR_PIN);
  if (PIRvalue == HIGH)
  {
    digitalWrite(LED_VERDE_PIN, HIGH);
    servo.write(90); // abre la puerta
  }
  else
  {
    digitalWrite(LED_VERDE_PIN, LOW);
    servo.write(0); // cierra la puerta
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

    INDICE = 0;
  }
}
