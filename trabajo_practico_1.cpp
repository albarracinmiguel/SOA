#include <Keypad.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
Servo servo;
// SECTOR PINES:
#define LED_TIRA_PIN 2 // PIN CONECTADO NEO PIXEL
#define PIR_PIN 12
#define FOTOSENSOR_PIN A0
#define LED_VERDE_PIN 3
#define LED_RGB_ROJO_PIN 11
#define LED_RGB_AZUL_PIN 10
#define LED_RGB_VERDE_PIN 9
#define SERVO_PIN 6

// SECTOR LEDS
#define LED_TIRA_CANT 4 // NUMERO DE PIXELES TOTALES
Adafruit_NeoPixel tiraLED = Adafruit_NeoPixel(LED_TIRA_CANT, LED_TIRA_PIN, NEO_RGB + NEO_KHZ800);

// SECTOR TECLADO
char contrasena[] = "3366";
char codigo[4];
int previaLecturaLuz = -1;
int cont = 0;
const byte ROWS = 4;
const byte COLS = 3;
char hexaKeys[ROWS][COLS] =
    {
        {'1', '2', '3'},
        {'4', '5', '6'},
        {'7', '8', '9'},
        {'*', '0', '#'},
};
byte rowPins[ROWS] = {13, 8, 7};
byte colPins[COLS] = {5, 4, 3};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

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
  digitalWrite(LED_RGB_AZUL_PIN, LOW); // apago el azul
  for (int i = 0; i < 255; i++)        // recorro de  0 a 255 para emular el titilado
  {
    analogWrite(LED_RGB_VERDE_PIN, i); // seteo el brillo del verde
    analogWrite(LED_RGB_ROJO_PIN, i);  // seteo el brillo del rojo
    delayMicroseconds(1000);           // espero 1ms para que se note la animacion
  }

  for (int i = 255; i > 0; i--) // recorro de 255 a 0 para emular el titilado
  {
    analogWrite(LED_RGB_VERDE_PIN, i); // seteo el brillo del verde
    analogWrite(LED_RGB_ROJO_PIN, i);  // seteo el brillo del rojo
    delayMicroseconds(1000);           // espero 1ms para que se note la animacion
  }
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
  pinMode(LED_RGB_AZUL_PIN, OUTPUT);
}

void loop()
{
  // esto lo lee todo el tiempo, hace que el emulador reviente por eso agruegue este checkeo
  if (previaLecturaLuz != analogRead(FOTOSENSOR_PIN)) // TODO: remover esta condicion y variable previaLecturaLuz y usar interrupcion
  {
    previaLecturaLuz = analogRead(FOTOSENSOR_PIN);
    // setBrilloTira(analogRead(FOTOSENSOR_PIN)); // si se descomenta temblequea el servo
  }

  // modoDeEspera(); // si se descomenta temblequea el servo

  int PIRvalue = digitalRead(PIR_PIN);
  // Serial.print(PIRvalue);
  // Serial.print("\t");
  // Serial.print(HIGH);
  // Serial.println();
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

  // char customKey = customKeypad.getKey();
  // if (customKey != NO_KEY)
  // {
  //   codigo[cont] = customKey;
  //   Serial.println(codigo[cont]);
  //   cont = cont + 1;
  //   Serial.println(codigo[cont]);
  //   if (cont == 4)
  //   {
  //     if (codigo[0] == contrasena[0] && codigo[1] == contrasena[1] && codigo[2] == contrasena[2] &&
  //         codigo[3] == contrasena[3])
  //     {
  //       // digitalWrite(verdeLED,HIGH);
  //       Serial.println(codigo);
  //       // digitalWrite(verde,LOW);
  //       delay(4000);
  //     }
  //     else
  //     {
  //       delay(4000);
  //     }
  //     cont = 0;
  //   }
  // }
}
