
#include <Keypad.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// Habilitacion de debug para la impresion por el puerto serial
//----------------------------------------------
#define SERIAL_DEBUG_ENABLED 1 // variable para habilitar el debug
#if SERIAL_DEBUG_ENABLED
#define DebugPrint(str)  \
  {                      \
    Serial.println(str); \
  }
#else
#define DebugPrint(str)
#endif

#define DebugPrintEstado(estado, evento)                           \
  {                                                                \
    String est = estado;                                           \
    String evt = evento;                                           \
    String str;                                                    \
    str = "-----------------------------------------------------"; \
    DebugPrint(str);                                               \
    str = "EST-> [" + est + "]: " + "EVT-> [" + evt + "].";        \
    DebugPrint(str);                                               \
    str = "-----------------------------------------------------"; \
    DebugPrint(str);                                               \
  }
//----------------------------------------------

// SECTOR PINES:
//----------------------------------------------
#define LED_TIRA_PIN 13
#define PIR_PIN 12
#define LED_RGB_ROJO_PIN 11
#define LED_RGB_VERDE_PIN 10
#define LED_RGB_AZUL_PIN 9
#define SERVO_PIN 3
#define LED_VERDE_PIN 2
#define FOTOSENSOR_PIN A0
//----------------------------------------------

// OTRAS CONSTANTES:
//----------------------------------------------
#define UMBRAL_DIFERENCIA_TIMEOUT 50
#define UMBRAL_DIFERENCIA_TIMEOUT_CORTO 3 * 1000
#define UMBRAL_DIFERENCIA_TIMEOUT_LARGO 5 * 1000
#define SERIAL_BAUD_RATE 9600

#define SENSOR_LUZ_UMBRAL 800  // valor del umbral para prender o apagar la tira led
#define SENSOR_LUZ_MAXIMO 1023 // valor maximo del sensor de luz de ambiente
#define BRILLO_MINIMO 0        // brillo minimo de la para led RGB pwm
#define BRILLO_MAXIMO 255      // brillo maximo de la para led RGB pwm
#define LED_TIRA_CANT 4        // cantidad de leds que tiene la tira
Servo servo;                   // variable para el servo
//----------------------------------------------

// SECTOR MAQUINA DE ESTADOS:
//----------------------------------------------

enum states
{
  ST_INIT,
  ST_ESPERA,
  ST_NO_AUTORIZADO,
  ST_AUTO_ENTRANDO,
  ST_AUTO_ADENTRO,
  ST_AUTO_SALIENDO,
  ST_AUTO_AFUERA,
  ST_ERROR
} current_state;
String states_s[] = {"ST_INIT", "ST_ESPERA", "ST_NO_AUTORIZADO", "ST_AUTO_ENTRANDO", "ST_AUTO_ADENTRO", "ST_AUTO_SALIENDO", "ST_AUTO_AFUERA", "ST_ERROR"};

enum events
{
  EV_PIR_DETECTADO,
  EV_CLAVE_CORRECTA,
  EV_CLAVE_INCORRECTA,
  EV_CONTINUE,
  EV_TIMEOUT_CORTO,
  EV_TIMEOUT_LARGO,
  EV_CAMBIO_DE_LUZ,

} new_event;
String events_s[] = {"EV_PIR_DETECTADO", "EV_CLAVE_CORRECTA", "EV_CLAVE_INCORRECTA", "EV_CONTINUE", "EV_TIMEOUT_CORTO", "EV_TIMEOUT_LARGO", "EV_CAMBIO_DE_LUZ"};

#define MAXSTATES 8
#define MAXEVENTS 7

typedef void (*transition)();
transition state_table[MAXSTATES][MAXEVENTS] = {
    // PIR_DETECTADO CLAVE_CORRECTA CLAVE_INCORRECTA CONTINUE TO3 TO10 CAMBIO_DE_LUZ
    {error, error, error, init_, none, none, none},                                    // STATE INIT
    {autoSaliendo, autoEntrando, noAutorizado, modoEspera, none, none, modificarTira}, // STATE ESPERA
    {error, error, error, none, modoEspera, none, modificarTira},                      // STATE NO_AUTORIZADO
    {autoAdentro, none, none, none, none, none, modificarTira},                        // STATE AUTO_ENTRANDO
    {none, error, error, modoEspera, none, none, modificarTira},                       // STATE AUTO_ADENTRO
    {autoSaliendo, error, error, none, none, autoAfuera, modificarTira},               // STATE AUTO_SALIENDO
    {autoSaliendo, error, error, modoEspera, none, none, modificarTira},               // STATE AUTO_AFUERA
    {error, error, error, error, error, error, error}                                  // STATE ERROR
};

long lct;      // variable para el contador de tiempo asociado al evento continue
long lctCorto; // variable para el timeout de 3 segundos inicializada en 0 porque no va a correr a no ser que sea necesaria
long lctLargo; // variable para el timeout de 10 segundos inicializada en 0 porque no va a correr a no ser que sea necesaria

bool timeout;
bool timeoutCorto;
bool timeoutLargo;

//----------------------------------------------

// SECTOR LEDS
int previaLecturaLuz = -1;

#define APAGADO 0
#define PRENDIDO 1
struct stTiraLED
{
  Adafruit_NeoPixel LEDs; // objeto para el led de la tira
  int estado;
};
stTiraLED tira = {Adafruit_NeoPixel(LED_TIRA_CANT, LED_TIRA_PIN, NEO_RGB + NEO_KHZ800), APAGADO};

//----------------------------------------------

int brilloPrevioLedAmarillo = 0;

// SECTOR TECLADO
const byte FILAS = 4;    // define numero de filas
const byte COLUMNAS = 3; // define numero de columnas
char keys[FILAS][COLUMNAS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

byte pinesFilas[FILAS] = {A1, 8, 7, 0};                                                // pines correspondientes a las filas
byte pinesColumnas[COLUMNAS] = {6, 5, 4};                                              // pines correspondientes a las columnas
Keypad teclado = Keypad(makeKeymap(keys), pinesFilas, pinesColumnas, FILAS, COLUMNAS); // crea objeto teclado con las configuraciones usando Keypad Library
char TECLA;                                                                            // almacena la tecla presionada
char CLAVE[5];                                                                         // almacena en un array 9digitos ingresados
char CLAVE_MAESTRA[5] = "1234";                                                        // almacena en un array la contraseña maestra
byte INDICE = 0;                                                                       // indice del array

// Modo de espera led rgb en amarillo verdoso
// TODO: todavia no se como hacer la maquina de estados pero se me ocurren 3 estados: en modo de espera (amarillo), auto pasando hasta el PIR O TIMEOUT (verde), contraseña incorrecta (rojo)
void modoDeEspera()
{
  digitalWrite(LED_RGB_AZUL_PIN, LOW);   // seteo el brillo del azul en digital
  digitalWrite(LED_RGB_VERDE_PIN, HIGH); // seteo el brillo del verde en digital
  analogWrite(LED_RGB_ROJO_PIN, 240);    // seteo el brillo del rojo en analogico
  delayMicroseconds(1000);               // espero 1ms para que se note la animacion
}

//----------------------------------------------

void do_init()
{
  Serial.begin(SERIAL_BAUD_RATE);

  tira.LEDs.begin();       // inicializa la libreria
  tira.LEDs.show();        // inicializa todos los pines en 'off'
  servo.attach(SERVO_PIN); // inicializa el servo

  pinMode(PIR_PIN, INPUT);            // inicializa el pin del PIR
  pinMode(LED_VERDE_PIN, OUTPUT);     // inicializa el pin del led verde
  pinMode(LED_RGB_ROJO_PIN, OUTPUT);  // inicializa el pin del led RGB rojo
  pinMode(LED_RGB_VERDE_PIN, OUTPUT); // inicializa el pin del led RGB verde
  pinMode(LED_RGB_AZUL_PIN, OUTPUT);  // inicializa el pin del led RGB azul

  // Inicializo el evento inicial
  current_state = ST_INIT;

  timeout = false;
  lct = millis();
}

void bajarBarrera()
{
  servo.write(0);
}

void subirBarrera()
{
  servo.write(90);
}

void apagarTiraLed()
{
  tira.estado = APAGADO;
  for (int i = 0; i < LED_TIRA_CANT; i++)
  {
    tira.LEDs.setPixelColor(i, tira.LEDs.Color(0, 0, 0)); // seteo el color verde con brillo el brillo adecuado
  }
  tira.LEDs.show(); // muestra los cambios
}

void prenderTiraLed()
{
  tira.estado = PRENDIDO;
  for (int i = 0; i < LED_TIRA_CANT; i++)
  {
    tira.LEDs.setPixelColor(i, tira.LEDs.Color(0, 255, 0)); // seteo el color verde con brillo el brillo adecuado
  }
  tira.LEDs.show(); // muestra los cambios
}

void apagarIndicadorDeEntradaSalida()
{
  digitalWrite(LED_VERDE_PIN, LOW);
}

void encenderIndicadorDeEntradaSalida()
{
  digitalWrite(LED_VERDE_PIN, HIGH);
}

void ledRGBRojo()
{
  // digitalWrite(LED_RGB_ROJO_PIN, HIGH);
  analogWrite(LED_RGB_ROJO_PIN, 255);
  digitalWrite(LED_RGB_VERDE_PIN, LOW);
  digitalWrite(LED_RGB_AZUL_PIN, LOW);
}

void ledRGBVerde()
{
  digitalWrite(LED_RGB_ROJO_PIN, LOW);
  digitalWrite(LED_RGB_VERDE_PIN, HIGH);
  digitalWrite(LED_RGB_AZUL_PIN, LOW);
}

void ledRGBAmarilloTitilante()
{
  // delayMicroseconds(500);
  if (brilloPrevioLedAmarillo == 256)
    brilloPrevioLedAmarillo = 0;
  analogWrite(LED_RGB_ROJO_PIN, brilloPrevioLedAmarillo);
  brilloPrevioLedAmarillo++;
  digitalWrite(LED_RGB_VERDE_PIN, HIGH);
  digitalWrite(LED_RGB_AZUL_PIN, LOW);
}

void modificarTira()
{
  !tira.estado ? prenderTiraLed() : apagarTiraLed();
}

//----------------------------------------------

void init_()
{
  DebugPrintEstado(states_s[current_state], events_s[new_event]);
  bajarBarrera();
  apagarTiraLed();
  apagarIndicadorDeEntradaSalida();
  current_state = ST_ESPERA;
}

void modoEspera()
{
  // DebugPrint("Volviendo a estado de espera");
  ledRGBAmarilloTitilante();
  current_state = ST_ESPERA;
}

void autoEntrando() // aka contrasenia correcta
{
  DebugPrint("Esta entrando un auto");
  ledRGBVerde();
  delay(1);
  encenderIndicadorDeEntradaSalida();
  subirBarrera();
  current_state = ST_AUTO_ENTRANDO;
}

void noAutorizado() // aka contrasenia incorrecta
{
  lctCorto = millis(); // reinicio el timeout de 3 segundos
  ledRGBRojo();
  current_state = ST_NO_AUTORIZADO;
}

void autoAdentro()
{
  ledRGBAmarilloTitilante();
  apagarIndicadorDeEntradaSalida();
  bajarBarrera();
  current_state = ST_AUTO_ADENTRO;
}

void autoSaliendo()
{
  lctLargo = millis(); // reinicio el timeout de 10 segundos esperando que salga el auto
  subirBarrera();
  encenderIndicadorDeEntradaSalida();
  current_state = ST_AUTO_SALIENDO;
}

void autoAfuera()
{
  apagarIndicadorDeEntradaSalida();
  bajarBarrera();
  current_state = ST_AUTO_AFUERA;
}

void error()
{
  ledRGBRojo();
  current_state = ST_ERROR;
}

void none()
{
}

// ---------------------------------------------

/*
 * Funcion que checkea eventos de clave correcta o incorrecta.
 */
bool verificarClave()
{
  TECLA = teclado.getKey(); // obtiene tecla presionada y asigna una variable
  if (TECLA)                // comprueba que se haya presionado una tecla
  {
    CLAVE[INDICE] = TECLA; // almacena en array la tecla presionada
    INDICE++;              // incrementa indice en uno
    DebugPrint(TECLA);     // envia un monitor serial la tecla presionada
  }
  if (INDICE == 4) // si ya se almacenaron los 6 digitos
  {
    if (!strcmp(CLAVE, CLAVE_MAESTRA)) // compara clave ingresada con clave maestra
    {
      new_event = EV_CLAVE_CORRECTA; // Evento de auto entrando
      DebugPrint("Clave Correcta");  // imprime en monitor serial que es correcta la clave mas
    }
    else
    {
      new_event = EV_CLAVE_INCORRECTA; // Evento de clave incorrecta
      DebugPrint("Clave Incorrecta");  // imprime en monitor serial que es incorrecta la clave
    }

    INDICE = 0; // resetea el indice para guardar una nueva clave
    return true;
  }
  return false;
}

bool verificarSensorPIR()
{
  if (digitalRead(PIR_PIN) == HIGH) // si el sensor detecta movimiento
  {
    new_event = EV_PIR_DETECTADO; // Evento de movimiento detectado
    return true;
  }
  return false;
}

bool verificarSensorLuz()
{
  if (previaLecturaLuz != analogRead(FOTOSENSOR_PIN)) // TODO: remover esta condicion y variable previaLecturaLuz y usar interrupcion
  {
    previaLecturaLuz = analogRead(FOTOSENSOR_PIN); // TODO: remover esta variable previaLecturaLuz y usar interrupcion
    new_event = EV_CAMBIO_DE_LUZ;                  // Evento de luz detectada
    return true;
  }
}

void get_new_event()
{
  long ct = millis();
  int diferencia = (ct - lct);
  int diferenciaCorta = (ct - lctCorto);
  int diferenciaLarga = (ct - lctLargo);

  timeout = (diferencia > UMBRAL_DIFERENCIA_TIMEOUT) ? true : false;
  timeoutCorto = (diferenciaCorta > UMBRAL_DIFERENCIA_TIMEOUT_CORTO) ? true : false;
  timeoutLargo = (diferenciaLarga > UMBRAL_DIFERENCIA_TIMEOUT_LARGO) ? true : false;

  if (timeoutCorto)
  {
    timeoutCorto = false;
    lctCorto = ct;
    new_event = EV_TIMEOUT_CORTO; // genero el evento de timeout 3 segundos
    return;
  }

  if (timeoutLargo)
  {
    timeoutLargo = false;
    lctLargo = ct;
    new_event = EV_TIMEOUT_LARGO; // genero el evento de timeout 10 segundos
    return;
  }

  if (timeout)
  {
    // Doy acuse de la recepcion del timeout
    timeout = false;
    lct = ct;

    if (verificarClave() == true || verificarSensorPIR() == true || verificarSensorLuz() == true)
    {
      return;
    }
  }
  // Genero evento dummy ....
  new_event = EV_CONTINUE;
}

void maquinaEstadosEstacionamiento()
{
  get_new_event();
  if ((new_event >= 0) && (new_event < MAXEVENTS) && (current_state >= 0) && (current_state < MAXSTATES))
  {
    if (new_event != EV_CONTINUE)
    {
      DebugPrintEstado(states_s[current_state], events_s[new_event]);
    }
    state_table[current_state][new_event]();
  }
  else
  {
    // DebugPrintEstado(states_s[ST_ERROR], events_s[EV_UNKNOW]);
  }
  // Consumo el evento...
  new_event = EV_CONTINUE;
}

//----------------------------------------------

// Funciones de arduino !.
//----------------------------------------------

void setup()
{
  do_init();
}

void loop()
{
  maquinaEstadosEstacionamiento();
}
