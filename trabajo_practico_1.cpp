
#include <Keypad.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

/* ---------- Sección de debug ---------- */
#define SERIAL_DEBUG_ENABLED 1 // Variable para habilitar el debug.
#if SERIAL_DEBUG_ENABLED
  #define DebugPrint(str)  \
  {                      \
    Serial.println(str); \
  }
#else
  #define DebugPrint(str)
#endif

/* Función de debug para imprimir estado y evento */
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
/* ---------- Fin sección de debug ---------- */

/* ---------- Sección de pines ---------- */
#define CAMBIAR_CLAVE_PIN 2
#define RESET_PIN 3
#define SERVO_PIN 6
#define LED_VERDE_PIN 8
#define LED_RGB_AZUL_PIN 9
#define LED_RGB_VERDE_PIN 10
#define LED_RGB_ROJO_PIN 11
#define PIR_PIN 12
#define LED_TIRA_PIN 13
#define FOTOSENSOR_PIN A0
/* ---------- Fin sección de pines ---------- */

/* ---------- Sección de constantes ---------- */
#define SERIAL_BAUD_RATE 9600
#define UMBRAL_DIFERENCIA_TIMEOUT 40 // Timeout para verificación genérica.
#define UMBRAL_DIFERENCIA_TIMEOUT_CORTO 3 * 1000 // Timeout de 3s para control de error de contraseña.
#define UMBRAL_DIFERENCIA_TIMEOUT_LARGO 5 * 1000 // Timeout de 10s para control de cierre de barrera.

#define APAGADO 0 // Valor para apagar la tira de LED.
#define PRENDIDO 1 // Valor para encender la tira de LED.
#define LED_TIRA_CANT 4 // Cantidad de LEDs de tira.
#define SENSOR_LUZ_UMBRAL 800 // Valor arbitrario para el encendido y apagado de la tira LED.

#define MAX_STATES 9 // Máxima cantidad de estados
#define MAX_EVENTS 9 //Máxima cantidad de eventos.
/* ---------- Fin sección de constantes ---------- */

/* ---------- Sección de estructuras de datos ---------- */
enum states
{
  ST_INIT,
  ST_ESPERA,
  ST_NO_AUTORIZADO,
  ST_AUTO_ENTRANDO,
  ST_AUTO_ADENTRO,
  ST_AUTO_SALIENDO,
  ST_AUTO_AFUERA,
  ST_ERROR,
  ST_CAMBIANDO_CLAVE,
} current_state;

volatile enum events {
  EV_PIR_DETECTADO,
  EV_CLAVE_CORRECTA,
  EV_CLAVE_INCORRECTA,
  EV_CONTINUE,
  EV_TIMEOUT_CORTO,
  EV_TIMEOUT_LARGO,
  EV_CAMBIO_DE_LUZ,
  EV_CAMBIO_CLAVE,
  EV_RESET,
} new_event;
/* ---------- Fin sección de estructuras de datos ---------- */

/* ---------- Sección de variables globales ---------- */
const byte FILAS = 4; // Número de filas del teclado.
const byte COLUMNAS = 3; // Número de columnas del teclado.

int previa_lectura_luz = -1; // Variable para manipular la cantidad de luz del fotosensor.
int brillo_previo_led_amarillo = 0; // Variable para manipular el brillo del LED amarillo.

long lct;      // Variable para el contador de tiempo. Asociada al evento CONTINUE.
long lct_corto; // Variable para el timeout de 3s. Se inicializa en cero debido a que no se va a ejecutar de no ser necesario.
long lct_largo; // Variable para el timeout de 10s. Se inicializa en cero debido a que no se va a ejecutar de no ser necesario.

bool timeout; // Variable de decisión asociada al contador de tiempo.
bool timeout_corto; // Variable de decisión asociada al timeout de 3s.
bool timeout_largo; // Variable de decisión asociada al timeout de 10s.

char TECLA; // Variable para almacenar la tecla ingresada.
char CLAVE[5]; // Array para almacenar la clave ingresada.
char CLAVE_MAESTRA[5] = "1234"; // Variable con la clave maestra.
char keys[FILAS][COLUMNAS] = { // Array con la definición de las teclas habilitadas.
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

String states_s[] = { //Vector de estados.
  "ST_INIT", 
  "ST_ESPERA", 
  "ST_NO_AUTORIZADO", 
  "ST_AUTO_ENTRANDO", 
  "ST_AUTO_ADENTRO", 
  "ST_AUTO_SALIENDO", 
  "ST_AUTO_AFUERA", 
  "ST_ERROR", 
  "ST_CAMBIANDO_CLAVE"
};

String events_s[] = { //Vector de eventos.
  "EV_PIR_DETECTADO", 
  "EV_CLAVE_CORRECTA", 
  "EV_CLAVE_INCORRECTA", 
  "EV_CONTINUE", 
  "EV_TIMEOUT_CORTO", 
  "EV_TIMEOUT_LARGO", 
  "EV_CAMBIO_DE_LUZ", 
  "EV_CAMBIO_CLAVE", 
  "EV_RESET"
};

Servo servo; // Variable de manipulación del servo.
/* ---------- Fin de sección de variables globales ---------- */

/* ---------- Máquina de estados ---------- */
typedef void (*transition)();
transition state_table[MAX_STATES][MAX_EVENTS] = { // Matriz para vincular estados y eventos.
    {error, error, error, init_, none, none, none, none, none},                                               // STATE INIT
    {autoSaliendo, autoEntrando, noAutorizado, modoEspera, none, none, modificarTira, cambiandoClave, reset}, // STATE ESPERA
    {error, error, error, none, modoEspera, none, modificarTira, none, reset},                                // STATE NO_AUTORIZADO
    {autoAdentro, none, none, none, none, none, modificarTira, none, reset},                                  // STATE AUTO_ENTRANDO
    {none, error, error, modoEspera, none, none, modificarTira, none, none},                                  // STATE AUTO_ADENTRO
    {autoSaliendo, error, error, none, none, autoAfuera, modificarTira, none, reset},                         // STATE AUTO_SALIENDO
    {autoSaliendo, error, error, modoEspera, none, none, modificarTira, none, reset},                         // STATE AUTO_AFUERA
    {error, error, error, error, error, error, error, error, reset},                                          // STATE ERROR
    {none, claveModificada, error, none, none, none, none, none, reset},                                      // STATE CAMBIANDO_CLAVE
};
// {PIR_DETECTADO, CLAVE_CORRECTA, CLAVE_INCORRECTA, CONTINUE, TO3, TO10, CAMBIO_DE_LUZ, CAMBIO_CLAVE, RESET}
/* ---------- Fin máquina de estados ---------- */

/* ---------- Tira de LED ---------- */
struct stTiraLED
{
  Adafruit_NeoPixel LEDs; // Objecto para el LED de la tira.
  int estado; // Estado de la tira de LED. [APAGADO|ENCENDIDO].
};
stTiraLED tira = {Adafruit_NeoPixel(LED_TIRA_CANT, LED_TIRA_PIN, NEO_RGB + NEO_KHZ800), APAGADO}; // Tira de LEDs.
/* ---------- Fin tira de LED ---------- */

/* ---------- Teclado ---------- */
byte INDICE = 0; // Indice del array.
byte pines_filas[FILAS] = {A1, A2, A3, 0}; // Array con la asociación de pines y filas.
byte pines_columnas[COLUMNAS] = {7, 5, 4}; // Array con la asociación de pines y columnas.
Keypad teclado = Keypad(makeKeymap(keys), pines_filas, pines_columnas, FILAS, COLUMNAS); // Definición de teclado con las configuraciones. Uso de Keypad Library.                                                 // almacena en un array la contraseña maestra
/* ---------- Fin teclado ---------- */

/*
 * Realiza la configuración inicial.
 */
void do_init()
{
  Serial.begin(SERIAL_BAUD_RATE);

  tira.LEDs.begin();       // Inicializa la libería de LEDs.
  tira.LEDs.show();        // Inicializa todos los pines LEDs apagados.
  servo.attach(SERVO_PIN); // Inicializa el Servo.

  pinMode(PIR_PIN, INPUT);            // Inicializa el pin del PIR.
  pinMode(LED_VERDE_PIN, OUTPUT);     // Inicializa el pin del LED verde.
  pinMode(LED_RGB_ROJO_PIN, OUTPUT);  // Inicializa el pin del LED RGB de color rojo.
  pinMode(LED_RGB_VERDE_PIN, OUTPUT); // Inicializa el pin del LED RGB de color verde.
  pinMode(LED_RGB_AZUL_PIN, OUTPUT);  // Inicializa el pin del LED RGB de color azul.

  current_state = ST_INIT; //Set del estado inicial.

  timeout = false; // Set del timeout general.
  lct = millis(); //Inicializo los milisegundos.

  attachInterrupt(digitalPinToInterrupt(CAMBIAR_CLAVE_PIN), dispararCambiarClave, RISING); //Attach de la interrupción para el botón de cambio de clave.
  attachInterrupt(digitalPinToInterrupt(RESET_PIN), dispararReset, RISING); //Attach de la interrupción para el botón de reset.
}

void dispararCambiarClave()
{
  // Serial.println("Cambio de clave");
  new_event = EV_CAMBIO_CLAVE;
  maquinaEstadosEstacionamiento(); // la interrupcion llama a la maquina de estados para moverla desde el evento.
}

void dispararReset()
{
  // Serial.println("Reseteo");
  new_event = EV_RESET;
  maquinaEstadosEstacionamiento(); // la interrupcion llama a la maquina de estados para moverla desde el evento.
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
    tira.LEDs.setPixelColor(i, tira.LEDs.Color(255, 0, 0)); // seteo el color verde con brillo el brillo adecuado GRB
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
  if (brillo_previo_led_amarillo == 256)
    brillo_previo_led_amarillo = 0;
  analogWrite(LED_RGB_ROJO_PIN, brillo_previo_led_amarillo);
  brillo_previo_led_amarillo++;
  digitalWrite(LED_RGB_VERDE_PIN, HIGH);
  digitalWrite(LED_RGB_AZUL_PIN, LOW);
}

void ledRGBAzul()
{
  digitalWrite(LED_RGB_ROJO_PIN, LOW);
  digitalWrite(LED_RGB_VERDE_PIN, LOW);
  digitalWrite(LED_RGB_AZUL_PIN, HIGH);
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

void cambiandoClave()
{
  // DebugPrint("Cambiando clave");
  ledRGBAzul();
  current_state = ST_CAMBIANDO_CLAVE;
}

void claveModificada()
{
  // DebugPrint("Clave modificada");
  ledRGBVerde();
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
  lct_corto = millis(); // reinicio el timeout de 3 segundos
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
  lct_largo = millis(); // reinicio el timeout de 10 segundos esperando que salga el auto
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

void reset()
{
  DebugPrint("Reseteando");
  ledRGBAmarilloTitilante();
  apagarIndicadorDeEntradaSalida();
  bajarBarrera();
  current_state = ST_ESPERA;
}

void none()
{
}

// ---------------------------------------------

/*
 * Funcion que checkea eventos de clave correcta o incorrecta. Y modifica clave si el modo es de cambio de clave
 */
bool verificarClave(int modoActual)
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
    if (modoActual == ST_CAMBIANDO_CLAVE)
    {
      strcpy(CLAVE_MAESTRA, CLAVE);
      DebugPrint("Clave Cambiadas"); // imprime en monitor serial que es correcta la clave mas
      new_event = EV_CLAVE_CORRECTA;
      INDICE = 0; // resetea el indice para guardar una nueva clave
      return true;
    }
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
  if (previa_lectura_luz - SENSOR_LUZ_UMBRAL > 0 && analogRead(FOTOSENSOR_PIN) - SENSOR_LUZ_UMBRAL < 0)
  {
    new_event = EV_CAMBIO_DE_LUZ;
    previa_lectura_luz = analogRead(FOTOSENSOR_PIN);
    return true;
  }
  if (previa_lectura_luz - SENSOR_LUZ_UMBRAL < 0 && analogRead(FOTOSENSOR_PIN) - SENSOR_LUZ_UMBRAL > 0)
  {
    new_event = EV_CAMBIO_DE_LUZ;
    previa_lectura_luz = analogRead(FOTOSENSOR_PIN);
    return true;
  }
}

void get_new_event()
{
  long ct = millis();
  int diferencia = (ct - lct);
  int diferenciaCorta = (ct - lct_corto);
  int diferenciaLarga = (ct - lct_largo);

  timeout = (diferencia > UMBRAL_DIFERENCIA_TIMEOUT) ? true : false;
  timeout_corto = (diferenciaCorta > UMBRAL_DIFERENCIA_TIMEOUT_CORTO) ? true : false;
  timeout_largo = (diferenciaLarga > UMBRAL_DIFERENCIA_TIMEOUT_LARGO) ? true : false;

  if (new_event == EV_CAMBIO_CLAVE || new_event == EV_RESET) // eventos interrupcion
  {
    return;
  }

  if (timeout_corto)
  {
    timeout_corto = false;
    lct_corto = ct;
    new_event = EV_TIMEOUT_CORTO; // genero el evento de timeout 3 segundos
    return;
  }

  if (timeout_largo)
  {
    timeout_largo = false;
    lct_largo = ct;
    new_event = EV_TIMEOUT_LARGO; // genero el evento de timeout 10 segundos
    return;
  }

  if (timeout)
  {
    // Doy acuse de la recepcion del timeout
    timeout = false;
    lct = ct;

    if (verificarClave(current_state) == true || verificarSensorPIR() == true || verificarSensorLuz() == true)
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
  if ((new_event >= 0) && (new_event < MAX_EVENTS) && (current_state >= 0) && (current_state < MAX_STATES))
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

  // Consumo el evento...)
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
