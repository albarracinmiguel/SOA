
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
#define UMBRAL_DIFERENCIA_TECLADO 20              // Timeout para pulling teclado.
#define UMBRAL_DIFERENCIA_TIMEOUT 700             // Timeout para verificación genérica.
#define UMBRAL_DIFERENCIA_TIMEOUT_CORTO 5 * 1000  // Timeout de 3s para control de error de contraseña.
#define UMBRAL_DIFERENCIA_TIMEOUT_LARGO 10 * 1000 // Timeout de 10s para control de cierre de barrera.

#define APAGADO 0             // Valor para apagar la tira de LED.
#define PRENDIDO 1            // Valor para encender la tira de LED.
#define LED_TIRA_CANT 4       // Cantidad de LEDs de tira.
#define DIFERENCIA_SENSOR_LUZ_UMBRAL = 0; // Límite de diferencia entre luz led y umbral del sensor de luz.
#define SENSOR_LUZ_UMBRAL 800 // Valor arbitrario para el encendido y apagado de la tira LED.

#define MIN_STATES = 0; // Minima cantidad de estados.
#define MAX_STATES 9 // Máxima cantidad de estados.

#define MIN_EVENT = 0; // Minima cantidad de eventos
#define MAX_EVENTS 9 // Máxima cantidad de eventos.

#define BARRERA_CERRADA = 0; // Grados de la barrera cerrada para el servo.
#define BARRERA_ABIERTA = 90; // Grados de la barrera abierta para el servo.

#define BRILLO_MINIMO_LED = 0; // Brillo minimo de un led.
#define BRILLO_MAXIMO_LED = 255; // Brillo máximo de un led.

#define INICIO_NUEVA_CLAVE = 13; // Valor del indice de comienzo de la nueva clave.
#define FIN_NUEVA_CLAVE = 17; // Valor del indice de fin de la nueva clave

#define VERIFICACION_ENTRADA_SERIAL = 0; // Valor de verificación de si se ingreso algún dato serial.
#define VERIFICACION_ACCION_TECLADO = -1; // Valor de verificación para saber si se realizo una acción de cambio de clave o reset.

#define MIN_INDICE = 0; // Valor minimo del indice para el cambio de clave o reset.
#define MAX_INDICE = 4; // Valor máximo para el indice de cambio de clave o reset.
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
const byte FILAS = 4;    // Número de filas del teclado.
const byte COLUMNAS = 3; // Número de columnas del teclado.

int previa_lectura_luz = -1;        // Variable para manipular la cantidad de luz del fotosensor.
int brillo_previo_led_amarillo = 0; // Variable para manipular el brillo del LED amarillo.

long lct;         // Variable para el contador de tiempo. Asociada al evento CONTINUE.
long lct_corto;   // Variable para el timeout de 3s. Se inicializa en cero debido a que no se va a ejecutar de no ser necesario.
long lct_largo;   // Variable para el timeout de 10s. Se inicializa en cero debido a que no se va a ejecutar de no ser necesario.
long lct_teclado; // Variable para el timeout de pulling teclado. Se inicializa en cero debido a que no se va a ejecutar de no ser necesario.

bool timeout;         // Variable de decisión asociada al contador de tiempo.
bool timeout_corto;   // Variable de decisión asociada al timeout de 3s.
bool timeout_largo;   // Variable de decisión asociada al timeout de 10s.
bool timeout_teclado; // Variable de decisión asociada al timeout del teclado.

char TECLA;                     // Variable para almacenar la tecla ingresada.
char CLAVE[5];                  // Array para almacenar la clave ingresada.
char CLAVE_MAESTRA[5] = "1234"; // Variable con la clave maestra.
char NUEVA_CLAVE[5];            // Array para almacenar la nueva clave ingresada.
char keys[FILAS][COLUMNAS] = {  // Array con la definición de las teclas habilitadas.
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

String states_s[] = { // Vector de estados.
    "ST_INIT",
    "ST_ESPERA",
    "ST_NO_AUTORIZADO",
    "ST_AUTO_ENTRANDO",
    "ST_AUTO_ADENTRO",
    "ST_AUTO_SALIENDO",
    "ST_AUTO_AFUERA",
    "ST_ERROR"};

String events_s[] = { // Vector de eventos.
    "EV_PIR_DETECTADO",
    "EV_CLAVE_CORRECTA",
    "EV_CLAVE_INCORRECTA",
    "EV_CONTINUE",
    "EV_TIMEOUT_CORTO",
    "EV_TIMEOUT_LARGO",
    "EV_CAMBIO_DE_LUZ",
    "EV_CAMBIO_CLAVE",
    "EV_RESET"};

Servo servo; // Variable de manipulación del servo.
/* ---------- Fin de sección de variables globales ---------- */

/* ---------- Máquina de estados ---------- */
typedef void (*transition)();
transition state_table[MAX_STATES][MAX_EVENTS] = {
    // Matriz para vincular estados y eventos.
    {error, error, error, init_, none, none, none, none, none},                                               // STATE INIT
    {autoSaliendo, autoEntrando, noAutorizado, modoEspera, none, none, modificarTira, cambiandoClave, reset}, // STATE ESPERA
    {none, none, none, none, modoEspera, none, modificarTira, none, reset},                                   // STATE NO_AUTORIZADO
    {autoAdentro, none, none, none, none, none, modificarTira, none, reset},                                  // STATE AUTO_ENTRANDO
    {none, error, error, modoEspera, none, none, modificarTira, none, none},                                  // STATE AUTO_ADENTRO
    {autoSaliendo, error, error, none, none, autoAfuera, modificarTira, none, reset},                         // STATE AUTO_SALIENDO
    {autoSaliendo, error, error, modoEspera, none, none, modificarTira, none, reset},                         // STATE AUTO_AFUERA
    {error, error, error, error, error, error, error, error, reset},                                          // STATE ERROR
};
// {PIR_DETECTADO, CLAVE_CORRECTA, CLAVE_INCORRECTA, CONTINUE, TO3, TO10, CAMBIO_DE_LUZ, CAMBIO_CLAVE, RESET}
/* ---------- Fin máquina de estados ---------- */

/* ---------- Tira de LED ---------- */
struct stTiraLED
{
  Adafruit_NeoPixel LEDs; // Objecto para el LED de la tira.
  int estado;             // Estado de la tira de LED. [APAGADO|ENCENDIDO].
};
stTiraLED tira = {Adafruit_NeoPixel(LED_TIRA_CANT, LED_TIRA_PIN, NEO_RGB + NEO_KHZ800), APAGADO}; // Tira de LEDs.
/* ---------- Fin tira de LED ---------- */

/* ---------- Teclado ---------- */
byte INDICE = 0;                                                                         // Indice del array.
byte pines_filas[FILAS] = {A1, A2, A3, 0};                                               // Array con la asociación de pines y filas.
byte pines_columnas[COLUMNAS] = {7, 5, 4};                                               // Array con la asociación de pines y columnas.
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

  current_state = ST_INIT; // Set del estado inicial.

  timeout = false; // Set del timeout general.
  lct = millis();  // Inicializo los milisegundos.
}

/*
 * Se realiza la acción de bajar la barrera.
 */
void bajarBarrera()
{
  servo.write(BARRERA_ABIERTA); // Se mueve el servo a 0 grados.
}

/*
 * Se realiza la acción de subir la barrera.
 */
void subirBarrera()
{
  servo.write(BARRERA_CERRADA); // Se mueve el servo a 90 grados.
}

/*
 * Se apaga la tira de LEDs.
 */
void apagarTiraLed()
{
  tira.estado = APAGADO;  // Set de estado apagado.
  for (size_t i = 0; i < LED_TIRA_CANT; i++)
  {
    tira.LEDs.setPixelColor(i, tira.LEDs.Color(BRILLO_MINIMO_LED, BRILLO_MINIMO_LED, BRILLO_MINIMO_LED)); // Set del LED sin color.
  }

  tira.LEDs.show(); // Se muestran los cambios.
}

/*
 * Se prende la tira de LEDs.
 */
void prenderTiraLed()
{
  tira.estado = PRENDIDO; // Set de estado prendido.
  for (size_t i = 0; i < LED_TIRA_CANT; i++)
  {
    tira.LEDs.setPixelColor(i, tira.LEDs.Color(BRILLO_MAXIMO_LED, BRILLO_MINIMO_LED, BRILLO_MINIMO_LED)); // Set del LED sin color.
  }

  tira.LEDs.show(); // Se muestran los cambios.
}

/*
 * Se apaga el LED que indica la entrada|salida de un vehículo.
 */
void apagarIndicadorDeEntradaSalida()
{
  digitalWrite(LED_VERDE_PIN, LOW); // Set de valor 0 para el LED.
}

/*
 * Se encender el LED que indica la entrada|salida de un vehículo.
 */
void encenderIndicadorDeEntradaSalida()
{
  digitalWrite(LED_VERDE_PIN, HIGH); // Set de valor 1 para el LED.
}

/*
 * Se enciende el LED color rojo que indica que el vehículo no esta habilitado para ingresar.
 */
void ledRGBRojo()
{
  analogWrite(LED_RGB_ROJO_PIN, BRILLO_MAXIMO_LED);   // Set máximo color rojo para el LED.
  digitalWrite(LED_RGB_VERDE_PIN, LOW); // Set mínimo color verde para el LED.
  digitalWrite(LED_RGB_AZUL_PIN, LOW);  // Set mínimo color azul para el LED.
}

/*
 * Se enciende el LED color verde que indica que el vehículo esta habilitado para ingresar|salir.
 */
void ledRGBVerde()
{
  digitalWrite(LED_RGB_ROJO_PIN, LOW);   // Set mínimo color rojo para el LED.
  digitalWrite(LED_RGB_VERDE_PIN, HIGH); // Set máximo color verde para el LED.
  digitalWrite(LED_RGB_AZUL_PIN, LOW);   // Set mínimo color azul para el LED.
}

/*
 * Se enciende el LED color amarillo titilante que indica el estado de espera por un vehículo.
 */
void ledRGBAmarilloTitilante()
{
  if (brillo_previo_led_amarillo == BRILLO_MAXIMO_LED)
  { // Reset del color amarillo. Se utiliza para hacer que el LED titile.
    brillo_previo_led_amarillo = BRILLO_MINIMO_LED;
  }

  analogWrite(LED_RGB_ROJO_PIN, brillo_previo_led_amarillo); // Set de color amarillo.

  brillo_previo_led_amarillo++; // Aumento de intensidad en color amarillo.

  digitalWrite(LED_RGB_VERDE_PIN, HIGH); // Set máximo color verde para el LED.
  digitalWrite(LED_RGB_AZUL_PIN, LOW);   // Set mínimo color azul para el LED.
}

/*
 * Se enciende el LED color azul que indica el cambio de contraseña.
 */
void ledRGBAzul()
{
  digitalWrite(LED_RGB_ROJO_PIN, LOW);  // Set mínimo color rojo para el LED.
  digitalWrite(LED_RGB_VERDE_PIN, LOW); // Set mínimo color verde para el LED.
  digitalWrite(LED_RGB_AZUL_PIN, HIGH); // Set máximo color azul para el LED.
}

/*
 * Se modifica la tira de LED al estado contrario en el que se encuentra.
 */
void modificarTira()
{
  !tira.estado ? prenderTiraLed() : apagarTiraLed(); // Condicional para saber si prender o apagar la luz. Depende del estado actual.
}

/*
 * Inicializó el programa.
 */
void init_()
{
  bajarBarrera();                   // Se baja la barrera.
  apagarTiraLed();                  // Se apaga la tira de LEDs.
  apagarIndicadorDeEntradaSalida(); // Se apaga el LED indicador de entrada|salida.

  current_state = ST_ESPERA; // Set del estado en espera.
}

/*
 * Activo el modo espera.
 */
void modoEspera()
{
  ledRGBAmarilloTitilante(); // Activo el LED amarillo titilante.

  current_state = ST_ESPERA; // Set del estado en espera.
}

/*
 * Activo cambio de clave.
 */
void cambiandoClave()
{
  ledRGBAzul();                       // Activo el LED azul.
  strcpy(CLAVE_MAESTRA, NUEVA_CLAVE); // Guardo nueva clave.
  INDICE = 0;                         // Reset del indice para guardar la clave.
  ledRGBAmarilloTitilante();          // Activo el LED amarillo titilante.
  apagarIndicadorDeEntradaSalida();   // Desactivo indicador de ingreso|salida de vehículo.
  bajarBarrera();                     // Bajo la barrera.

  current_state = ST_ESPERA; // Set del estado en espera.
}

/*
 * Activo auto entrando.
 */
void autoEntrando()
{
  ledRGBVerde();                      // Activo el LED verde.
  encenderIndicadorDeEntradaSalida(); // Activo indicador de ingreso|salida de vehículo.
  subirBarrera();                     // Levanto la barrera.

  current_state = ST_AUTO_ENTRANDO; // Set del estado auto entrando.
}

/*
 * Activo vehículo no autorizado.
 */
void noAutorizado()
{
  lct_corto = millis(); // Reinicio timeout de 3s.

  ledRGBRojo(); // Activo el LED rojo.

  current_state = ST_NO_AUTORIZADO; // Set del estado no autorizado.
}

/*
 * Activo vehículo dentro.
 */
void autoAdentro()
{
  ledRGBAmarilloTitilante();        // Activo el LED amarillo titilante.
  apagarIndicadorDeEntradaSalida(); // Desactivo indicador de ingreso|salida de vehículo.
  bajarBarrera();                   // Bajo la barrera.

  current_state = ST_AUTO_ADENTRO; // Set del estado auto adentro.
}

/*
 * Activo vehículo saliendo.
 */
void autoSaliendo()
{
  lct_largo = millis(); // Reinicio timeout de 10s. Espera que el auto salga.

  subirBarrera();                     // Levanto la barrera.
  encenderIndicadorDeEntradaSalida(); // Activo indicador de ingreso|salida de vehículo.

  current_state = ST_AUTO_SALIENDO; // Set del estado auto saliendo.
}

/*
 * Activo vehículo fuera.
 */
void autoAfuera()
{
  apagarIndicadorDeEntradaSalida(); // Desactivo indicador de ingreso|salida de vehículo.
  bajarBarrera();                   // Bajo la barrera.

  current_state = ST_AUTO_AFUERA; // Set del estado auto fuera.
}

/*
 * Activo error.
 */
void error()
{
  ledRGBRojo(); // Activo el LED rojo.

  current_state = ST_ERROR; // Set del estado auto fuera.
}

/*
 * Activo reset del programa.
 */
void reset()
{
  ledRGBAmarilloTitilante();        // Activo el LED amarillo titilante.
  apagarIndicadorDeEntradaSalida(); // Desactivo indicador de ingreso|salida de vehículo.
  bajarBarrera();                   // Bajo la barrera.
  memset(CLAVE, MIN_INDICE, sizeof(CLAVE));  // reseteo clave ingresada
  INDICE = MIN_INDICE;                       // reseteo indice

  current_state = ST_ESPERA; // Set del estado en espera.
}

/*
 * Activo none. Función para realizar un skip de próxima acción.
 */
void none() {}

/*
 * Verifico eventos de clave correcta|incorrecta. Actualiza la clave en caso que el modo sea cambio de clave.
 */
bool verificarClave()
{
  TECLA = teclado.getKey(); // Set tecla presionada.

  if (TECLA) // Verfico tecla presionada.
  {                        
    DebugPrint(TECLA);     // Imprimo tecla presionada.
    CLAVE[INDICE] = TECLA; // Guarda tecla presionada.
    INDICE++;              // Incremento indice
  }

  if (INDICE != MAX_INDICE) // Verifico clave de 4 digitos.
  { 
    return false;
  }

  if (!strcmp(CLAVE, CLAVE_MAESTRA))
  {                                // Verifico clave ingresada.
    new_event = EV_CLAVE_CORRECTA; // Set evento clave correcta.
  }
  else
  {
    new_event = EV_CLAVE_INCORRECTA; // Set evento clave incorrecta.
  }

  INDICE = MIN_INDICE; // Reset del inficio para guardar la clave.

  return true;
}

/*
 * Verifico sensor de movimiento PIR.
 */
bool verificarSensorPIR()
{
  if (digitalRead(PIR_PIN) == HIGH)
  {                               // Verifico si el sensor detectó movimiento.
    new_event = EV_PIR_DETECTADO; // Set evento movimiento detectado.
    return true;
  }

  return false;
}

/*
 * Verifico fotosensor.
 */
bool verificarSensorLuz()
{
  if (
      (previa_lectura_luz - SENSOR_LUZ_UMBRAL > DIFERENCIA_SENSOR_LUZ_UMBRAL && analogRead(FOTOSENSOR_PIN) - SENSOR_LUZ_UMBRAL < DIFERENCIA_SENSOR_LUZ_UMBRAL) ||
      (previa_lectura_luz - SENSOR_LUZ_UMBRAL < DIFERENCIA_SENSOR_LUZ_UMBRAL && analogRead(FOTOSENSOR_PIN) - SENSOR_LUZ_UMBRAL > DIFERENCIA_SENSOR_LUZ_UMBRAL))
  {                                                  // Verifico si hubo un cambio de luz respecto al umbral.
    new_event = EV_CAMBIO_DE_LUZ;                    // Set evento cambio de luz.
    previa_lectura_luz = analogRead(FOTOSENSOR_PIN); // Lectura del fotosensor.

    return true;
  }

  return false;
}

/*
 * Verifico entrada serial.
 */
bool verificarSerial()
{
  if (Serial.available() > VERIFICACION_ENTRADA_SERIAL) // Verifico si hay datos en la entrada serial.
  {
    String data = Serial.readStringUntil('\n'); // Leo datos del serial.
    if (data.indexOf("cambiarClave") != VERIFICACION_ACCION_TECLADO)     // Verifico si es cambio de clave.
    {
      String parametro = data.substring(INICIO_NUEVA_CLAVE, FIN_NUEVA_CLAVE); // Set nueva clave.
      strcpy(NUEVA_CLAVE, parametro.c_str());    // Guarda nueva clave.
      new_event = EV_CAMBIO_CLAVE;               // Set evento cambio de clave.
      return true;
    }
    if (data.indexOf("reset") != VERIFICACION_ACCION_TECLADO) // Verifico si reset
    {
      new_event = EV_RESET; // Set evento reset.
      return true;
    }
  }
  return false;
}

/*
 * Manejador de eventos.
 */
void get_new_event()
{
  long ct = millis();                                             // Inicializo contador de current time.
  int pollingTeclado = (ct - lct_teclado);                        // Diferencia de timeout largo.
  timeout_teclado = (pollingTeclado > UMBRAL_DIFERENCIA_TECLADO); // Lectura de teclado.

  if (timeout_teclado) // polling rapido de teclado para evitar que no se presione la tecla.
  {
    timeout = false; // Reseteo timeout general.
    lct_teclado = ct;
    if (verificarClave() == true) // verificar clave de teclado
    {
      return;
    }
  }

  int diferencia = (ct - lct);                                         // Diferencia de timeout genérico.
  int diferenciaCorta = (ct - lct_corto);                              // Diferencia de timeout corto.
  int diferenciaLarga = (ct - lct_largo);                              // Diferencia de timeout largo.
  timeout = (diferencia > UMBRAL_DIFERENCIA_TIMEOUT);                  // Activar de timeout genérico.
  timeout_corto = (diferenciaCorta > UMBRAL_DIFERENCIA_TIMEOUT_CORTO); // Activar de timeout corto.
  timeout_largo = (diferenciaLarga > UMBRAL_DIFERENCIA_TIMEOUT_LARGO); // Activar de timeout largo.

  if (timeout_corto) // Verifico timout corto.
  {
    timeout_corto = false; // Reseteo timeout corto.
    lct_corto = ct;
    new_event = EV_TIMEOUT_CORTO; // Set evento timeout corto.
    return;
  }

  if (timeout_largo) // Verifico timout largo.
  {
    timeout_largo = false; // Reseteo timeout largo.
    lct_largo = ct;
    new_event = EV_TIMEOUT_LARGO; // Set evento timeout largo.
    return;
  }

  if (timeout)
  {                  // Verifico timout general.
    timeout = false; // Reseteo timeout general.
    lct = ct;

    if (verificarSensorPIR() == true || verificarSensorLuz() == true || verificarSerial() == true)
    { // Verifico eventos de chequeo.
      return;
    }
  }

  new_event = EV_CONTINUE; // Set evento continue. Dummy event.
}

/*
 * Activo máquina de estados.
 */
void maquinaEstadosEstacionamiento()
{
  get_new_event(); // Set del evento entrante.

  if ((new_event >= MIN_EVENT) && (new_event < MAX_EVENTS) && (current_state >= MIN_STATES) && (current_state < MAX_STATES)) // Verifico evento y estados.
  {
    if (new_event != EV_CONTINUE)
    {                                                                 // Verifico evento distinto de continue.
      DebugPrintEstado(states_s[current_state], events_s[new_event]); // Muestro estado y evento.
    }

    state_table[current_state][new_event](); // Acciono evento corespondiente al estado actual.
  }

  new_event = EV_CONTINUE; // Set evento continue. Dummy event.
}

/*
 * Setup del Arduino.
 */
void setup()
{
  do_init(); // Inicializó programa.
}

/*
 * Loop del Arduino.
 */
void loop()
{
  maquinaEstadosEstacionamiento(); // Activo máquina de estados.
}
