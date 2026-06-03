/*
  Controlador de temperatura por angulo de fase para Arduino UNO / ATmega328P.

  Arquitectura general
  --------------------
  1. El sistema mide una termocupla tipo K con un MAX6675, lee un potenciometro
     como setpoint, ejecuta un controlador PI/PID discreto cada 250 ms y muestra
     estado en un LCD 16x2 I2C. Todas estas tareas viven en el loop cooperativo.

  2. El cruce por cero de red entra por INT0 (D2 / PD2). No se usa
     attachInterrupt(); se configura EICRA/EIMSK directamente y el semiciclo se
     sincroniza en ISR(INT0_vect).

  3. En cada cruce por cero, Timer1 se reinicia a cero y se programan OCR1A y
     OCR1B. OCR1A representa el retardo desde el cruce por cero hasta el disparo
     del TRIAC. Ese retardo se calcula desde el angulo:

       delay_us = (angle_deg / 180.0) * half_cycle_us

     Para una carga resistiva se incluye ademas una linealizacion opcional de
     potencia RMS: P/Pmax = ((pi - alpha) + sin(2 alpha)/2) / pi.

  4. OCR1A inicia el pulso al LED del optotriac MOC3053. OCR1B lo corta 100 us
     despues. Con F_CPU = 16 MHz y prescaler 8, Timer1 avanza a 0.5 us/tick, por
     lo que 100 us equivalen a 200 ticks.

  5. El LCD, Wire/I2C, la libreria MAX6675, la lectura ADC y el calculo PID no
     participan en la temporizacion de disparo. Pueden bloquear el loop, pero no
     detienen Timer1 ni INT0. Las ISR solo hacen operaciones cortas sobre
     registros y variables volatile.

  6. Partes criticas en tiempo real: INT0, TIMER1_COMPA y TIMER1_COMPB, mas el
     acceso directo a PORTB para el gate del MOC3053. Partes no criticas: LCD,
     MAX6675, ADC, PID, debounce de llave y formateo de texto.

  7. LiquidCrystal_I2C y MAX6675 pueden ser bloqueantes porque corren en loop.
     Aun asi, en AVR ninguna ISR puede interrumpir a otra ISR ya en ejecucion;
     por eso se debe verificar que esas librerias no deshabiliten interrupciones
     durante tiempos largos. El codigo evita llamarlas desde ISR y actualiza el
     LCD con baja frecuencia para reducir jitter residual.

  8. BRINGUP_IO_TEST_MODE permite probar entradas y salidas antes de conectar
     potencia. En ese modo el LCD muestra temperatura, setpoint del pote,
     angulo de prueba, estado ON/OFF y contador de cruces por cero. La llave
     habilita los pulsos en D8 y el pote mueve el retardo de disparo, aun si el
     sensor aparece como T:ERR. Para el control termico final, ponerlo en 0.

  Librerias usadas
  ----------------
  - LiquidCrystal_I2C para LCD 16x2.
  - Adafruit MAX6675 library: constructor MAX6675(SCLK, CS, MISO), lectura con
    readCelsius(). El orden de parametros usado es SCK, CS, SO/MISO.

  Notas y advertencias tecnicas
  -----------------------------
  - Trabajar con tension de red es peligroso y potencialmente letal. Probar con
    aislamiento, protecciones y supervison adecuada.
  - El detector de cruce por cero debe estar aislado y se debe verificar con
    osciloscopio su polaridad, ancho de pulso, jitter y nivel logico.
  - Usar fusible, puesta a tierra, caja aislante, disipador para el BTA16,
    proteccion termica independiente y cableado apto para la corriente real.
  - Para 1800 W en 220 VAC la corriente RMS ronda 8.2 A; en 120 VAC ronda 15 A.
    Confirmar margen termico del TRIAC, corriente RMS, corriente de gate,
    disipacion y ventilacion.
  - En cargas reales conviene evaluar snubber RC y protecciones contra dv/dt,
    EMI y transitorios. Aunque la plancha sea resistiva, el cableado y la red no
    son ideales.
  - Probar primero con lampara incandescente o carga resistiva menor antes de
    conectar la plancha.
  - El MOC3053 es de disparo aleatorio/no-zero-cross y sirve para control por
    fase. No usar optotriacs con cruce por cero integrado para este esquema.
  - Confirmar la corriente de disparo del LED del MOC3053 y dimensionar la
    resistencia serie desde el pin del AVR para no exceder el pin ni quedar por
    debajo de la IFT necesaria.
  - Confirmar que LiquidCrystal_I2C, Wire y MAX6675 no deshabiliten
    interrupciones durante ventanas prolongadas. Si se requiere jitter de pocos
    microsegundos garantizado, medirlo con osciloscopio y considerar salida por
    comparador hardware OC1A/OC1B en un pin compatible.
  - Timer1 queda reservado para el control de fase. No usar Servo, tone ni PWM
    dependiente de Timer1 en D9/D10 mientras este firmware este activo.

  Plan recomendado de pruebas de banco, sin potencia conectada
  ------------------------------------------------------------
  1. Prueba visual y alimentacion:
     Alimentar solo el Arduino, LCD, MAX6675, potenciometro y llave. No conectar
     red electrica, TRIAC, plancha ni etapa de potencia. Verificar 5 V, GND
     comun, ausencia de calentamiento y polaridad del LCD/I2C.

  2. Prueba de LCD:
     Cargar el sketch con BRINGUP_IO_TEST_MODE = 1. El LCD debe mostrar
     temperatura actual o T:ERR, setpoint, angulo, contador Z y estado ON/OFF.
     Si no muestra nada, probar direccion LCD_I2C_ADDRESS 0x27 o 0x3F y ajustar
     contraste del backpack.

  3. Prueba de potenciometro:
     Girar el pote en A0. La linea superior debe cambiar S: objetivo. En modo
     prueba tambien debe cambiar A: angulo. Esto confirma ADC, referencia AVcc,
     divisor del pote y conversion de setpoint.

  4. Prueba de llave:
     Con el nuevo cableado, D4 a 0 V debe leerse OFF y D4 a 5 V debe leerse ON.
     El LCD debe cambiar ON/OFF sin bloquear temperatura ni setpoint. Al pasar
     a OFF, D8 debe ir inmediatamente a 0 y no deben salir nuevos pulsos.

  5. Prueba del MAX6675:
     Con termocupla conectada, el LCD debe mostrar T: valor en Celsius. Si
     aparece T:ERR, revisar SCK D13, CS D10, SO D12, alimentacion del modulo y
     polaridad de la termocupla. En modo prueba T:ERR no bloquea los pulsos para
     poder validar D2/D8 antes de resolver el sensor.

  6. Prueba de cruce por cero simulado por el propio Arduino:
     Dejar SIMULATED_ZERO_CROSS_OUTPUT_ENABLE = 1. El pin D7/PD7 genera una
     salida logica simulada con millis(). Conectar D7 a D2/INT0 usando una
     resistencia serie de 1 k a 4.7 k. El contador ZC del LCD debe incrementar
     y en D8 debe aparecer el pulso de disparo. Esta senal sirve para banco, no
     es una referencia precisa de frecuencia.

  7. Prueba de cruce por cero con generador externo:
     Si se usa generador de funciones u otro microcontrolador aislado de la red,
     aplicar a D2 una onda logica 0-5 V, con GND comun con Arduino y resistencia
     serie de 1 k a 4.7 k. Para MAINS_FREQ_HZ=50 usar 100 Hz; para 60 Hz usar
     120 Hz. No aplicar 12 V, 24 V, tension negativa ni red al pin D2.

  8. Prueba de salida de disparo:
     Con osciloscopio, CH1 en D2 y CH2 en D8, ambos referidos a GND Arduino.
     Usar trigger en flanco de D2. Con llave ON se debe ver en D8 un pulso
     positivo de aproximadamente 100 us por cruce aceptado. Al mover el pote,
     el retardo entre D2 y D8 debe cambiar. Con llave OFF no debe haber pulsos.

  9. Prueba del optotriac sin red:
     Solo despues de validar D8, conectar el LED de entrada del MOC3053 con su
     resistencia serie correcta y seguir midiendo del lado logico. Confirmar que
     el pin AVR no exceda corriente segura y que la resistencia asegure la IFT
     requerida por el MOC3053. Aun no conectar MT1/MT2 ni carga de potencia.

  10. Prueba de detector real de cruce, todavia sin TRIAC/carga:
     Con el detector de cruce aislado y alimentado correctamente, medir primero
     su salida con osciloscopio. Debe entregar niveles compatibles 0-5 V al pin
     D2. Luego conectar a D2 y verificar que Z incremente estable y que el pulso
     de D8 quede sincronizado. Si hay doble conteo o ruido, ajustar el detector,
     flanco o ZERO_CROSS_BLANK_US.

  11. Paso a potencia:
      Recien despues de que LCD, sensor, pote, llave, cruce y pulsos esten
      verificados, conectar etapa de potencia con carga pequena resistiva,
      fusible, aislamiento, disipador y osciloscopio. La plancha de 1800 W debe
      ser la ultima prueba, no la primera.
*/

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <max6675.h>
#include <math.h>
#include <stdio.h>
#include <util/atomic.h>

// ---------------------------------------------------------------------------
// Configuracion general
// ---------------------------------------------------------------------------

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define MAINS_FREQ_HZ                  50UL
#define TIMER1_PRESCALER               8UL
#define TRIAC_GATE_PULSE_US            100UL
#define HALF_CYCLE_GUARD_US            150UL
#define ZERO_CROSS_BLANK_US            1000UL

#define BRINGUP_IO_TEST_MODE           1       // 1=modo prueba: LCD/sensor/pote/llave/ZC/pulsos; 0=control PID final.
#define SIMULATED_ZERO_CROSS_OUTPUT_ENABLE 1   // 1=genera cruce simulado en D7 para puentear a D2; 0=deshabilitado.
#define SIMULATED_ZERO_CROSS_PULSE_MS  2UL     // Ancho del pulso alto simulado. Con millis(), 1..2 ms es practico.

#define APP_TICK_MS                    10UL
#define MS_TO_TICKS(ms)                (((ms) + APP_TICK_MS - 1UL) / APP_TICK_MS)

#define CONTROL_INTERVAL_MS            250UL
#define CONTROL_SAMPLE_TIME_SEC        0.250f
#define LCD_REFRESH_INTERVAL_MS        250UL
#define LCD_AUTO_PAGE_INTERVAL_MS      3000UL
#define LCD_NAVIGATION_USE_BUTTON      0       // 0=rotacion automatica, 1=avanza pagina al soltar USER_BUTTON.
#define LCD_SETPOINT_PREVIEW_HOLD_MS   1000UL
#define MAX6675_STARTUP_MS             500UL

#define SETPOINT_MIN_C                 0.0f
#define SETPOINT_MAX_C                 240.0f
#define POT_CHANGE_THRESHOLD_ADC       8U

#define PID_KP                         3.50f
#define PID_KI                         0.08f
#define PID_KD                         0.00f
#define PID_OUTPUT_MIN_PCT             0.0f
#define PID_OUTPUT_MAX_PCT             100.0f

#define FIRING_ANGLE_MIN_DEG           2.0f
#define FIRING_ANGLE_MAX_DEG           175.0f
#define USE_RMS_POWER_LINEARIZATION    1

#define LCD_I2C_ADDRESS                0x27
#define LCD_COLUMNS                    16
#define LCD_ROWS                       2
#define LCD_I2C_CLOCK_HZ               400000UL

// ---------------------------------------------------------------------------
// Pinout configurable
// ---------------------------------------------------------------------------

// Cruce por cero: D2 / PD2 / INT0.
#define ZERO_CROSS_DDR                 DDRD      // Registro de direccion del puerto D.
#define ZERO_CROSS_PORT                PORTD     // Registro de salida/pull-up del puerto D.
#define ZERO_CROSS_PINREG              PIND      // Registro de lectura instantanea del puerto D.
#define ZERO_CROSS_BIT                 PD2       // Bit fisico asociado a D2 / INT0.
#define ZERO_CROSS_USE_PULLUP          0
#define ZERO_CROSS_RISING_EDGE         1

// Salida simulada de cruce por cero: D7 / PD7. Puente externo D7 -> D2.
#define SIM_ZERO_CROSS_DDR             DDRD      // Registro de direccion del puerto D.
#define SIM_ZERO_CROSS_PORT            PORTD     // Registro que gobierna el nivel logico de D7.
#define SIM_ZERO_CROSS_BIT             PD7       // Bit fisico asociado a D7 / PD7.

// Salida de disparo: D8 / PB0.
#define TRIAC_GATE_DDR                 DDRB      // Registro de direccion del puerto B.
#define TRIAC_GATE_PORT                PORTB     // Registro que gobierna el nivel logico del gate.
#define TRIAC_GATE_BIT                 PB0       // Bit fisico asociado a D8 / PB0.

// Llave digital ON/OFF: D4 / PD4, activa con 5 V. La llave debe entregar
// 5 V en ON y 0 V en OFF; el pull-up interno queda apagado.
#define ENABLE_SWITCH_DDR              DDRD      // Direccion de la entrada de llave.
#define ENABLE_SWITCH_PORT             PORTD     // Pull-up de la llave: se deja apagado para permitir 0/5 V reales.
#define ENABLE_SWITCH_PINREG           PIND      // Lectura directa de la llave.
#define ENABLE_SWITCH_BIT              PD4       // Bit fisico asociado a D4 / PD4.
#define ENABLE_SWITCH_USE              0         // 0=ignora la llave y muestra DES; 1=usa D4 como habilitacion ON/OFF.
#define ENABLE_SWITCH_ACTIVE_LOW       0
#define ENABLE_SWITCH_DEBOUNCE_MS      100UL

// Boton de usuario para navegar pantallas: D3 / PD3, activo con 5 V y pull-down externo.
#define USER_BUTTON_DDR                DDRD      // Direccion de la entrada del boton.
#define USER_BUTTON_PORT               PORTD     // En AVR clasico no existe pull-down interno: PORT en 0 deja la entrada sin pull-up.
#define USER_BUTTON_PINREG             PIND      // Lectura directa del pin del boton.
#define USER_BUTTON_BIT                PD3       // Bit fisico asociado a D3 / PD3.
#define USER_BUTTON_DEBOUNCE_MS        100UL

// Potenciometro de setpoint: A0 / PC0 / ADC0.
#define POT_ADC_CHANNEL                0
#define POT_ADC_DDR                    DDRC      // Direccion del puerto analogico.
#define POT_ADC_PORT                   PORTC     // Pull-up digital del pin analogico.
#define POT_ADC_BIT                    PC0       // Bit fisico asociado a A0 / ADC0.

// MAX6675. Los numeros digitales son los que usa la libreria.
#define MAX6675_SCK_PIN                13
#define MAX6675_CS_PIN                 10
#define MAX6675_SO_PIN                 12

// Inicializacion directa para el pinout sugerido del MAX6675.
#define MAX6675_SCK_DDR                DDRB      // Direccion del reloj bit-bang del MAX6675.
#define MAX6675_SCK_PORT               PORTB     // Nivel inicial del reloj bit-bang.
#define MAX6675_SCK_BIT                PB5       // Bit fisico asociado a D13 / SCK.
#define MAX6675_CS_DDR                 DDRB      // Direccion de chip-select del MAX6675.
#define MAX6675_CS_PORT                PORTB     // Nivel de chip-select; alto es inactivo.
#define MAX6675_CS_BIT                 PB2       // Bit fisico asociado a D10 / CS.
#define MAX6675_SO_DDR                 DDRB      // Direccion de serial-out del MAX6675.
#define MAX6675_SO_PORT                PORTB     // Pull-up digital de SO, dejado apagado.
#define MAX6675_SO_BIT                 PB4       // Bit fisico asociado a D12 / MISO.

// ---------------------------------------------------------------------------
// Constantes derivadas de temporizacion
// ---------------------------------------------------------------------------

#if (MAINS_FREQ_HZ != 50UL) && (MAINS_FREQ_HZ != 60UL)
#error "MAINS_FREQ_HZ debe ser 50 o 60 para esta configuracion."
#endif

#define TIMER1_TICKS_PER_US            (F_CPU / TIMER1_PRESCALER / 1000000UL)
#define HALF_CYCLE_US                  (1000000UL / (2UL * MAINS_FREQ_HZ))
#define HALF_CYCLE_TICKS               ((F_CPU / TIMER1_PRESCALER) / (2UL * MAINS_FREQ_HZ))
#define TRIAC_GATE_PULSE_TICKS         (TRIAC_GATE_PULSE_US * TIMER1_TICKS_PER_US)
#define HALF_CYCLE_GUARD_TICKS         (HALF_CYCLE_GUARD_US * TIMER1_TICKS_PER_US)
#define MIN_FIRING_DELAY_TICKS         ((uint16_t)((FIRING_ANGLE_MIN_DEG / 180.0f) * (float)HALF_CYCLE_TICKS))
#define MAX_SAFE_DELAY_TICKS           (HALF_CYCLE_TICKS - TRIAC_GATE_PULSE_TICKS - HALF_CYCLE_GUARD_TICKS)
#define ZERO_CROSS_BLANK_TICKS         (ZERO_CROSS_BLANK_US * TIMER1_TICKS_PER_US)
#define SIMULATED_ZERO_CROSS_PERIOD_MS (1000UL / (2UL * MAINS_FREQ_HZ))

#if (TIMER1_TICKS_PER_US != 2UL)
#warning "Este sketch esta ajustado para F_CPU=16MHz y prescaler 8: 0.5 us/tick."
#endif

#if (HALF_CYCLE_TICKS >= 65535UL)
#error "El semiciclo no entra en Timer1 de 16 bits con este prescaler."
#endif

#if (MAX_SAFE_DELAY_TICKS <= TRIAC_GATE_PULSE_TICKS)
#error "No queda ventana temporal segura para el pulso de disparo."
#endif

#if (SIMULATED_ZERO_CROSS_PULSE_MS >= SIMULATED_ZERO_CROSS_PERIOD_MS)
#error "SIMULATED_ZERO_CROSS_PULSE_MS debe ser menor que SIMULATED_ZERO_CROSS_PERIOD_MS."
#endif

/*
  Guia rapida de asignaciones bitwise usadas en este firmware
  -----------------------------------------------------------

  En AVR, registros como DDRB, PORTB, EICRA, TIMSK1 o ADCSRA son bytes donde
  cada bit controla una funcion del hardware. Por eso se modifican con mascaras
  para tocar solo el bit necesario y conservar intactos los demas.

  - (1 << BIT):
    Desplaza el 1 hasta la posicion BIT. Si BIT=PB0, la mascara queda
    00000001. Si BIT=PB5, queda 00100000.

  - registro |= mascara:
    Hace OR y asigna. Sirve para poner en 1 los bits de la mascara sin cambiar
    los otros bits. Ejemplo: DDRB |= (1 << PB0) deja PB0 como salida y conserva
    el estado previo de PB1..PB7.

  - registro &= ~mascara:
    Invierte la mascara con ~ y luego hace AND. Sirve para poner en 0 los bits
    de la mascara sin cambiar los otros. Ejemplo: PORTB &= ~(1 << PB0) baja PB0
    y conserva PB1..PB7.

  - registro = mascara:
    Es asignacion directa. Reemplaza todo el registro. Se usa solo cuando se
    quiere definir un estado completo conocido, por ejemplo TCCR1A = 0.

  - if (registro & mascara):
    Hace AND para probar un bit. El resultado es distinto de cero si ese bit
    esta en 1. Ejemplo: PIND & (1 << PD4) lee el nivel real del pin PD4.

  - flags AVR como TIFR1 o EIFR:
    En estos registros, escribir 1 no prende la bandera: la limpia. Por eso
    TIFR1 = (1 << OCF1A) borra OCF1A. Es una regla especial de muchos flags AVR.
*/

// ---------------------------------------------------------------------------
// Prototipos
// ---------------------------------------------------------------------------

extern "C" void initVariant(void);
void setup(void);
void loop(void);

void initPins(void);
void initExternalInterrupt(void);
void initTimer1(void);
void initADC(void);
void initLCD(void);
void initMAX6675(void);

bool updateAppTimeBaseTask(void);
void updateSetpointPotTask(void);
float readSetpointPot(void);
float adcToSetpointC(uint16_t raw);
uint16_t readADCBlocking(uint8_t channel);
float readTemperatureMAX6675(void);
float computePID(float setpointC, float measuredC);
float powerToFiringAngle(float powerPct);
uint16_t angleToTimerTicks(float angleDeg);

void updateControlTask(void);
void updateLCDTask(void);
void readEnableSwitchTask(void);
void readUserButtonTask(void);
void updateSimulatedZeroCrossTask(void);
void setOutputEnabledAtomic(bool enable);
bool getOutputEnabledAtomic(void);
bool readEnableSwitchRawOn(void);
bool readUserButtonRawPressed(void);
bool consumeUserButtonReleaseEvent(void);

static inline void setTriacGateHigh(void);
static inline void setTriacGateLow(void);
static inline float clampFloat(float value, float low, float high);

// ---------------------------------------------------------------------------
// Objetos de libreria
// ---------------------------------------------------------------------------

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);
MAX6675 thermocouple(MAX6675_SCK_PIN, MAX6675_CS_PIN, MAX6675_SO_PIN);

// ---------------------------------------------------------------------------
// Variables compartidas con ISR
// ---------------------------------------------------------------------------

volatile uint16_t g_firingDelayTicks = MAX_SAFE_DELAY_TICKS;
volatile bool g_outputEnabled = false;
volatile bool g_zeroCrossSynchronized = false;
volatile bool g_semicycleArmed = false;
volatile bool g_semicycleFired = false;
volatile uint32_t g_zeroCrossCount = 0;
volatile uint32_t g_firedPulseCount = 0;
volatile uint32_t g_blockedPulseCount = 0;

// ---------------------------------------------------------------------------
// Estado de loop, no critico
// ---------------------------------------------------------------------------

float g_temperatureC = NAN;
float g_setpointC = SETPOINT_MIN_C;
float g_powerPct = 0.0f;
float g_angleDeg = FIRING_ANGLE_MAX_DEG;
float g_pidIntegralTerm = 0.0f;
float g_pidLastError = 0.0f;
bool g_pidHasLastError = false;
bool g_sensorFault = true;
bool g_enableSwitchRawOn = false;
bool g_enableSwitchStableOn = false;
bool g_enableSwitchLastRawOn = false;
uint32_t g_enableSwitchLastChangeTick = 0;
uint32_t g_appTickCount = 0;
uint32_t g_lastAppTickMs = 0;
uint32_t g_lastControlTick = 0;
uint32_t g_lastLCDRefreshTick = 0;
uint32_t g_lastLCDPageChangeTick = 0;
uint32_t g_max6675ReadyTick = MS_TO_TICKS(MAX6675_STARTUP_MS);
uint32_t g_simZeroCrossNextPulseMs = 0;
uint32_t g_simZeroCrossPulseEndMs = 0;
bool g_simZeroCrossPulseActive = false;
uint8_t g_lcdPage = 0;
uint8_t g_lcdPageBeforeSetpointPreview = 0;
bool g_lcdForceRefresh = true;
bool g_lcdSetpointPreviewWasActive = false;
bool g_userButtonStablePressed = false;
bool g_userButtonLastRawPressed = false;
bool g_userButtonPressedFlag = false;
bool g_userButtonReleaseEvent = false;
uint32_t g_userButtonLastChangeTick = 0;
uint16_t g_setpointRaw = 0;
uint16_t g_setpointPreviewReferenceRaw = 0;
uint32_t g_setpointPreviewLastMotionTick = 0;
bool g_setpointPreviewActive = false;

/*
  initVariant()

  Arduino AVR llama a initVariant() despues de inicializar el core y antes de
  setup(). Aqui se configura Timer1 con registros, para que el temporizador
  critico exista antes de cualquier tarea de aplicacion. No toca librerias, no
  usa funciones bloqueantes y no debe llamarse desde una ISR. Este punto de
  entrada evita depender de configuraciones tardias del loop para la base de
  tiempo del disparo.
*/
extern "C" void initVariant(void)
{
  initTimer1();
}

/*
  setup()

  Inicializa pines, interrupcion externa, ADC, LCD y estado de sensores. La
  salida del MOC3053 queda forzada en bajo hasta que la llave este habilitada y
  exista una lectura valida de temperatura. No contiene delay(); las esperas de
  estabilizacion del MAX6675 se resuelven con marcas de tiempo en loop. No se
  llama desde ISR.
*/
void setup(void)
{
  initPins();
  initADC();
  initLCD();
  initMAX6675();
  initExternalInterrupt();

  uint32_t now = millis();
  g_lastAppTickMs = now;
  g_appTickCount = 0;
  g_setpointRaw = readADCBlocking(POT_ADC_CHANNEL);
  g_setpointPreviewReferenceRaw = g_setpointRaw;
  g_setpointC = adcToSetpointC(g_setpointRaw);
#if ENABLE_SWITCH_USE
  g_enableSwitchRawOn = readEnableSwitchRawOn();
#else
  g_enableSwitchRawOn = true;
#endif
  g_enableSwitchLastRawOn = g_enableSwitchRawOn;
  g_enableSwitchStableOn = g_enableSwitchRawOn;
  g_enableSwitchLastChangeTick = g_appTickCount;
  g_userButtonLastRawPressed = readUserButtonRawPressed();
  g_userButtonStablePressed = g_userButtonLastRawPressed;
  g_userButtonPressedFlag = g_userButtonStablePressed;
  g_userButtonLastChangeTick = g_appTickCount;
  g_lastLCDRefreshTick = g_appTickCount;
  g_lastLCDPageChangeTick = g_appTickCount;
  g_setpointPreviewLastMotionTick = g_appTickCount;
  g_lcdForceRefresh = true;
  g_simZeroCrossNextPulseMs = now;
  g_simZeroCrossPulseEndMs = now;
  g_simZeroCrossPulseActive = false;
  setOutputEnabledAtomic(false);
}

/*
  loop()

  Ejecuta una arquitectura cooperativa con base de 10 ms para entradas, pote,
  control y LCD. Si una lectura del MAX6675 o una transaccion I2C tarda mas de
  lo esperado, el gate del TRIAC sigue dependiendo de INT0 y Timer1. No debe
  incluir delay() ni rutinas que deshabiliten interrupciones durante tiempos
  largos.
*/
void loop(void)
{
  updateSimulatedZeroCrossTask();

  if (!updateAppTimeBaseTask()) {
    return;
  }

  readEnableSwitchTask();
  readUserButtonTask();
  updateSetpointPotTask();
  updateControlTask();
  updateLCDTask();
}

/*
  initPins()

  Configura directamente DDR y PORT de los pines usados. PB0 queda como salida
  y en bajo para mantener apagado el LED del MOC3053. PD2 queda como entrada de
  cruce por cero, con pull-up opcional segun el detector externo. PD4 queda
  como entrada directa 0/5 V para la llave ON/OFF y PD3 como entrada para boton
  con pull-down externo. PC0 se deja como entrada analogica sin pull-up.
  Tambien se inicializan los pines sugeridos del MAX6675 para estados
  electricos seguros. No usa librerias bloqueantes y no debe llamarse desde ISR.
*/
void initPins(void)
{
  /*
    DDRB:
    - PB0 / D8: salida digital hacia el LED del MOC3053 por resistencia serie.
    - PB2 / D10: salida CS del MAX6675, queda en alto inactivo.
    - PB5 / D13: salida SCK del MAX6675, queda en bajo.
    - PB4 / D12: entrada SO/MISO del MAX6675.
  */
  TRIAC_GATE_DDR |= (uint8_t)(1 << TRIAC_GATE_BIT);          // |= OR con mascara: pone PB0 en 1 como salida sin alterar otros bits de DDRB.
  TRIAC_GATE_PORT &= (uint8_t)~(1 << TRIAC_GATE_BIT);        // &= con mascara invertida: pone PB0 en 0 y mantiene apagado el MOC3053.

  MAX6675_CS_DDR |= (uint8_t)(1 << MAX6675_CS_BIT);          // |= setea solo PB2 en DDRB: CS queda configurado como salida.
  MAX6675_CS_PORT |= (uint8_t)(1 << MAX6675_CS_BIT);         // |= setea PB2 en PORTB: CS alto deselecciona el MAX6675.
  MAX6675_SCK_DDR |= (uint8_t)(1 << MAX6675_SCK_BIT);        // |= setea solo PB5 en DDRB: SCK queda configurado como salida.
  MAX6675_SCK_PORT &= (uint8_t)~(1 << MAX6675_SCK_BIT);      // &= ~mascara limpia PB5 en PORTB: SCK inicia en bajo.
  MAX6675_SO_DDR &= (uint8_t)~(1 << MAX6675_SO_BIT);         // &= ~mascara limpia PB4 en DDRB: SO queda como entrada.
  MAX6675_SO_PORT &= (uint8_t)~(1 << MAX6675_SO_BIT);        // &= ~mascara limpia PB4 en PORTB: pull-up digital apagado.

  /*
    DDRD/PORTD:
    - PD2 / INT0: entrada de cruce por cero. El pull-up se habilita solo si el
      detector externo lo requiere.
    - PD4: entrada de llave ON/OFF directa 0/5 V. Con el cableado sugerido,
      nivel alto representa ON.
    - PD3: entrada de boton de usuario con pull-down externo. Nivel alto
      representa boton presionado.
  */
  ZERO_CROSS_DDR &= (uint8_t)~(1 << ZERO_CROSS_BIT);         // &= ~mascara limpia PD2 en DDRD: INT0 queda como entrada.
#if ZERO_CROSS_USE_PULLUP
  ZERO_CROSS_PORT |= (uint8_t)(1 << ZERO_CROSS_BIT);         // |= setea PD2 en PORTD: activa pull-up interno.
#else
  ZERO_CROSS_PORT &= (uint8_t)~(1 << ZERO_CROSS_BIT);        // &= ~mascara limpia PD2 en PORTD: desactiva pull-up.
#endif

#if SIMULATED_ZERO_CROSS_OUTPUT_ENABLE
  SIM_ZERO_CROSS_DDR |= (uint8_t)(1 << SIM_ZERO_CROSS_BIT);   // |= setea PD7 en DDRD: D7 queda como salida simulada.
  SIM_ZERO_CROSS_PORT &= (uint8_t)~(1 << SIM_ZERO_CROSS_BIT); // &= ~mascara limpia PD7: pulso simulado inicia en bajo.
#else
  SIM_ZERO_CROSS_DDR &= (uint8_t)~(1 << SIM_ZERO_CROSS_BIT);  // &= ~mascara limpia PD7: deja D7 libre como entrada.
  SIM_ZERO_CROSS_PORT &= (uint8_t)~(1 << SIM_ZERO_CROSS_BIT); // &= ~mascara limpia PD7: pull-up apagado si el modo esta deshabilitado.
#endif

  ENABLE_SWITCH_DDR &= (uint8_t)~(1 << ENABLE_SWITCH_BIT);   // &= ~mascara limpia PD4 en DDRD: llave como entrada.
  ENABLE_SWITCH_PORT &= (uint8_t)~(1 << ENABLE_SWITCH_BIT);  // &= ~mascara limpia PD4 en PORTD: pull-up apagado para recibir 0/5 V reales.

  USER_BUTTON_DDR &= (uint8_t)~(1 << USER_BUTTON_BIT);       // &= ~mascara limpia PD3 en DDRD: boton como entrada.
  USER_BUTTON_PORT &= (uint8_t)~(1 << USER_BUTTON_BIT);      // &= ~mascara limpia PD3 en PORTD: sin pull-up; requiere pull-down externo.

  /*
    DDRC/PORTC:
    - PC0 / ADC0 / A0: entrada analogica de potenciometro. El pull-up se deja
      apagado para no deformar el divisor resistivo.
  */
  POT_ADC_DDR &= (uint8_t)~(1 << POT_ADC_BIT);               // &= ~mascara limpia PC0 en DDRC: A0 queda como entrada.
  POT_ADC_PORT &= (uint8_t)~(1 << POT_ADC_BIT);              // &= ~mascara limpia PC0 en PORTC: sin pull-up sobre el potenciometro.
}

/*
  initExternalInterrupt()

  Configura INT0 por registros nativos del ATmega328P. EICRA selecciona flanco
  ascendente o descendente segun el detector de cruce por cero; EIMSK habilita
  INT0 y EIFR limpia una bandera pendiente antes de arrancar. Esta interrupcion
  es la unica fuente de sincronismo de fase. No usa attachInterrupt(), no usa
  librerias y no debe llamarse desde ISR.
*/
void initExternalInterrupt(void)
{
  uint8_t oldSREG = SREG;
  cli();

  EIMSK &= (uint8_t)~(1 << INT0);                            // &= ~mascara limpia INT0: deshabilita INT0 sin tocar otras interrupciones externas.

  /*
    EICRA:
    ISC01 ISC00 = 11: flanco ascendente en INT0.
    ISC01 ISC00 = 10: flanco descendente en INT0.
    Si el detector entrega pulsos invertidos, cambiar ZERO_CROSS_RISING_EDGE.
  */
  EICRA &= (uint8_t)~((1 << ISC01) | (1 << ISC00));          // Primero crea mascara ISC01|ISC00, luego ~ la invierte y &= limpia solo esos bits.
#if ZERO_CROSS_RISING_EDGE
  EICRA |= (uint8_t)((1 << ISC01) | (1 << ISC00));           // |= prende ISC01 e ISC00: INT0 por flanco ascendente.
#else
  EICRA |= (uint8_t)(1 << ISC01);                            // |= prende solo ISC01; ISC00 quedo en 0: flanco descendente.
#endif

  EIFR = (uint8_t)(1 << INTF0);                              // Asignacion directa al flag: en EIFR escribir 1 borra INTF0.
  EIMSK |= (uint8_t)(1 << INT0);                             // |= prende INT0: habilita la interrupcion despues de configurar EICRA.

  SREG = oldSREG;
}

/*
  initTimer1()

  Configura Timer1 como contador libre de 16 bits con prescaler 8. Se eligio
  contador normal en lugar de CTC con OCR1A como TOP porque OCR1A debe marcar el
  inicio del pulso y OCR1B debe ocurrir despues; si OCR1A fuera TOP, el timer se
  reiniciaria exactamente al disparar. El esquema sigue siendo de comparacion:
  TCNT1 se reinicia en cada cruce por cero, OCR1A inicia el gate y OCR1B lo
  apaga. No habilita interrupciones hasta que INT0 arme un semiciclo. No usa
  librerias y no debe llamarse desde ISR salvo reconfiguracion muy controlada.
*/
void initTimer1(void)
{
  uint8_t oldSREG = SREG;
  cli();

  /*
    TCCR1A:
    COM1A1:0 = 00 y COM1B1:0 = 00: los pines OC1A/OC1B no son controlados por
    hardware, porque el gate usa D8/PB0 por acceso directo a PORTB.
    WGM11:0 = 00: parte baja del modo normal.
  */
  TCCR1A = 0;                                                // Asignacion directa: pone todos los bits de TCCR1A en 0.

  /*
    TCCR1B:
    WGM13:2 = 00: modo normal, TCNT1 cuenta hasta 0xFFFF.
    CS12:0 = 010: prescaler 8. A 16 MHz equivale a 2 MHz, 0.5 us por tick.
  */
  TCCR1B = (uint8_t)(1 << CS11);                             // Asignacion directa: solo CS11 queda en 1, seleccionando prescaler 8.

  /*
    TCNT1/OCR1A/OCR1B:
    TCNT1 se reinicia en cada cruce por cero. OCR1A se carga con el retardo de
    disparo del semiciclo y OCR1B con retardo + 100 us.
  */
  TCNT1 = 0;                                                 // Escribe 0 en el registro contador completo de 16 bits.
  OCR1A = MAX_SAFE_DELAY_TICKS;                              // Asignacion de 16 bits: compare A recibe el retardo seguro inicial.
  OCR1B = MAX_SAFE_DELAY_TICKS + TRIAC_GATE_PULSE_TICKS;     // Asignacion de 16 bits: compare B queda 100 us despues de OCR1A.

  /*
    TIMSK1:
    Las interrupciones de comparacion quedan apagadas por defecto. INT0 las
    habilita solo para el semiciclo actual, evitando redisparos.
  */
  TIMSK1 &= (uint8_t)~((1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1)); // &= ~mascara apaga OCIE1A/OCIE1B/TOIE1 sin tocar otros bits.

  /*
    TIFR1:
    En AVR las banderas se limpian escribiendo 1. Se eliminan compare flags y
    overflow flags viejas antes de empezar.
  */
  TIFR1 = (uint8_t)((1 << OCF1A) | (1 << OCF1B) | (1 << TOV1));       // Flags AVR: asignar 1 a OCF1A/OCF1B/TOV1 los borra.

  SREG = oldSREG;
}

/*
  initADC()

  Configura el ADC del ATmega328P en modo single conversion, referencia AVcc y
  prescaler 128. A 16 MHz el reloj ADC queda en 125 kHz, apropiado para 10 bits.
  La lectura es bloqueante en loop, pero las interrupciones globales permanecen
  habilitadas durante la conversion, asi que no bloquea el disparo de fase. No
  debe llamarse desde ISR.
*/
void initADC(void)
{
  /*
    ADMUX:
    REFS0 = 1: referencia AVcc con capacitor externo en AREF.
    MUX3:0 = POT_ADC_CHANNEL: seleccion ADC0 por defecto.
  */
  ADMUX = (uint8_t)((1 << REFS0) | (POT_ADC_CHANNEL & 0x0F));          // (1<<REFS0) prende AVcc; channel&0x0F asegura solo MUX3:0.

  /*
    ADCSRA:
    ADEN = 1: habilita ADC.
    ADPS2:0 = 111: prescaler 128.
    ADIE = 0: no se usa interrupcion ADC; el muestreo no es critico.
  */
  ADCSRA = (uint8_t)((1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)); // OR combina bits: ADEN=1 y ADPS2:0=111.

  /*
    DIDR0:
    Deshabilita el buffer digital de ADC0 para reducir consumo y ruido.
  */
  DIDR0 |= (uint8_t)(1 << ADC0D);                                    // |= prende ADC0D: deshabilita el buffer digital de ADC0.
}

/*
  initLCD()

  Inicializa Wire y LiquidCrystal_I2C para un LCD 16x2. La direccion queda en
  LCD_I2C_ADDRESS, tipicamente 0x27 pero configurable a 0x3F si el backpack lo
  requiere. Esta funcion usa librerias bloqueantes y solo puede llamarse desde
  setup/loop, nunca desde ISR. Su efecto fisico es activar el bus I2C, preparar
  el expansor y encender la retroiluminacion.
*/
void initLCD(void)
{
  Wire.begin();
  Wire.setClock(LCD_I2C_CLOCK_HZ);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Temp controller"));
  lcd.setCursor(0, 1);
  lcd.print(F("Sync by INT0"));
}

/*
  initMAX6675()

  Prepara el estado logico de lectura del MAX6675. La libreria Adafruit no
  requiere begin(); el objeto global ya conoce SCK, CS y SO. Se deja una ventana
  inicial sin delay() antes de aceptar lecturas, porque el conversor necesita
  tiempo para estabilizar la primera muestra. Puede ser bloqueante si se decide
  leer el sensor, por eso no debe llamarse desde ISR.
*/
void initMAX6675(void)
{
  g_max6675ReadyTick = g_appTickCount + MS_TO_TICKS(MAX6675_STARTUP_MS);
}

/*
  updateAppTimeBaseTask()

  Genera la base cooperativa de 10 ms usando millis(). Las tareas de bajo ritmo
  cuentan ticks desde aqui; solo INT0 y Timer1 quedan para tiempos de precision.
  Si una tarea bloqueante demora un poco, el contador avanza los ticks
  transcurridos sin intentar ejecutar tareas atrasadas una por una.
*/
bool updateAppTimeBaseTask(void)
{
  uint32_t now = millis();
  uint32_t elapsedMs = now - g_lastAppTickMs;

  if (elapsedMs < APP_TICK_MS) {
    return false;
  }

  uint32_t elapsedTicks = elapsedMs / APP_TICK_MS;
  g_lastAppTickMs += elapsedTicks * APP_TICK_MS;
  g_appTickCount += elapsedTicks;
  return true;
}

/*
  updateSetpointPotTask()

  Lee el potenciometro en la base de 10 ms, actualiza el setpoint 0..240 C y
  detecta movimiento suficiente para abrir la pantalla temporal con barra.
*/
void updateSetpointPotTask(void)
{
  uint16_t raw = readADCBlocking(POT_ADC_CHANNEL);
  uint16_t delta = (raw > g_setpointPreviewReferenceRaw)
      ? (raw - g_setpointPreviewReferenceRaw)
      : (g_setpointPreviewReferenceRaw - raw);

  g_setpointRaw = raw;
  g_setpointC = adcToSetpointC(raw);

  if (delta < POT_CHANGE_THRESHOLD_ADC) {
    return;
  }

  g_setpointPreviewReferenceRaw = raw;
  g_setpointPreviewLastMotionTick = g_appTickCount;

  if (!g_setpointPreviewActive) {
    g_lcdPageBeforeSetpointPreview = g_lcdPage;
    g_setpointPreviewActive = true;
  }

  g_lcdForceRefresh = true;
}

/*
  readSetpointPot()

  Lee ADC0 en forma bloqueante y convierte el valor 0..1023 a un setpoint entre
  SETPOINT_MIN_C y SETPOINT_MAX_C. El ADC no es parte de la temporizacion
  critica; si la conversion demora, Timer1 e INT0 siguen interrumpiendo. No usa
  librerias externas y no debe llamarse desde ISR porque espera el fin de
  conversion.
*/
float readSetpointPot(void)
{
  uint16_t raw = readADCBlocking(POT_ADC_CHANNEL);
  return adcToSetpointC(raw);
}

/*
  adcToSetpointC()

  Convierte una lectura ADC 0..1023 al rango configurable del setpoint.
*/
float adcToSetpointC(uint16_t raw)
{
  float span = SETPOINT_MAX_C - SETPOINT_MIN_C;
  return SETPOINT_MIN_C + ((float)raw * span / 1023.0f);
}

/*
  readADCBlocking()

  Ejecuta una conversion ADC de un canal especifico con acceso directo a ADMUX
  y ADCSRA. La espera es activa pero no deshabilita interrupciones, de modo que
  las ISR de cruce por cero y Timer1 conservan prioridad temporal. No debe
  llamarse desde ISR porque bloquearia otras interrupciones.
*/
uint16_t readADCBlocking(uint8_t channel)
{
  ADMUX = (uint8_t)((ADMUX & 0xF0) | (channel & 0x0F));              // ADMUX&0xF0 conserva REFS/ADLAR; OR agrega el canal en MUX3:0.
  ADCSRA |= (uint8_t)(1 << ADSC);                                    // |= prende ADSC sin cambiar ADEN/prescaler: inicia conversion.

  while (ADCSRA & (1 << ADSC)) {                                     // AND prueba ADSC: distinto de 0 significa conversion en curso.
    // Espera corta. Las interrupciones globales siguen habilitadas.
  }

  return ADC;
}

/*
  readTemperatureMAX6675()

  Lee la temperatura en Celsius usando la libreria Adafruit MAX6675 mediante
  thermocouple.readCelsius(). La libreria bit-bang usa SCK, CS y SO configurados
  en el constructor MAX6675(SCK, CS, SO). La lectura puede bloquear brevemente y
  puede devolver NAN si la termocupla esta desconectada; por seguridad se trata
  como falla de sensor y se corta la potencia. Nunca debe llamarse desde ISR.
*/
float readTemperatureMAX6675(void)
{
  if ((int32_t)(g_appTickCount - g_max6675ReadyTick) < 0) {
    return NAN;
  }

  return thermocouple.readCelsius();
}

/*
  computePID()

  Calcula un controlador discreto PI/PID en unidades de porcentaje de potencia.
  Las ganancias Kp, Ki y Kd son configurables. La integral esta limitada y usa
  anti-windup condicional: si la salida esta saturada, solo integra cuando el
  error ayuda a salir de la saturacion. Es una tarea lenta, apropiada para una
  carga termica de alta inercia, y no tiene efectos directos sobre hardware. No
  debe llamarse desde ISR.
*/
float computePID(float setpointC, float measuredC)
{
  float error = setpointC - measuredC;
  float proportional = PID_KP * error;
  float derivative = 0.0f;

  if (g_pidHasLastError) {
    derivative = PID_KD * (error - g_pidLastError) / CONTROL_SAMPLE_TIME_SEC;
  } else {
    g_pidHasLastError = true;
  }

  float candidateIntegral = g_pidIntegralTerm + (PID_KI * error * CONTROL_SAMPLE_TIME_SEC);
  candidateIntegral = clampFloat(candidateIntegral, PID_OUTPUT_MIN_PCT, PID_OUTPUT_MAX_PCT);

  float unsaturated = proportional + candidateIntegral + derivative;
  float saturated = clampFloat(unsaturated, PID_OUTPUT_MIN_PCT, PID_OUTPUT_MAX_PCT);

  bool outputIsSaturated = (unsaturated != saturated);
  bool saturationWouldReduce =
      ((saturated >= PID_OUTPUT_MAX_PCT) && (error < 0.0f)) ||
      ((saturated <= PID_OUTPUT_MIN_PCT) && (error > 0.0f));

  if (!outputIsSaturated || saturationWouldReduce) {
    g_pidIntegralTerm = candidateIntegral;
  }

  g_pidIntegralTerm = clampFloat(g_pidIntegralTerm, PID_OUTPUT_MIN_PCT, PID_OUTPUT_MAX_PCT);
  g_pidLastError = error;

  return clampFloat(proportional + g_pidIntegralTerm + derivative,
                    PID_OUTPUT_MIN_PCT,
                    PID_OUTPUT_MAX_PCT);
}

/*
  powerToFiringAngle()

  Convierte potencia requerida 0..100% a angulo de disparo. Mayor potencia
  produce angulo mas bajo y disparo mas temprano. Por defecto usa la relacion
  RMS real de una carga resistiva y una busqueda binaria corta; si se desactiva
  USE_RMS_POWER_LINEARIZATION, usa mapeo lineal simple. No toca hardware, no es
  critico en tiempo real y no debe llamarse desde ISR.
*/
float powerToFiringAngle(float powerPct)
{
  powerPct = clampFloat(powerPct, 0.0f, 100.0f);

  if (powerPct <= 0.0f) {
    return FIRING_ANGLE_MAX_DEG;
  }

  if (powerPct >= 100.0f) {
    return FIRING_ANGLE_MIN_DEG;
  }

#if USE_RMS_POWER_LINEARIZATION
  float target = powerPct / 100.0f;
  float low = 0.0f;
  float high = PI;

  for (uint8_t i = 0; i < 18; ++i) {
    float mid = 0.5f * (low + high);
    float normalizedPower = ((PI - mid) + (0.5f * sinf(2.0f * mid))) / PI;

    if (normalizedPower > target) {
      low = mid;
    } else {
      high = mid;
    }
  }

  float angleDeg = high * 180.0f / PI;
  return clampFloat(angleDeg, FIRING_ANGLE_MIN_DEG, FIRING_ANGLE_MAX_DEG);
#else
  float span = FIRING_ANGLE_MAX_DEG - FIRING_ANGLE_MIN_DEG;
  return FIRING_ANGLE_MAX_DEG - ((powerPct / 100.0f) * span);
#endif
}

/*
  angleToTimerTicks()

  Convierte el angulo de disparo a ticks de Timer1 desde el cruce por cero.
  Aplica limites de seguridad para que OCR1A + pulso de 100 us + margen no
  excedan el semiciclo. El resultado se escribe luego de forma atomica en la
  variable volatile que usa INT0. No toca hardware directamente y no debe
  llamarse desde ISR.
*/
uint16_t angleToTimerTicks(float angleDeg)
{
  angleDeg = clampFloat(angleDeg, FIRING_ANGLE_MIN_DEG, FIRING_ANGLE_MAX_DEG);

  uint32_t ticks = (uint32_t)((angleDeg * (float)HALF_CYCLE_TICKS / 180.0f) + 0.5f);

  if (ticks < MIN_FIRING_DELAY_TICKS) {
    ticks = MIN_FIRING_DELAY_TICKS;
  }

  if (ticks > MAX_SAFE_DELAY_TICKS) {
    ticks = MAX_SAFE_DELAY_TICKS;
  }

  return (uint16_t)ticks;
}

/*
  updateControlTask()

  Se ejecuta cada CONTROL_INTERVAL_MS usando la base de 10 ms. Toma el setpoint
  ya muestreado del pote, lee temperatura, valida falla de termocupla, calcula
  el PID, transforma potencia a angulo y publica el retardo de disparo en ticks
  para las ISR. La publicacion de la variable de 16 bits se hace con
  ATOMIC_BLOCK. En modo final, si el sensor falla, fuerza potencia cero y
  deshabilita inmediatamente la salida del TRIAC. En
  BRINGUP_IO_TEST_MODE, el pote controla una potencia/angulo de prueba y la
  llave permite emitir pulsos aunque la termocupla no este lista, para validar
  D2/D8 con osciloscopio sin conectar potencia. No debe llamarse desde ISR.
*/
void updateControlTask(void)
{
  if ((uint32_t)(g_appTickCount - g_lastControlTick) < MS_TO_TICKS(CONTROL_INTERVAL_MS)) {
    return;
  }
  g_lastControlTick = g_appTickCount;

  float newTemperatureC = readTemperatureMAX6675();

  if (isnan(newTemperatureC)) {
    g_sensorFault = true;
    g_temperatureC = NAN;

#if BRINGUP_IO_TEST_MODE
    g_pidIntegralTerm = 0.0f;
    g_pidHasLastError = false;

    float normalizedPot = (g_setpointC - SETPOINT_MIN_C) / (SETPOINT_MAX_C - SETPOINT_MIN_C);
    g_powerPct = clampFloat(normalizedPot * 100.0f, 0.0f, 100.0f);
    g_angleDeg = powerToFiringAngle(g_powerPct);

    uint16_t testDelayTicks = angleToTimerTicks(g_angleDeg);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      g_firingDelayTicks = testDelayTicks;
    }

    setOutputEnabledAtomic(g_enableSwitchStableOn);
    return;
#else
    g_powerPct = 0.0f;
    g_angleDeg = FIRING_ANGLE_MAX_DEG;
    g_pidIntegralTerm = 0.0f;
    g_pidHasLastError = false;

    uint16_t safeDelay = angleToTimerTicks(g_angleDeg);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      g_firingDelayTicks = safeDelay;
    }
    setOutputEnabledAtomic(false);
    return;
#endif
  }

  g_sensorFault = false;
  g_temperatureC = newTemperatureC;

#if BRINGUP_IO_TEST_MODE
  g_pidIntegralTerm = 0.0f;
  g_pidHasLastError = false;

  float normalizedPot = (g_setpointC - SETPOINT_MIN_C) / (SETPOINT_MAX_C - SETPOINT_MIN_C);
  g_powerPct = clampFloat(normalizedPot * 100.0f, 0.0f, 100.0f);
  g_angleDeg = powerToFiringAngle(g_powerPct);
#else
  g_powerPct = computePID(g_setpointC, g_temperatureC);
  g_angleDeg = powerToFiringAngle(g_powerPct);
#endif

  uint16_t nextDelayTicks = angleToTimerTicks(g_angleDeg);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_firingDelayTicks = nextDelayTicks;
  }

  setOutputEnabledAtomic(g_enableSwitchStableOn);
}

/*
  updateLCDTask()

  Actualiza el LCD en forma no bloqueante y separa el refresco visual del cambio
  de pagina. En modo automatico, cada pagina permanece LCD_AUTO_PAGE_INTERVAL_MS
  antes de pasar a la siguiente. En modo boton, la pagina avanza solo cuando se
  consume un evento de release debounced del USER_BUTTON. Si el pote se mueve,
  pausa la rotacion y muestra TEMP OBJ con una barra de 16 columnas hasta que el
  pote quede quieto durante LCD_SETPOINT_PREVIEW_HOLD_MS. La esquina superior
  derecha muestra ON/OFF segun la lectura actual 0/5 V de D4. Usa
  LiquidCrystal_I2C, por lo tanto es bloqueante y nunca debe ejecutarse dentro
  de una ISR.
*/
void updateLCDTask(void)
{
  const uint8_t lcdPageCount =
#if SIMULATED_ZERO_CROSS_OUTPUT_ENABLE
      3;
#else
      2;
#endif

  if (g_lcdPage >= lcdPageCount) {
    g_lcdPage = 0;
  }

  if (g_setpointPreviewActive &&
      (uint32_t)(g_appTickCount - g_setpointPreviewLastMotionTick) >= MS_TO_TICKS(LCD_SETPOINT_PREVIEW_HOLD_MS)) {
    g_setpointPreviewActive = false;
    g_lcdPage = g_lcdPageBeforeSetpointPreview;
    if (g_lcdPage >= lcdPageCount) {
      g_lcdPage = 0;
    }
    g_lastLCDPageChangeTick = g_appTickCount;
    g_lcdForceRefresh = true;
  }

#if LCD_NAVIGATION_USE_BUTTON
  if (g_setpointPreviewActive) {
    (void)consumeUserButtonReleaseEvent();
  }
#endif

  if (!g_setpointPreviewActive) {
#if LCD_NAVIGATION_USE_BUTTON
    if (consumeUserButtonReleaseEvent()) {
      g_lcdPage++;
      if (g_lcdPage >= lcdPageCount) {
        g_lcdPage = 0;
      }
      g_lcdForceRefresh = true;
    }
#else
    if ((uint32_t)(g_appTickCount - g_lastLCDPageChangeTick) >= MS_TO_TICKS(LCD_AUTO_PAGE_INTERVAL_MS)) {
      g_lastLCDPageChangeTick = g_appTickCount;
      g_lcdPage++;
      if (g_lcdPage >= lcdPageCount) {
        g_lcdPage = 0;
      }
      g_lcdForceRefresh = true;
    }
#endif
  }

  if (!g_lcdForceRefresh &&
      (uint32_t)(g_appTickCount - g_lastLCDRefreshTick) < MS_TO_TICKS(LCD_REFRESH_INTERVAL_MS)) {
    return;
  }

  g_lastLCDRefreshTick = g_appTickCount;
  g_lcdForceRefresh = false;

  char topAttribute[14];
  char bottomAttribute[17];
  char topLine[17];
  char bottomLine[17];
  bool switchEnabled = g_enableSwitchStableOn;
  const char *stateText =
#if ENABLE_SWITCH_USE
      switchEnabled ? "ON " : "OFF";
#else
      "DES";
#endif
  bool showStateText = !g_setpointPreviewActive;

  if (g_setpointPreviewActive && !g_lcdSetpointPreviewWasActive) {
    lcd.clear();
  }
  g_lcdSetpointPreviewWasActive = g_setpointPreviewActive;

  int tempActualInt = 0;
  if (!g_sensorFault && !isnan(g_temperatureC)) {
    tempActualInt = (int)(g_temperatureC + (g_temperatureC >= 0.0f ? 0.5f : -0.5f));
  }

  int tempObjetivoInt = (int)(g_setpointC + 0.5f);
  int angleDisplay = (int)(g_angleDeg + 0.5f);
  uint32_t zeroCrossCountSnapshot = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    zeroCrossCountSnapshot = g_zeroCrossCount;
  }
  uint16_t zeroCrossDisplay = (uint16_t)(zeroCrossCountSnapshot % 10000UL);

  if (g_setpointPreviewActive) {
    float barStepC = (SETPOINT_MAX_C - SETPOINT_MIN_C) / (float)LCD_COLUMNS;
    uint8_t filledColumns = 0;

    if (barStepC > 0.0f && g_setpointC > SETPOINT_MIN_C) {
      filledColumns = (uint8_t)((g_setpointC - SETPOINT_MIN_C + barStepC - 0.001f) / barStepC);
    }

    if (filledColumns > LCD_COLUMNS) {
      filledColumns = LCD_COLUMNS;
    }

    snprintf(topAttribute, sizeof(topAttribute), "TEMP OBJ:%4d", tempObjetivoInt);

    for (uint8_t column = 0; column < LCD_COLUMNS; column++) {
      bottomLine[column] = (column < filledColumns) ? (char)255 : ' ';
    }
    bottomLine[LCD_COLUMNS] = '\0';
  } else {
    switch (g_lcdPage) {
      case 0:
        if (g_sensorFault || isnan(g_temperatureC)) {
          snprintf(topAttribute, sizeof(topAttribute), "TEMP ACT: ERR");
        } else {
          snprintf(topAttribute, sizeof(topAttribute), "TEMP ACT:%4d", tempActualInt);
        }
        snprintf(bottomAttribute, sizeof(bottomAttribute), "TEMP OBJ:%4d", tempObjetivoInt);
        break;

      case 1:
        snprintf(topAttribute, sizeof(topAttribute), "ANG DISP:%4d", angleDisplay);
        snprintf(bottomAttribute, sizeof(bottomAttribute), "ZC CNT:%5u", zeroCrossDisplay);
        break;

      default:
#if SIMULATED_ZERO_CROSS_OUTPUT_ENABLE
        snprintf(topAttribute, sizeof(topAttribute), "SIM ZC: ON");
        snprintf(bottomAttribute, sizeof(bottomAttribute), "PER:%2lums W:%lums",
                 (unsigned long)SIMULATED_ZERO_CROSS_PERIOD_MS,
                 (unsigned long)SIMULATED_ZERO_CROSS_PULSE_MS);
#else
        snprintf(topAttribute, sizeof(topAttribute), "SIM ZC:OFF");
        snprintf(bottomAttribute, sizeof(bottomAttribute), "PUENTE:NO");
#endif
        break;
    }

    snprintf(bottomLine, sizeof(bottomLine), "%-16.16s", bottomAttribute);
  }

  if (showStateText) {
    snprintf(topLine, sizeof(topLine), "%-13.13s%s", topAttribute, stateText);
  } else {
    snprintf(topLine, sizeof(topLine), "%-16.16s", topAttribute);
  }

  lcd.setCursor(0, 0);
  lcd.print(topLine);
  lcd.setCursor(0, 1);
  lcd.print(bottomLine);
}

/*
  readEnableSwitchTask()

  Lee la llave ON/OFF con debounce por software no bloqueante cuando
  ENABLE_SWITCH_USE esta activo. Si ENABLE_SWITCH_USE vale 0, fuerza el permiso
  de salida a habilitado y el LCD muestra DES. Con la llave activa, 5 V
  significa ON y 0 V significa OFF; al pasar a OFF corta inmediatamente el gate
  del MOC3053 y deshabilita las comparaciones de Timer1 de forma atomica.
*/
void readEnableSwitchTask(void)
{
#if !ENABLE_SWITCH_USE
  if (!g_enableSwitchStableOn || !g_enableSwitchRawOn) {
    g_enableSwitchRawOn = true;
    g_enableSwitchStableOn = true;
    g_enableSwitchLastRawOn = true;
    g_lcdForceRefresh = true;
  }
  return;
#endif

  bool rawOn = readEnableSwitchRawOn();

  if (rawOn != g_enableSwitchRawOn) {
    g_enableSwitchRawOn = rawOn;
    g_lcdForceRefresh = true;
  }

  if (rawOn != g_enableSwitchLastRawOn) {
    g_enableSwitchLastRawOn = rawOn;
    g_enableSwitchLastChangeTick = g_appTickCount;
  }

  if ((uint32_t)(g_appTickCount - g_enableSwitchLastChangeTick) >= MS_TO_TICKS(ENABLE_SWITCH_DEBOUNCE_MS)) {
    if (rawOn != g_enableSwitchStableOn) {
      g_enableSwitchStableOn = rawOn;

      if (!g_enableSwitchStableOn) {
        setOutputEnabledAtomic(false);
      } else if (!g_sensorFault) {
        setOutputEnabledAtomic(true);
#if BRINGUP_IO_TEST_MODE
      } else {
        setOutputEnabledAtomic(true);
#endif
      }
    }
  }
}

/*
  readUserButtonTask()

  Lee un boton de usuario activo en alto con debounce no bloqueante basado en
  la base cooperativa de 10 ms. Cuando detecta una presion estable, eleva
  g_userButtonPressedFlag.
  Cuando detecta la suelta estable despues de haber estado presionado, limpia
  esa bandera y publica un unico evento g_userButtonReleaseEvent consumible.
  Asi la accion asociada se dispara solo al soltar y no se repite mientras se
  mantiene el boton apretado. No debe llamarse desde ISR.
*/
void readUserButtonTask(void)
{
  bool rawPressed = readUserButtonRawPressed();

  if (rawPressed != g_userButtonLastRawPressed) {
    g_userButtonLastRawPressed = rawPressed;
    g_userButtonLastChangeTick = g_appTickCount;
  }

  if ((uint32_t)(g_appTickCount - g_userButtonLastChangeTick) < MS_TO_TICKS(USER_BUTTON_DEBOUNCE_MS)) {
    return;
  }

  if (rawPressed == g_userButtonStablePressed) {
    return;
  }

  g_userButtonStablePressed = rawPressed;

  if (g_userButtonStablePressed) {
    g_userButtonPressedFlag = true;
    return;
  }

  if (g_userButtonPressedFlag) {
    g_userButtonPressedFlag = false;
    g_userButtonReleaseEvent = true;
  }
}

/*
  consumeUserButtonReleaseEvent()

  Devuelve true una sola vez por cada secuencia presionar-soltar valida del
  boton de usuario. Leerlo consume el evento para evitar multiples avances de
  pagina con una misma pulsacion. No debe llamarse desde ISR.
*/
bool consumeUserButtonReleaseEvent(void)
{
  if (!g_userButtonReleaseEvent) {
    return false;
  }

  g_userButtonReleaseEvent = false;
  return true;
}

/*
  updateSimulatedZeroCrossTask()

  Genera una senal de banco en D7/PD7 para simular cruces por cero sin usar
  red electrica ni detector externo. La salida produce un pulso alto cada
  semiciclo teorico calculado desde MAINS_FREQ_HZ, usando millis(). Para probar
  la cadena completa se debe puentear D7 a D2/INT0 con una resistencia serie de
  1 k a 4.7 k. Esta funcion no es una referencia temporal de precision; solo
  sirve para validar recepcion de INT0, contador ZC y pulsos de salida D8. No
  usa librerias bloqueantes, no afecta Timer1 y no debe llamarse desde ISR.
*/
void updateSimulatedZeroCrossTask(void)
{
#if SIMULATED_ZERO_CROSS_OUTPUT_ENABLE
  uint32_t now = millis();

  if (g_simZeroCrossPulseActive) {
    if ((int32_t)(now - g_simZeroCrossPulseEndMs) >= 0) {
      SIM_ZERO_CROSS_PORT &= (uint8_t)~(1 << SIM_ZERO_CROSS_BIT);     // &= ~mascara limpia PD7: termina el pulso simulado.
      g_simZeroCrossPulseActive = false;
    }
  } else {
    if ((int32_t)(now - g_simZeroCrossNextPulseMs) >= 0) {
      SIM_ZERO_CROSS_PORT |= (uint8_t)(1 << SIM_ZERO_CROSS_BIT);      // |= OR con mascara: sube D7 y crea flanco para INT0.
      g_simZeroCrossPulseActive = true;
      g_simZeroCrossPulseEndMs = now + SIMULATED_ZERO_CROSS_PULSE_MS;
      g_simZeroCrossNextPulseMs += SIMULATED_ZERO_CROSS_PERIOD_MS;

      if ((int32_t)(now - g_simZeroCrossNextPulseMs) > (int32_t)SIMULATED_ZERO_CROSS_PERIOD_MS) {
        g_simZeroCrossNextPulseMs = now + SIMULATED_ZERO_CROSS_PERIOD_MS;
      }
    }
  }
#else
  SIM_ZERO_CROSS_PORT &= (uint8_t)~(1 << SIM_ZERO_CROSS_BIT);         // Modo deshabilitado: mantiene D7 en bajo si fue usado antes.
  g_simZeroCrossPulseActive = false;
#endif
}

/*
  setOutputEnabledAtomic()

  Publica el permiso de disparo que lee INT0. Si se deshabilita, fuerza el gate
  en bajo, desarma el semiciclo y apaga OCIE1A/OCIE1B para impedir disparos
  pendientes. Usa ATOMIC_BLOCK porque modifica variables volatile y registros
  compartidos con ISR. Puede llamarse desde loop; no debe llamarse desde una ISR
  porque usa una primitiva atomica pensada para contexto principal.
*/
void setOutputEnabledAtomic(bool enable)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_outputEnabled = enable;

    if (!enable) {
      setTriacGateLow();                                             // Baja PB0 antes de desarmar comparadores.
      g_semicycleArmed = false;
      g_semicycleFired = false;
      TIMSK1 &= (uint8_t)~((1 << OCIE1A) | (1 << OCIE1B));           // &= ~mascara limpia bits OCIE1A/OCIE1B: compare A/B quedan bloqueados.
      TIFR1 = (uint8_t)((1 << OCF1A) | (1 << OCF1B));                // Flags AVR: escribir 1 borra OCF1A y OCF1B.
    }
  }
}

/*
  getOutputEnabledAtomic()

  Devuelve el permiso de salida compartido con las ISR. Aunque bool es de 8
  bits en AVR, se lee dentro de ATOMIC_BLOCK para mantener una interfaz
  uniforme y evitar carreras si se agregan banderas relacionadas. No toca
  hardware y no debe llamarse desde ISR.
*/
bool getOutputEnabledAtomic(void)
{
  bool enabled;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    enabled = g_outputEnabled;
  }
  return enabled;
}

/*
  readEnableSwitchRawOn()

  Lee directamente PIND para interpretar la llave digital sin librerias. Con
  ENABLE_SWITCH_ACTIVE_LOW en 0, 5 V en D4 representa ON y 0 V representa OFF.
  Esta lectura no es bloqueante, no toca temporizacion critica y puede llamarse
  en loop. No es necesario llamarla desde ISR.
*/
bool readEnableSwitchRawOn(void)
{
  bool pinIsHigh = (ENABLE_SWITCH_PINREG & (uint8_t)(1 << ENABLE_SWITCH_BIT)) != 0; // AND con mascara aisla PD4; !=0 lo convierte a booleano.

#if ENABLE_SWITCH_ACTIVE_LOW
  return !pinIsHigh;
#else
  return pinIsHigh;
#endif
}

/*
  readUserButtonRawPressed()

  Lee directamente PIND para interpretar el boton de usuario. Esta version
  asume hardware con pull-down externo: 0 V en reposo y 5 V mientras el boton
  esta presionado. No es bloqueante y se usa solo desde loop.
*/
bool readUserButtonRawPressed(void)
{
  return (USER_BUTTON_PINREG & (uint8_t)(1 << USER_BUTTON_BIT)) != 0;
}

/*
  setTriacGateHigh()

  Activa el pin PB0/D8 escribiendo directamente en PORTB. Fisicamente enciende
  el LED de entrada del MOC3053 a traves de su resistencia serie, iniciando el
  disparo del TRIAC principal si hay corriente suficiente en la carga. Es una
  operacion de tiempo constante y sin librerias. Se conserva para codigo de
  loop, pero las ISR escriben PORTB directamente para no llamar funciones desde
  interrupciones.
*/
static inline void setTriacGateHigh(void)
{
  TRIAC_GATE_PORT |= (uint8_t)(1 << TRIAC_GATE_BIT);                 // |= OR con mascara: setea PB0 y conserva PB1..PB7.
}

/*
  setTriacGateLow()

  Desactiva PB0/D8 por acceso directo a PORTB. Fisicamente apaga el LED del
  MOC3053. El TRIAC de potencia seguira conduciendo hasta que su corriente caiga
  por debajo de la corriente de mantenimiento en el proximo cruce por cero; esta
  funcion solo corta el pulso de gate. Es de tiempo constante y sin librerias.
  Se conserva para codigo de loop; las ISR hacen el acceso al puerto dentro del
  propio vector.
*/
static inline void setTriacGateLow(void)
{
  TRIAC_GATE_PORT &= (uint8_t)~(1 << TRIAC_GATE_BIT);                // &= AND con mascara invertida: limpia PB0 y conserva PB1..PB7.
}

/*
  clampFloat()

  Limita un valor flotante entre dos cotas. Se usa en tareas no criticas para
  PID y conversion de angulo. No toca hardware ni librerias externas. No es
  costosa, pero no se necesita dentro de ISR.
*/
static inline float clampFloat(float value, float low, float high)
{
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

/*
  ISR(INT0_vect)

  Sincroniza cada semiciclo con el cruce por cero. Filtra pulsos repetidos muy
  cercanos, reinicia TCNT1, programa OCR1A/OCR1B, limpia flags y habilita las
  comparaciones solo si el sistema esta autorizado. No lee sensores, no calcula
  PID, no usa LCD, no llama librerias y no llama funciones auxiliares: todo el
  acceso al gate queda escrito dentro del propio vector. El objetivo fisico es
  preparar un unico disparo por semiciclo.
*/
ISR(INT0_vect)
{
  uint16_t elapsedTicks = TCNT1;                                     // Lectura directa de 16 bits del contador Timer1.
  bool acceptZeroCross =
      (!g_zeroCrossSynchronized || (elapsedTicks >= ZERO_CROSS_BLANK_TICKS)); // Acepta solo si paso la ventana de blanking.

  if (acceptZeroCross) {
    g_zeroCrossSynchronized = true;
    g_zeroCrossCount++;

    TRIAC_GATE_PORT &= (uint8_t)~(1 << TRIAC_GATE_BIT);              // &= ~mascara limpia PB0: corta inmediatamente el gate.

    TIMSK1 &= (uint8_t)~((1 << OCIE1A) | (1 << OCIE1B));             // &= ~mascara limpia OCIE1A/OCIE1B antes de reprogramar OCR.
    TCNT1 = 0;                                                       // Asignacion directa: contador de Timer1 vuelve a cero.
    TIFR1 = (uint8_t)((1 << OCF1A) | (1 << OCF1B) | (1 << TOV1));    // Flags AVR: escribir 1 borra compare/overflow pendientes.

    if (!g_outputEnabled) {
      g_semicycleArmed = false;
      g_semicycleFired = false;
      g_blockedPulseCount++;
    } else {
      uint16_t delayTicks = g_firingDelayTicks;

      if (delayTicks < MIN_FIRING_DELAY_TICKS) {
        delayTicks = MIN_FIRING_DELAY_TICKS;
      }

      if (delayTicks > MAX_SAFE_DELAY_TICKS) {
        delayTicks = MAX_SAFE_DELAY_TICKS;
      }

      OCR1A = delayTicks;                                            // Asignacion directa: OCR1A fija el instante de inicio de pulso.
      OCR1B = delayTicks + TRIAC_GATE_PULSE_TICKS;                   // Asignacion directa: OCR1B fija el corte 100 us despues.

      g_semicycleArmed = true;
      g_semicycleFired = false;

      TIMSK1 |= (uint8_t)((1 << OCIE1A) | (1 << OCIE1B));            // |= prende OCIE1A y OCIE1B: arma ambos compare del semiciclo.
    }
  }
}

/*
  ISR(TIMER1_COMPA_vect)

  Marca el instante de disparo. Si el semiciclo esta armado y la salida sigue
  habilitada, sube PB0 para encender el MOC3053. Deshabilita OCIE1A para impedir
  cualquier repeticion dentro del mismo semiciclo y deja OCIE1B encargado de
  cortar el pulso 100 us despues. No ejecuta calculos pesados, no llama
  librerias y no llama funciones auxiliares.
*/
ISR(TIMER1_COMPA_vect)
{
  TIMSK1 &= (uint8_t)~(1 << OCIE1A);                                 // &= ~mascara limpia OCIE1A: compare A no puede repetirse.

  if (g_outputEnabled && g_semicycleArmed && !g_semicycleFired) {
    TRIAC_GATE_PORT |= (uint8_t)(1 << TRIAC_GATE_BIT);               // |= OR con mascara: sube PB0 y enciende LED del MOC3053.
    g_semicycleFired = true;
    g_firedPulseCount++;
  } else {
    TRIAC_GATE_PORT &= (uint8_t)~(1 << TRIAC_GATE_BIT);              // &= ~mascara: baja PB0 aunque los demas bits de PORTB sigan intactos.
  }
}

/*
  ISR(TIMER1_COMPB_vect)

  Finaliza el pulso de gate. Baja PB0, apaga las comparaciones de Timer1 y deja
  el semiciclo desarmado hasta el proximo INT0. Esto garantiza un solo pulso por
  semiciclo. No lee sensores, no actualiza display, no usa llamadas bloqueantes
  y no llama funciones auxiliares.
*/
ISR(TIMER1_COMPB_vect)
{
  TRIAC_GATE_PORT &= (uint8_t)~(1 << TRIAC_GATE_BIT);                // &= ~mascara limpia PB0: termina el pulso de gate.
  TIMSK1 &= (uint8_t)~((1 << OCIE1A) | (1 << OCIE1B));               // &= ~mascara apaga OCIE1A/OCIE1B hasta el siguiente cruce.
  TIFR1 = (uint8_t)((1 << OCF1A) | (1 << OCF1B));                    // Flags AVR: escribir 1 borra banderas de compare consumidas.
  g_semicycleArmed = false;
}
