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
*/

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <max6675.h>
#include <math.h>
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

#define CONTROL_INTERVAL_MS            250UL
#define CONTROL_SAMPLE_TIME_SEC        0.250f
#define LCD_MAX_INTERVAL_MS            1000UL
#define MAX6675_STARTUP_MS             500UL

#define SETPOINT_MIN_C                 50.0f
#define SETPOINT_MAX_C                 300.0f

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
#define LCD_TEMP_DELTA_C               0.5f
#define LCD_SETPOINT_DELTA_C           1.0f
#define LCD_ANGLE_DELTA_DEG            1.0f

// ---------------------------------------------------------------------------
// Pinout configurable
// ---------------------------------------------------------------------------

// Cruce por cero: D2 / PD2 / INT0.
#define ZERO_CROSS_DDR                 DDRD
#define ZERO_CROSS_PORT                PORTD
#define ZERO_CROSS_PINREG              PIND
#define ZERO_CROSS_BIT                 PD2
#define ZERO_CROSS_USE_PULLUP          0
#define ZERO_CROSS_RISING_EDGE         1

// Salida de disparo: D8 / PB0.
#define TRIAC_GATE_DDR                 DDRB
#define TRIAC_GATE_PORT                PORTB
#define TRIAC_GATE_BIT                 PB0

// Llave digital ON/OFF: D4 / PD4, pull-up interno.
#define ENABLE_SWITCH_DDR              DDRD
#define ENABLE_SWITCH_PORT             PORTD
#define ENABLE_SWITCH_PINREG           PIND
#define ENABLE_SWITCH_BIT              PD4
#define ENABLE_SWITCH_ACTIVE_LOW       1
#define ENABLE_SWITCH_DEBOUNCE_MS      40UL

// Potenciometro de setpoint: A0 / PC0 / ADC0.
#define POT_ADC_CHANNEL                0
#define POT_ADC_DDR                    DDRC
#define POT_ADC_PORT                   PORTC
#define POT_ADC_BIT                    PC0

// MAX6675. Los numeros digitales son los que usa la libreria.
#define MAX6675_SCK_PIN                13
#define MAX6675_CS_PIN                 10
#define MAX6675_SO_PIN                 12

// Inicializacion directa para el pinout sugerido del MAX6675.
#define MAX6675_SCK_DDR                DDRB
#define MAX6675_SCK_PORT               PORTB
#define MAX6675_SCK_BIT                PB5
#define MAX6675_CS_DDR                 DDRB
#define MAX6675_CS_PORT                PORTB
#define MAX6675_CS_BIT                 PB2
#define MAX6675_SO_DDR                 DDRB
#define MAX6675_SO_PORT                PORTB
#define MAX6675_SO_BIT                 PB4

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

#if (TIMER1_TICKS_PER_US != 2UL)
#warning "Este sketch esta ajustado para F_CPU=16MHz y prescaler 8: 0.5 us/tick."
#endif

#if (HALF_CYCLE_TICKS >= 65535UL)
#error "El semiciclo no entra en Timer1 de 16 bits con este prescaler."
#endif

#if (MAX_SAFE_DELAY_TICKS <= TRIAC_GATE_PULSE_TICKS)
#error "No queda ventana temporal segura para el pulso de disparo."
#endif

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

float readSetpointPot(void);
uint16_t readADCBlocking(uint8_t channel);
float readTemperatureMAX6675(void);
float computePID(float setpointC, float measuredC);
float powerToFiringAngle(float powerPct);
uint16_t angleToTimerTicks(float angleDeg);

void updateControlTask(void);
void updateLCDTask(void);
void readEnableSwitchTask(void);
void setOutputEnabledAtomic(bool enable);
bool getOutputEnabledAtomic(void);
bool readEnableSwitchRawOn(void);

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
bool g_enableSwitchStableOn = false;
bool g_enableSwitchLastRawOn = false;
uint32_t g_enableSwitchLastChangeMs = 0;
uint32_t g_lastControlMs = 0;
uint32_t g_lastLCDMs = 0;
uint32_t g_max6675ReadyMs = MAX6675_STARTUP_MS;

float g_lcdLastTempC = NAN;
float g_lcdLastSetpointC = NAN;
float g_lcdLastAngleDeg = NAN;
bool g_lcdLastOutputEnabled = false;
bool g_lcdLastSensorFault = true;

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

  g_enableSwitchLastRawOn = readEnableSwitchRawOn();
  g_enableSwitchStableOn = g_enableSwitchLastRawOn;
  setOutputEnabledAtomic(false);
}

/*
  loop()

  Ejecuta una arquitectura cooperativa: debounce de llave en cada pasada,
  control cada 250 ms y LCD segun umbrales o cada 1 s. Si una lectura del
  MAX6675 o una transaccion I2C tarda mas de lo esperado, el gate del TRIAC
  sigue dependiendo de INT0 y Timer1. No debe incluir delay() ni rutinas que
  deshabiliten interrupciones durante tiempos largos.
*/
void loop(void)
{
  readEnableSwitchTask();
  updateControlTask();
  updateLCDTask();
}

/*
  initPins()

  Configura directamente DDR y PORT de los pines usados. PB0 queda como salida
  y en bajo para mantener apagado el LED del MOC3053. PD2 queda como entrada de
  cruce por cero, con pull-up opcional segun el detector externo. PD4 queda
  como entrada con pull-up interno para la llave ON/OFF. PC0 se deja como
  entrada analogica sin pull-up. Tambien se inicializan los pines sugeridos del
  MAX6675 para estados electricos seguros. No usa librerias bloqueantes y no
  debe llamarse desde ISR.
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
  TRIAC_GATE_DDR |= (uint8_t)(1 << TRIAC_GATE_BIT);
  TRIAC_GATE_PORT &= (uint8_t)~(1 << TRIAC_GATE_BIT);

  MAX6675_CS_DDR |= (uint8_t)(1 << MAX6675_CS_BIT);
  MAX6675_CS_PORT |= (uint8_t)(1 << MAX6675_CS_BIT);
  MAX6675_SCK_DDR |= (uint8_t)(1 << MAX6675_SCK_BIT);
  MAX6675_SCK_PORT &= (uint8_t)~(1 << MAX6675_SCK_BIT);
  MAX6675_SO_DDR &= (uint8_t)~(1 << MAX6675_SO_BIT);
  MAX6675_SO_PORT &= (uint8_t)~(1 << MAX6675_SO_BIT);

  /*
    DDRD/PORTD:
    - PD2 / INT0: entrada de cruce por cero. El pull-up se habilita solo si el
      detector externo lo requiere.
    - PD4: entrada de llave ON/OFF con pull-up interno. Con el cableado
      habitual, llave cerrada a GND significa ON.
  */
  ZERO_CROSS_DDR &= (uint8_t)~(1 << ZERO_CROSS_BIT);
#if ZERO_CROSS_USE_PULLUP
  ZERO_CROSS_PORT |= (uint8_t)(1 << ZERO_CROSS_BIT);
#else
  ZERO_CROSS_PORT &= (uint8_t)~(1 << ZERO_CROSS_BIT);
#endif

  ENABLE_SWITCH_DDR &= (uint8_t)~(1 << ENABLE_SWITCH_BIT);
  ENABLE_SWITCH_PORT |= (uint8_t)(1 << ENABLE_SWITCH_BIT);

  /*
    DDRC/PORTC:
    - PC0 / ADC0 / A0: entrada analogica de potenciometro. El pull-up se deja
      apagado para no deformar el divisor resistivo.
  */
  POT_ADC_DDR &= (uint8_t)~(1 << POT_ADC_BIT);
  POT_ADC_PORT &= (uint8_t)~(1 << POT_ADC_BIT);
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

  EIMSK &= (uint8_t)~(1 << INT0);

  /*
    EICRA:
    ISC01 ISC00 = 11: flanco ascendente en INT0.
    ISC01 ISC00 = 10: flanco descendente en INT0.
    Si el detector entrega pulsos invertidos, cambiar ZERO_CROSS_RISING_EDGE.
  */
  EICRA &= (uint8_t)~((1 << ISC01) | (1 << ISC00));
#if ZERO_CROSS_RISING_EDGE
  EICRA |= (uint8_t)((1 << ISC01) | (1 << ISC00));
#else
  EICRA |= (uint8_t)(1 << ISC01);
#endif

  EIFR = (uint8_t)(1 << INTF0);
  EIMSK |= (uint8_t)(1 << INT0);

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
  TCCR1A = 0;

  /*
    TCCR1B:
    WGM13:2 = 00: modo normal, TCNT1 cuenta hasta 0xFFFF.
    CS12:0 = 010: prescaler 8. A 16 MHz equivale a 2 MHz, 0.5 us por tick.
  */
  TCCR1B = (uint8_t)(1 << CS11);

  /*
    TCNT1/OCR1A/OCR1B:
    TCNT1 se reinicia en cada cruce por cero. OCR1A se carga con el retardo de
    disparo del semiciclo y OCR1B con retardo + 100 us.
  */
  TCNT1 = 0;
  OCR1A = MAX_SAFE_DELAY_TICKS;
  OCR1B = MAX_SAFE_DELAY_TICKS + TRIAC_GATE_PULSE_TICKS;

  /*
    TIMSK1:
    Las interrupciones de comparacion quedan apagadas por defecto. INT0 las
    habilita solo para el semiciclo actual, evitando redisparos.
  */
  TIMSK1 &= (uint8_t)~((1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1));

  /*
    TIFR1:
    En AVR las banderas se limpian escribiendo 1. Se eliminan compare flags y
    overflow flags viejas antes de empezar.
  */
  TIFR1 = (uint8_t)((1 << OCF1A) | (1 << OCF1B) | (1 << TOV1));

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
  ADMUX = (uint8_t)((1 << REFS0) | (POT_ADC_CHANNEL & 0x0F));

  /*
    ADCSRA:
    ADEN = 1: habilita ADC.
    ADPS2:0 = 111: prescaler 128.
    ADIE = 0: no se usa interrupcion ADC; el muestreo no es critico.
  */
  ADCSRA = (uint8_t)((1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));

  /*
    DIDR0:
    Deshabilita el buffer digital de ADC0 para reducir consumo y ruido.
  */
  DIDR0 |= (uint8_t)(1 << ADC0D);
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
  g_max6675ReadyMs = millis() + MAX6675_STARTUP_MS;
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
  ADMUX = (uint8_t)((ADMUX & 0xF0) | (channel & 0x0F));
  ADCSRA |= (uint8_t)(1 << ADSC);

  while (ADCSRA & (1 << ADSC)) {
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
  if ((int32_t)(millis() - g_max6675ReadyMs) < 0) {
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

  Se ejecuta cada CONTROL_INTERVAL_MS. Lee setpoint y temperatura, valida falla
  de termocupla, calcula el PID, transforma potencia a angulo y publica el
  retardo de disparo en ticks para las ISR. La publicacion de la variable de 16
  bits se hace con ATOMIC_BLOCK. Si el sensor falla, fuerza potencia cero y
  deshabilita inmediatamente la salida del TRIAC. No debe llamarse desde ISR.
*/
void updateControlTask(void)
{
  uint32_t now = millis();
  if ((uint32_t)(now - g_lastControlMs) < CONTROL_INTERVAL_MS) {
    return;
  }
  g_lastControlMs = now;

  g_setpointC = readSetpointPot();
  float newTemperatureC = readTemperatureMAX6675();

  if (isnan(newTemperatureC)) {
    g_sensorFault = true;
    g_temperatureC = NAN;
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
  }

  g_sensorFault = false;
  g_temperatureC = newTemperatureC;
  g_powerPct = computePID(g_setpointC, g_temperatureC);
  g_angleDeg = powerToFiringAngle(g_powerPct);
  uint16_t nextDelayTicks = angleToTimerTicks(g_angleDeg);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_firingDelayTicks = nextDelayTicks;
  }

  setOutputEnabledAtomic(g_enableSwitchStableOn);
}

/*
  updateLCDTask()

  Actualiza el LCD solo si hubo cambios significativos de temperatura,
  setpoint, angulo/estado o si paso LCD_MAX_INTERVAL_MS. Usa LiquidCrystal_I2C,
  por lo tanto es bloqueante y nunca debe ejecutarse dentro de una ISR. El
  refresco es intencionalmente lento para no llenar el bus I2C de trafico
  innecesario; el control por fase permanece en Timer1.
*/
void updateLCDTask(void)
{
  uint32_t now = millis();
  bool outputEnabled = getOutputEnabledAtomic();

  bool forceByTime = ((uint32_t)(now - g_lastLCDMs) >= LCD_MAX_INTERVAL_MS);
  bool tempChanged =
      isnan(g_lcdLastTempC) ||
      isnan(g_temperatureC) ||
      (fabs(g_temperatureC - g_lcdLastTempC) >= LCD_TEMP_DELTA_C);
  bool setpointChanged =
      isnan(g_lcdLastSetpointC) ||
      (fabs(g_setpointC - g_lcdLastSetpointC) >= LCD_SETPOINT_DELTA_C);
  bool angleChanged =
      isnan(g_lcdLastAngleDeg) ||
      (fabs(g_angleDeg - g_lcdLastAngleDeg) >= LCD_ANGLE_DELTA_DEG);
  bool stateChanged =
      (outputEnabled != g_lcdLastOutputEnabled) ||
      (g_sensorFault != g_lcdLastSensorFault);

  if (!forceByTime && !tempChanged && !setpointChanged && !angleChanged && !stateChanged) {
    return;
  }

  g_lastLCDMs = now;
  g_lcdLastTempC = g_temperatureC;
  g_lcdLastSetpointC = g_setpointC;
  g_lcdLastAngleDeg = g_angleDeg;
  g_lcdLastOutputEnabled = outputEnabled;
  g_lcdLastSensorFault = g_sensorFault;

  lcd.setCursor(0, 0);
  if (g_sensorFault) {
    lcd.print(F("T:ERR "));
  } else {
    lcd.print(F("T:"));
    lcd.print(g_temperatureC, 1);
    lcd.print(F(" "));
  }
  lcd.print(F("S:"));
  lcd.print(g_setpointC, 0);
  lcd.print(F("   "));

  lcd.setCursor(0, 1);
  lcd.print(F("A:"));
  lcd.print((int)(g_angleDeg + 0.5f));
  lcd.print(F(" P:"));
  lcd.print((int)(g_powerPct + 0.5f));
  lcd.print(F("%"));

  if (outputEnabled) {
    lcd.print(F(" ON "));
  } else {
    lcd.print(F(" OFF"));
  }
}

/*
  readEnableSwitchTask()

  Lee la llave ON/OFF con debounce por software no bloqueante. La entrada tiene
  pull-up interno y, por defecto, nivel bajo significa ON. Si el estado estable
  pasa a OFF, corta inmediatamente el gate del MOC3053 y deshabilita las
  comparaciones de Timer1 de forma atomica. No detiene el sensor ni el LCD. No
  debe llamarse desde ISR.
*/
void readEnableSwitchTask(void)
{
  uint32_t now = millis();
  bool rawOn = readEnableSwitchRawOn();

  if (rawOn != g_enableSwitchLastRawOn) {
    g_enableSwitchLastRawOn = rawOn;
    g_enableSwitchLastChangeMs = now;
  }

  if ((uint32_t)(now - g_enableSwitchLastChangeMs) >= ENABLE_SWITCH_DEBOUNCE_MS) {
    if (rawOn != g_enableSwitchStableOn) {
      g_enableSwitchStableOn = rawOn;

      if (!g_enableSwitchStableOn) {
        setOutputEnabledAtomic(false);
      } else if (!g_sensorFault) {
        setOutputEnabledAtomic(true);
      }
    }
  }
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
      setTriacGateLow();
      g_semicycleArmed = false;
      g_semicycleFired = false;
      TIMSK1 &= (uint8_t)~((1 << OCIE1A) | (1 << OCIE1B));
      TIFR1 = (uint8_t)((1 << OCF1A) | (1 << OCF1B));
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
  ENABLE_SWITCH_ACTIVE_LOW en 1, la llave conectada a GND representa ON gracias
  al pull-up interno. Esta lectura no es bloqueante, no toca temporizacion
  critica y puede llamarse en loop. No es necesario llamarla desde ISR.
*/
bool readEnableSwitchRawOn(void)
{
  bool pinIsHigh = (ENABLE_SWITCH_PINREG & (uint8_t)(1 << ENABLE_SWITCH_BIT)) != 0;

#if ENABLE_SWITCH_ACTIVE_LOW
  return !pinIsHigh;
#else
  return pinIsHigh;
#endif
}

/*
  setTriacGateHigh()

  Activa el pin PB0/D8 escribiendo directamente en PORTB. Fisicamente enciende
  el LED de entrada del MOC3053 a traves de su resistencia serie, iniciando el
  disparo del TRIAC principal si hay corriente suficiente en la carga. Es una
  operacion de tiempo constante, sin librerias y apta para ISR.
*/
static inline void setTriacGateHigh(void)
{
  TRIAC_GATE_PORT |= (uint8_t)(1 << TRIAC_GATE_BIT);
}

/*
  setTriacGateLow()

  Desactiva PB0/D8 por acceso directo a PORTB. Fisicamente apaga el LED del
  MOC3053. El TRIAC de potencia seguira conduciendo hasta que su corriente caiga
  por debajo de la corriente de mantenimiento en el proximo cruce por cero; esta
  funcion solo corta el pulso de gate. Es de tiempo constante y apta para ISR.
*/
static inline void setTriacGateLow(void)
{
  TRIAC_GATE_PORT &= (uint8_t)~(1 << TRIAC_GATE_BIT);
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
  PID, no usa LCD y no llama librerias. El objetivo fisico es preparar un unico
  disparo por semiciclo.
*/
ISR(INT0_vect)
{
  uint16_t elapsedTicks = TCNT1;

  if (g_zeroCrossSynchronized && (elapsedTicks < ZERO_CROSS_BLANK_TICKS)) {
    return;
  }

  g_zeroCrossSynchronized = true;
  g_zeroCrossCount++;
  setTriacGateLow();

  TIMSK1 &= (uint8_t)~((1 << OCIE1A) | (1 << OCIE1B));
  TCNT1 = 0;
  TIFR1 = (uint8_t)((1 << OCF1A) | (1 << OCF1B) | (1 << TOV1));

  if (!g_outputEnabled) {
    g_semicycleArmed = false;
    g_semicycleFired = false;
    g_blockedPulseCount++;
    return;
  }

  uint16_t delayTicks = g_firingDelayTicks;

  if (delayTicks < MIN_FIRING_DELAY_TICKS) {
    delayTicks = MIN_FIRING_DELAY_TICKS;
  }

  if (delayTicks > MAX_SAFE_DELAY_TICKS) {
    delayTicks = MAX_SAFE_DELAY_TICKS;
  }

  OCR1A = delayTicks;
  OCR1B = delayTicks + TRIAC_GATE_PULSE_TICKS;

  g_semicycleArmed = true;
  g_semicycleFired = false;

  TIMSK1 |= (uint8_t)((1 << OCIE1A) | (1 << OCIE1B));
}

/*
  ISR(TIMER1_COMPA_vect)

  Marca el instante de disparo. Si el semiciclo esta armado y la salida sigue
  habilitada, sube PB0 para encender el MOC3053. Deshabilita OCIE1A para impedir
  cualquier repeticion dentro del mismo semiciclo y deja OCIE1B encargado de
  cortar el pulso 100 us despues. No ejecuta calculos pesados ni librerias.
*/
ISR(TIMER1_COMPA_vect)
{
  TIMSK1 &= (uint8_t)~(1 << OCIE1A);

  if (g_outputEnabled && g_semicycleArmed && !g_semicycleFired) {
    setTriacGateHigh();
    g_semicycleFired = true;
    g_firedPulseCount++;
  } else {
    setTriacGateLow();
  }
}

/*
  ISR(TIMER1_COMPB_vect)

  Finaliza el pulso de gate. Baja PB0, apaga las comparaciones de Timer1 y deja
  el semiciclo desarmado hasta el proximo INT0. Esto garantiza un solo pulso por
  semiciclo. No lee sensores, no actualiza display y no usa llamadas
  bloqueantes.
*/
ISR(TIMER1_COMPB_vect)
{
  setTriacGateLow();
  TIMSK1 &= (uint8_t)~((1 << OCIE1A) | (1 << OCIE1B));
  TIFR1 = (uint8_t)((1 << OCF1A) | (1 << OCF1B));
  g_semicycleArmed = false;
}
