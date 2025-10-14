/**
 * @file proyecto_final.ino
 * @brief Sistema de confort térmico inteligente para aula (documentado para Doxygen).
 *
 * @details
 * Este archivo contiene el firmware principal para un sistema embebido basado en
 * Arduino que monitorea temperatura, humedad y luz para mantener condiciones de
 * confort térmico en un aula de dos estudiantes. El sistema emplea una máquina de
 * estados (StateMachineLib), tareas asíncronas (AsyncTaskLib), lectura de RFID
 * (MFRC522), DHT11, NTC, LDR, y controla actuadores como relé, servo y buzzer.
 *
 * @author Edward Esteban Dávila Salazar
 * @author Juan Jose Hurtado Molano
 * @date Octubre 2025
 */

#include "StateMachineLib.h"
#include <AsyncTaskLib.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Servo.h>
#include <MFRC522.h>
#include <SPI.h>
#include <EEPROM.h>
#include <math.h>

// ==================== PINES ====================
/**
 * @def LED_RED
 * @brief Pin digital conectado al LED rojo indicador.
 */
#define LED_RED        26

/**
 * @def LED_BLUE
 * @brief Pin digital conectado al LED azul indicador.
 */
#define LED_BLUE       22

/**
 * @def BUTTON_PIN
 * @brief Pin del pulsador físico usado para desbloqueo.
 */
#define BUTTON_PIN      7

/**
 * @def RELAY_PIN
 * @brief Pin que controla el módulo relé (ventilador 12V).
 */
#define RELAY_PIN      31

/**
 * @def SERVO_PIN
 * @brief Pin PWM conectado al servomotor.
 */
#define SERVO_PIN       6

/**
 * @def BUZZER_PIN
 * @brief Pin del buzzer (alarma sonora).
 */
#define BUZZER_PIN      8

/**
 * @def BUZZER_ACTIVE_HIGH
 * @brief Definición del comportamiento lógico del buzzer.
 * Si el buzzer suena cuando el pin es HIGH, ponga 1; si suena con LOW, ponga 0.
 */
#define BUZZER_ACTIVE_HIGH 0

/**
 * @brief Habilita el buzzer (según polaridad definida).
 */
inline void buzzerOn()  { digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_HIGH ? HIGH : LOW); }

/**
 * @brief Deshabilita el buzzer (según polaridad definida).
 */
inline void buzzerOff() { digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_HIGH ? LOW  : HIGH); }

// DHT11
#define DHTPIN         10
#define DHTTYPE        DHT11

// RFID
#define RST_PIN         9
#define SS_PIN         53

// Sensores analógicos
#define NTC_PIN        A0   // Temperatura (KY-013)
#define LDR_PIN        A1   // Luz (LDR o potenciómetro)

// LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

// Keypad
const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = {
  { '1','2','3','A' },
  { '4','5','6','B' },
  { '7','8','9','C' },
  { '*','0','#','D' }
};
byte rowPins[ROWS] = { 28, 30, 32, 34 };
byte colPins[COLS] = { 36, 38, 40, 42 };

// ==================== OBJETOS ====================
DHT dht(DHTPIN, DHTTYPE);     /**< @brief Objeto DHT para lectura de humedad. */
Servo servo;                  /**< @brief Objeto Servo para control de posición. */
MFRC522 mfrc522(SS_PIN, RST_PIN); /**< @brief Lector RFID RC522 (SPI). */
MFRC522::MIFARE_Key mifareKey;    /**< @brief Clave MIFARE por defecto. */
Keypad keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS); /**< @brief Teclado matricial. */
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                    /**< @brief Pantalla LCD 16x2. */

// RFID UIDs válidos (ajústalos a los tuyos)
byte tarjetaUID[4] = {0x9A, 0xD5, 0xB3, 0x05};
byte llaveroUID[4] = {0x3A, 0xD4, 0xA8, 0x00};

// ==================== VARIABLES ====================
/**
 * @brief Clave almacenada por defecto.
 */
String clave_store = "1234";

float temperatura_actual = 0.0f; /**< @brief Temperatura medida por NTC (°C). */
float humedad_actual     = 0.0f; /**< @brief Humedad relativa medida por DHT (%). */
int   luz_actual         = 0;    /**< @brief Valor del LDR (ADC). */
float pmv_actual         = 0.0f; /**< @brief Valor PMV calculado. */

int   streak_temp_alta   = 0;    /**< @brief Contador de lecturas consecutivas > umbral. */
const float TEMP_ALARMA_HI  = 30.0f; /**< @brief Umbral superior para alarma (°C). */
const float TEMP_ALARMA_LO  = 29.5f; /**< @brief Umbral inferior con histéresis (°C). */

// Promedios simples para estabilidad
const uint8_t AVG_N = 5;
float bufT[AVG_N];  uint8_t idxT = 0;
float bufRH[AVG_N]; uint8_t idxRH = 0;

// Ventana de “reposo” al volver a MONITOR
unsigned long monitor_hold_until = 0;
const unsigned long MONITOR_HOLD_MS = 2500;

// ==================== NTC (10k/10k típico) ====================
const float NTC_BETA      = 3950.0f;
const float NTC_R_SERIES  = 10000.0f; /**< @brief Resistencia serie del divisor NTC. */
const float NTC_R0        = 10000.0f;
const float NTC_T0        = 298.15f;  /**< @brief Temperatura de referencia 25°C en Kelvin. */

// ==================== PMV por LOOKUP (T en °C, RH %) ====================
const float PMV_HIGH_TH  =  0.5f; /**< @brief Umbral PMV alto */
const float PMV_LOW_TH   = -0.7f; /**< @brief Umbral PMV bajo */

const uint8_t NT=6, NH=5;
const float Tgrid[NT]  = {20,23,26,29,32,35};
const float RHgrid[NH] = {30,45,60,75,90};
const float pmvLUT[NH][NT] = {
//T:   20    23    26    29    32    35
     {-1.6, -1.2, -0.5,  0.1,  0.7,  1.3}, // 30%
     {-1.5, -1.0, -0.3,  0.3,  0.9,  1.6}, // 45%
     {-1.4, -0.8,  0.0,  0.5,  1.1,  1.9}, // 60%
     {-1.3, -0.6,  0.2,  0.7,  1.3,  2.1}, // 75%
     {-1.2, -0.4,  0.3,  0.9,  1.5,  2.3}  // 90%
};

// ==================== FSM ====================
/**
 * @enum State
 * @brief Estados de la máquina de estados principal.
 */
enum State { inicio, Config, Bloqueado, Alarma, Monitor, pmv_alto, pmv_bajo };

/**
 * @enum Input
 * @brief Tipos de señales/entradas evaluadas por la máquina de estados.
 */
enum Input { tiempo, boton, Unknown, pmv, temperatura, keypadInput, keypadBlock, alarmaTemp };

StateMachine stateMachine(7, 24); /**< @brief Máquina de estados (StateMachineLib). */
Input inputSignal = Unknown;      /**< @brief Señal de entrada actual. */

// ==================== TAREAS ====================
/**
 * @brief Tarea asíncrona que marca el evento de tiempo para el monitor (cada 2500 ms).
 */
AsyncTask taskMonitor(2500, true, [](){ inputSignal = tiempo; });

// Tareas de parpadeo y control por estado (descritas con sus intervalos)
bool rBlock = false;
AsyncTask taskRedBlock(500, true, [](){
  rBlock = !rBlock; digitalWrite(LED_RED, rBlock?HIGH:LOW);
});

bool rHigh = false;
AsyncTask taskRedHigh(300, true, [](){
  rHigh = !rHigh;
  digitalWrite(LED_RED, rHigh?HIGH:LOW);
  taskRedHigh.SetIntervalMillis(rHigh?300:100);
});

bool bLow = false;
AsyncTask taskBlueLow(100, true, [](){
  bLow = !bLow;
  digitalWrite(LED_BLUE, bLow?HIGH:LOW);
  taskBlueLow.SetIntervalMillis(bLow?100:400);
});

bool rAlarm=false, buzz=false;
AsyncTask taskRedAlarm(100, true, [](){
  rAlarm = !rAlarm;
  digitalWrite(LED_RED, rAlarm?HIGH:LOW);
  taskRedAlarm.SetIntervalMillis(rAlarm?100:500);
});
AsyncTask taskBuzzer(100, true, [](){
  buzz = !buzz;
  if (buzz) buzzerOn(); else buzzerOff();
  taskBuzzer.SetIntervalMillis(buzz?100:500);
});

AsyncTask taskStayPMVHigh(3000, false, [](){ inputSignal = tiempo; }); // 3 s
AsyncTask taskStayPMVLow (2000, false, [](){ inputSignal = tiempo; }); // 2 s

volatile bool tag_ok = false;
AsyncTask taskWaitToMonitor(5000, false, [](){ inputSignal = tiempo; });

// ==================== PROTOTIPOS ====================
void setupStateMachine();
void enteringInicio();   void leavingInicio();
void enteringConfig();   void leavingConfig();
void enteringBloqueado();void leavingBloqueado();
void enteringAlarma();   void leavingAlarma();
void enteringMonitor();  void leavingMonitor();
void enteringPMVALTO();  void leavingPmvAlto();
void enteringPMVBAJO();  void leavingPmvBajo();

int  readInput();
void leerDatosRFID();

// ==================== UTIL ====================
/**
 * @brief Restringe un valor entre dos límites (clamp).
 * @param v Valor a restringir.
 * @param lo Límite inferior.
 * @param hi Límite superior.
 * @return Valor limitado a [lo, hi].
 */
static inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

/**
 * @brief Inserta un valor en buffer circular y calcula promedio simple.
 * @param buf Puntero al buffer de floats.
 * @param idx Índice actual (se actualiza por referencia).
 * @param v Valor a insertar.
 * @return Promedio simple del buffer tras insertar el nuevo valor.
 */
float pushAvg(float *buf, uint8_t &idx, float v){
  buf[idx]=v; idx=(idx+1)%AVG_N;
  float s=0; for(uint8_t i=0;i<AVG_N;i++) s+=buf[i];
  return s/AVG_N;
}

/**
 * @brief Lee el ADC del NTC y convierte a grados Celsius usando la ecuación Beta.
 * @return Temperatura en °C calculada a partir del NTC.
 */
float readNTC_C(){
  int adc = analogRead(NTC_PIN);
  float v  = (adc<=1022) ? (adc*(5.0f/1023.0f)) : 4.99f;
  float rN = (NTC_R_SERIES * v) / (5.0f - v);
  float invT = (1.0f/NTC_T0) + (1.0f/NTC_BETA)*logf(rN/NTC_R0);
  return (1.0f/invT) - 273.15f;
}

/**
 * @brief Busca el índice inferior en un array ordenado donde 'x' encaja.
 * @param g Puntero al array ordenado.
 * @param n Longitud del array.
 * @param x Valor a buscar.
 * @return Índice i tal que g[i] <= x <= g[i+1] (o 0 / n-2 en límites).
 */
int lowerIndex(const float *g, uint8_t n, float x){
  if (x<=g[0]) return 0;
  for(uint8_t i=0;i<n-1;i++) if(x<=g[i+1]) return i;
  return n-2;
}

/**
 * @brief Interpola el valor PMV a partir de una tabla (lookup table) en T y RH.
 * @param T Temperatura en °C.
 * @param RH Humedad relativa en %.
 * @return Valor PMV interpolado (limitado a [-3, 3]).
 */
float pmv_from_table(float T, float RH){
  const uint8_t NTloc=NT, NHloc=NH;
  float Tm = clampf(T, Tgrid[0], Tgrid[NTloc-1]);
  float RHm= clampf(RH,RHgrid[0],RHgrid[NHloc-1]);
  int it=lowerIndex(Tgrid,NTloc,Tm), ih=lowerIndex(RHgrid,NHloc,RHm);
  float t0=Tgrid[it], t1=Tgrid[it+1], h0=RHgrid[ih], h1=RHgrid[ih+1];
  float ft=(Tm-t0)/(t1-t0), fh=(RHm-h0)/(h1-h0);
  float Q11=pmvLUT[ih][it],   Q21=pmvLUT[ih][it+1];
  float Q12=pmvLUT[ih+1][it], Q22=pmvLUT[ih+1][it+1];
  float R1=Q11+ft*(Q21-Q11), R2=Q12+ft*(Q22-Q12);
  float res=R1+fh*(R2-R1);
  if(res>3)res=3; if(res<-3)res=-3;
  return res;
}

// ==================== SETUP ====================
/**
 * @brief Configuración inicial del hardware y arranque de la FSM.
 *
 * Inicializa el puerto serie, pines, servomotor, lector RFID, LCD, sensor DHT,
 * buffers de promediado, la máquina de estados y la tarea de monitor.
 */
void setup(){
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);   pinMode(LED_BLUE, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT); pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_RED, LOW); digitalWrite(LED_BLUE, LOW);
  digitalWrite(RELAY_PIN, LOW); buzzerOff();

  servo.attach(SERVO_PIN); servo.write(0);

  SPI.begin(); mfrc522.PCD_Init();
  for(byte i=0;i<6;i++) mifareKey.keyByte[i]=0xFF;

  lcd.begin(16,2);
  dht.begin();

  for(uint8_t i=0;i<AVG_N;i++){ bufT[i]=25; bufRH[i]=50; }

  setupStateMachine();
  stateMachine.SetState(inicio, false, true);

  taskMonitor.Start();
}

/**
 * @brief Bucle principal del firmware.
 *
 * Actualiza las tareas asíncronas y procesa la máquina de estados en cada iteración.
 */
void loop(){
  // Actualizar tareas activas
  taskMonitor.Update();
  taskRedBlock.Update();
  taskRedHigh.Update();
  taskBlueLow.Update();
  taskRedAlarm.Update();
  taskBuzzer.Update();
  taskStayPMVHigh.Update();
  taskStayPMVLow.Update();
  taskWaitToMonitor.Update();

  if (stateMachine.GetState() != Alarma) {
    if (taskBuzzer.IsActive()) taskBuzzer.Stop();
    buzzerOff();
  }

  // FSM
  inputSignal = static_cast<Input>(readInput());
  stateMachine.Update();
}

// ==================== FSM CONFIG ====================
/**
 * @brief Configura transiciones y callbacks de la máquina de estados.
 */
void setupStateMachine(){
  // INICIO
  stateMachine.AddTransition(inicio,   Config,    [](){ return inputSignal==keypadInput; });
  stateMachine.AddTransition(inicio,   Bloqueado, [](){ return inputSignal==keypadBlock; });

  // BLOQUEO
  stateMachine.AddTransition(Bloqueado,inicio,    [](){ return inputSignal==boton; });

  // CONFIG -> MONITOR
  stateMachine.AddTransition(Config,   Monitor,   [](){ return (tag_ok && inputSignal==tiempo); });

  // MONITOR
  stateMachine.AddTransition(Monitor,  Alarma,    [](){ return (inputSignal==alarmaTemp && streak_temp_alta>=3); });
  stateMachine.AddTransition(Monitor,  pmv_alto,  [](){ return (inputSignal==pmv && pmv_actual>PMV_HIGH_TH && streak_temp_alta<3); });
  stateMachine.AddTransition(Monitor,  pmv_bajo,  [](){ return (inputSignal==pmv && pmv_actual<PMV_LOW_TH); });

  // PMV_* -> MONITOR
  stateMachine.AddTransition(pmv_alto, Monitor,   [](){ return inputSignal==tiempo; });
  stateMachine.AddTransition(pmv_bajo, Monitor,   [](){ return inputSignal==tiempo; });

  // ALARMA -> INICIO
  stateMachine.AddTransition(Alarma,   inicio,    [](){ return inputSignal==keypadInput; });

  // Callbacks
  stateMachine.SetOnEntering(inicio,      enteringInicio);
  stateMachine.SetOnLeaving (inicio,      leavingInicio);

  stateMachine.SetOnEntering(Config,      enteringConfig);
  stateMachine.SetOnLeaving (Config,      leavingConfig);

  stateMachine.SetOnEntering(Bloqueado,   enteringBloqueado);
  stateMachine.SetOnLeaving (Bloqueado,   leavingBloqueado);

  stateMachine.SetOnEntering(Alarma,      enteringAlarma);
  stateMachine.SetOnLeaving (Alarma,      leavingAlarma);

  stateMachine.SetOnEntering(Monitor,     enteringMonitor);
  stateMachine.SetOnLeaving (Monitor,     leavingMonitor);

  stateMachine.SetOnEntering(pmv_alto,    enteringPMVALTO);
  stateMachine.SetOnLeaving (pmv_alto,    leavingPmvAlto);

  stateMachine.SetOnEntering(pmv_bajo,    enteringPMVBAJO);
  stateMachine.SetOnLeaving (pmv_bajo,    leavingPmvBajo);
}

// ==================== ENTRIES / EXITS ====================
/**
 * @brief Callback al entrar en estado INICIO.
 *
 * Actualiza la pantalla LCD a "INICIO", resetea actuadores y detiene tareas
 * específicas de otros estados.
 */
void enteringInicio(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("INICIO");
  lcd.setCursor(0,1); lcd.print("Ingrese clave");
  digitalWrite(LED_RED,LOW); digitalWrite(LED_BLUE,LOW);
  digitalWrite(RELAY_PIN,LOW); buzzerOff();
  taskRedBlock.Stop(); taskRedHigh.Stop(); taskBlueLow.Stop();
  taskRedAlarm.Stop(); taskBuzzer.Stop();
  servo.write(0);
}
void leavingInicio(){}

/**
 * @brief Callback al entrar en estado BLOQUEADO.
 * Muestra mensaje y arranca tarea de parpadeo.
 */
void enteringBloqueado(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("BLOQUEO");
  lcd.setCursor(0,1); lcd.print("Pulse BOTON");
  taskRedBlock.Start();                
  buzzerOff();
}
void leavingBloqueado(){
  taskRedBlock.Stop();
  digitalWrite(LED_RED,LOW);
}

/**
 * @brief Callback al entrar en estado CONFIG.
 * Prepara la lectura de RFID y resetea flag de tag.
 */
void enteringConfig(){
  tag_ok = false;
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("CONFIG");
  lcd.setCursor(0,1); lcd.print("Acerque TAG...");
  buzzerOff();
}
void leavingConfig(){
  taskWaitToMonitor.Stop();
}

/**
 * @brief Callback al entrar en estado MONITOR.
 * Muestra mensaje e inicia ventana de reposo.
 */
void enteringMonitor(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("MONITOR");
  lcd.setCursor(0,1); lcd.print("Leyendo datos...");
  monitor_hold_until = millis() + MONITOR_HOLD_MS;
  buzzerOff();
}
void leavingMonitor(){}

/**
 * @brief Callback al entrar en estado PMV_ALTO.
 * Enciende ventilador mediante relé y arranca parpadeo.
 */
void enteringPMVALTO(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("PMV ALTO");
  lcd.setCursor(0,1); lcd.print("Ventilador ON");
  digitalWrite(RELAY_PIN, HIGH);
  taskRedHigh.Start();                 
  taskStayPMVHigh.Start();            
  buzzerOff();
}
void leavingPmvAlto(){
  taskRedHigh.Stop(); digitalWrite(LED_RED,LOW);
  digitalWrite(RELAY_PIN, LOW);
}

/**
 * @brief Callback al entrar en estado PMV_BAJO.
 * Mueve servo a 90° y arranca parpadeo del LED azul.
 */
void enteringPMVBAJO(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("PMV BAJO");
  lcd.setCursor(0,1); lcd.print("Servo ON");
  servo.write(90);
  taskBlueLow.Start();                 
  taskStayPMVLow.Start();              
  buzzerOff();
}
void leavingPmvBajo(){
  taskBlueLow.Stop(); digitalWrite(LED_BLUE,LOW);
  servo.write(0);
}

/**
 * @brief Callback al entrar en estado ALARMA.
 * Arranca parpadeo y buzzer; requiere '#'.
 */
void enteringAlarma(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("ALARMA!");
  lcd.setCursor(0,1); lcd.print("Presione #");
  taskRedAlarm.Start();               
  taskBuzzer.Start();                  
}
void leavingAlarma(){
  taskRedAlarm.Stop(); digitalWrite(LED_RED,LOW);
  taskBuzzer.Stop();   buzzerOff();
  streak_temp_alta = 0;
}

// ==================== LECTURAS Y ENTRADAS ====================
/**
 * @brief Compara dos UIDs de 4 bytes.
 * @param u Puntero al UID leído.
 * @param ref Puntero al UID de referencia.
 * @return true Si los 4 bytes coinciden.
 */
static inline bool cmpUID(byte *u, byte *ref){ for(byte i=0;i<4;i++) if(u[i]!=ref[i]) return false; return true; }

/**
 * @brief Intenta leer una tarjeta RFID y validar contra UIDs conocidos.
 *
 * Si la tarjeta es válida, escribe un marcador en EEPROM, establece tag_ok = true
 * y arranca el temporizador para pasar a MONITOR.
 */
void leerDatosRFID(){
  if(!mfrc522.PICC_IsNewCardPresent()) return;
  if(!mfrc522.PICC_ReadCardSerial())   return;

  bool ok = cmpUID(mfrc522.uid.uidByte, tarjetaUID) || cmpUID(mfrc522.uid.uidByte, llaveroUID);
  if(ok){
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("TAG correcta");
    lcd.setCursor(0,1); lcd.print("Guardando...");
    EEPROM.write(0, 0x42);
    tag_ok = true;
    taskWaitToMonitor.Start();     // esperar 5 s -> MONITOR
  }else{
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("TAG invalida");
    lcd.setCursor(0,1); lcd.print("Intente otra");
  }
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

/**
 * @brief Ejecuta un paso de monitoreo: lee sensores, actualiza medias y decide señales.
 *
 * Lecturas:
 *  - NTC -> temperatura_actual (promediado)
 *  - DHT -> humedad_actual (promediado)
 *  - LDR -> luz_actual (ADC)
 *
 * Calcula pmv_actual a través de pmv_from_table. Actualiza LCD con datos.
 * Control de transiciones: alarma por temperatura, señales PMV alto/bajo.
 */
void doMonitorStep(){
  // Lee sensores
  float Tntc = readNTC_C();
  float RH   = dht.readHumidity();
  int   L    = analogRead(LDR_PIN);

  if(!isnan(Tntc)) temperatura_actual = pushAvg(bufT, idxT, Tntc);
  if(!isnan(RH))   humedad_actual     = pushAvg(bufRH, idxRH, RH);
  luz_actual = L;

  // Calcula PMV por tabla
  pmv_actual = pmv_from_table(temperatura_actual, humedad_actual);

  // Muestra
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T:"); lcd.print(temperatura_actual,1); lcd.print((char)223); lcd.print("C ");
  lcd.print("H:"); lcd.print(humedad_actual,0); lcd.print("%");
  lcd.setCursor(0,1);
  lcd.print("L:"); lcd.print(luz_actual); lcd.print(" PMV:"); lcd.print(pmv_actual,1);

  // No decidir de inmediato si aún estamos en ventana de reposo
  if (millis() < monitor_hold_until) { inputSignal = Unknown; return; }

  // Reglas de transición a ALARMA
  if(temperatura_actual > TEMP_ALARMA_HI){
    streak_temp_alta++;
    if(streak_temp_alta >= 3){ inputSignal = alarmaTemp; return; }
  }else if(temperatura_actual < TEMP_ALARMA_LO){
    streak_temp_alta = 0;
  }

  // Reglas de PMV
  if (pmv_actual > PMV_HIGH_TH || pmv_actual < PMV_LOW_TH) inputSignal = pmv;
  else inputSignal = Unknown;
}

/**
 * @brief Lee una clave de n dígitos desde el teclado en modo bloqueante.
 * @param n Longitud de la clave (por defecto 4).
 * @return String con la clave ingresada (solo dígitos).
 */
String recibirCodigoBloqueante(uint8_t n=4){
  String r="";
  lcd.clear(); lcd.setCursor(0,0); lcd.print("INICIO");
  lcd.setCursor(0,1); lcd.print("Clave: ");
  while(r.length()<n){
    char k=keypad.getKey();
    if(k && k>='0' && k<='9'){ r+=k; lcd.print('*'); }
  }
  return r;
}

/**
 * @brief Lee las entradas relevantes según el estado actual de la FSM.
 * @return Un valor del tipo Input representando la señal detectada.
 */
int readInput(){
  State st = (State)stateMachine.GetState();

  if(st==inicio){
    String code = recibirCodigoBloqueante(4);
    return (code==clave_store)?keypadInput:keypadBlock;
  }

  if(st==Bloqueado){
    if(digitalRead(BUTTON_PIN)==LOW) return boton;
    return Unknown;
  }

  if(st==Config){
    leerDatosRFID();
    if(tag_ok && inputSignal==tiempo) return tiempo; // tras 5 s
    return Unknown;
  }

  if(st==Alarma){
    char k=keypad.getKey();
    if(k=='#') return keypadInput;
    return Unknown;
  }

  if(st==pmv_alto || st==pmv_bajo){
    // Esperan su temporizador (tiempo)
    return inputSignal;
  }

  if(st==Monitor){
    if(inputSignal==tiempo){ doMonitorStep(); return inputSignal; }
    return Unknown;
  }

  return Unknown;
}
