/**
\file   sketch.ino
\date   2023-12-05
author Adrián Fernando Delgado <adriandelgado@unicauca.edu.co>, Juliana Dazza Pineda <julianad@unicauca.edu.co>, Juan David Burgos Arturo <juandavidburgos@unicauca.edu.co>, Juan Esteban Muñoz Gomez <juanestebanmunoz@unicauca.edu.co>
@brief  Invernadero inteligente Tercer corte

\par Copyright
La información contenida aquí es propiedad de los fines
comerciales confidenciales de la Universidad del Cauca, y
está sujeto a restricciones de uso y divulgación.

\par
Copyright (c) Universidad del Cauca 2023. Todos los derechos reservados
*/


#include "DHT.h"               
#include <LiquidCrystal.h>     
#include <Keypad.h>            
#include <AverageValue.h>       
#include "StateMachineLib.h"    
#include "AsyncTaskLib.h"   

/**
* @brief Asignación de Pines para componentes y sensores.
* @param DHTPIN Definición para el pin conectado al sensor DHT
* @param BUZZER Definición para el pin conectado al buzzer
* @param DHTTYPE Definición para el tipo de sensor DHT usado (DHT11 o DHT22)
* @param PHOTOCELL_PIN Definición para el pin conectado al sensor de la fotocelda
*/
#define DHTPIN A5  
#define BUZZER 13    
#define DHTTYPE DHT11          
#define PHOTOCELL_PIN A0   

/** @brief Objeto LCD con configuración de pines.
*
*/
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

/**
* @brief Asignación de Pines para el LED RGB.
* @param redPin Pin para el componente rojo del LED
* @param greenPin Pin para el componente verde del LED
* @param bluePine Pin para el componente azul del LED
*/
const int redPin = 8;  
const int greenPin = 7; 
const int bluePin = 6;

/**
* @brief Variables relacionadas al fotorresistor
* @param gammaValue Valor de gamma para los cálculos relacionados
* @param resistanceRL10Value Valor de la resistencia para los cálculos de la fotocelda
*/
const float gammaValue = 0.7;  
const float resistanceRL10Value = 50;  

/** @brief Variable para guardar el tiempo de inicio. */
unsigned int beginingTime = 0;

/** @brief Variable para guardar el tiempo actual */
unsigned int currentTime = 0;

/** @brief Variable para guardar la intensidad de la luz */
short int light = 0.0;

/**
* @brief Asignación de Pines para el Keypad
* @param password Array que guarda la contraseña
* @param passwordNumberArray Array que guarda los digitos ingresados de la contraseña
* @param counterPasswordNumbers Contador para los digitos ingresados de la contraseña
* @param counterPasswordMistakes Contador para los digitos erróneamente ingresados de la contraseña
* @param counterPasswordCorrects Contador para los digitos correctos ingresados de la contraseña
*/
char password[] = {'A', 'B', 'C', 'D'}; 
int passwordNumbersArray[4];                 
int counterPasswordNumbers = 0;                    
int counterPasswordMistakes = 0;            
int counterPasswordCorrects = 0;            

/**
* @brief Configuración del Keypad.
*/
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {24, 26, 28, 30}; 
byte colPins[COLS] = {32, 34, 36, 38};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

/**
 * @brief Seteo del color del LED RGB
 * @param red Valor para el componente rojo.
 * @param green Valor para el componente verde.
 * @param blue Valor para el componente azul.
 */
void color(unsigned char red, unsigned char green, unsigned char blue) {
    analogWrite(redPin, red);
    analogWrite(bluePin, blue);
    analogWrite(greenPin, green);
}

/**
* @brief Creación de una instancia de StateMachine con la configuración de los pines.
*/
StateMachine stateMachine(5, 8);
int trackCondition = 0;      /**< Variable para el trackeo de una condición. */

/**
 * @brief Variables input para los estados
 * @param InInit Input para indicar la inicialización.
 * @param InBlocked Input para indicar el estado de bloqueo.
 * @param InMonitorTH Input para el monitoreo de la temperatura y la humedad.
 * @param InAlarm Input para indicar la condición de alarma.
 * @param InLight Input para el monitoreo de la intensidad de la luz.
 * @param TimeOut5 Input para el timeout de 5 segundos.
 * @param TimeOut6 Input para el timeout de 6 segundos.
 * @param TimeOut7 Input para el timeout de 7 segundos.
*/
enum Input
{
    InInit = 0,     
    InBlocked = 1,   
    InMonitorTH = 2,  
    InAlarm = 3,  
    InLight = 4,  
    TimeOut5 = 5,  
    TimeOut6 = 6,      
    TimeOut7 = 7   
};

Input input;  /** Variable que guarda el último input digitado por el usuario. */

/**
 * @brief Enumeración de los diferentes estados para la StateMachine.
 * @param initialState Estado inicial de la máquina.
 * @param blockedState Estado indicando la condición de bloqueo de la máquina.
 * @param monitorTHState Estado para el monitoreo de la temperatura y la humedad.
 * @param alarmState Estado para indicar la condición de la alarma.
 * @param monitorLightState Estado para el monitoreo de la intensidad de la luz.
 */
enum State {
    initialState = 0,   
    blockedState = 1,    
    monitorTHState = 2,   
    alarmState = 3,   
    monitorLightState = 4, 
};

/**
 * @brief Creación y agrupación por tareas
 * @param timeOutTask5 Tarea para el manejo del timeout de 5 segundos
 * @param timeOutTask3 Tarea para el manejo del timeout de 3 segundos
 * @param timeOutTask6 Tarea para el manejo del timeout de 6 segundos
 * @param initTask Tarea para la inicialización
 * @param monitorTHTask Tarea para el monitoreo de la temperatura y la humedad
 * @param monitorLightTask Tarea para el monitoreo de la intensidad de la luz
 * @param alarmTask Tarea para el estado de alarma
 */
AsyncTask timeOutTask5(5000, false, []() { fnTimeOut5(); });  
AsyncTask timeOutTask3(3000, false, []() { fnTimeOut6(); });  
AsyncTask timeOutTask6(6000, false, []() { fnTimeOut7(); });  
AsyncTask initTask(50, true, []() { fnInit(); });            
AsyncTask monitorTHTask(500, true, []() { fnMonitorTH(); });  
AsyncTask monitorLightTask(200, true, []() { fnMonitorLuz(); });
AsyncTask alarmTask(50, true, []() { fnAlarma(); });         

/**
 * @brief Función para el manejo del timeout de 5 segundos
 */
void fnTimeOut5()
{
    input = TimeOut5;    /**< Asigna el timeout de 5 segundos como el último input de usuario. */
    trackCondition = 0;   /**< Resetea la condición. */
}

/**
 * @brief Función para el manejo del timeout de 6 segundos
 */
void fnTimeOut6()
{
    input = TimeOut6;    /**< Asigna el timeout de 6 segundos como el último input de usuario. */
    trackCondition = 0;   /**< Resetea la condición. */
}

/**
 * @brief Función para el manejo del timeout de 7 segundos
 */
void fnTimeOut7()
{
    input = TimeOut7;    /**< Asigna el timeout de 7 segundos como el último input de usuario. */
    trackCondition = 0;   /**< Resetea la condición. */
}

/**
 * @brief Función para iniciar la tarea de inicialización
 */
void inputInit()
{
    initTask.Start();
}

/**
 * @brief Función para detener la tarea de inicialización
 */
void outputInit()
{
    initTask.Stop();
}

/**
 * @brief Función para iniciar la tarea de monitoreo de humedad y temperatura
 */
void inputMonitorTH()
{
    monitorTHTask.Start();
}

/**
 * @brief Función para detener la tarea de monitoreo de humedad y temperatura
 */
void outputMonitorTH()
{
    monitorTHTask.Stop();
}

/**
 * @brief Función para iniciar la tarea de monitoreo de luz
 */
void inputMonitorLight()
{
    monitorTHTask.Stop();
    monitorLightTask.Start();
}

/**
 * @brief Función para detener la tarea de monitoreo de luz
 */
void outputMonitorLight()
{
    monitorLightTask.Stop();
}

/**
 * @brief Función para iniciar la tarea de la alarma
 */
void inputAlarm()
{
    alarmTask.Start();
}

/**
 * @brief Función para detener la tarea de la alarma
 */
void outputAlarm()
{
    alarmTask.Stop();
}

/* Funciones para la contraseña */

/**
 * @brief Función para cuando la contraseña digitada es correcta
 */
void accessGranted(){
    lcd.clear();
    color(0, 255, 0);
    lcd.print("--Identidad Reconocida--");
    delay(800);
    lcd.clear();
    color(0, 0, 0);
    input = InMonitorTH;
}

/**
 * @brief Función para cuando la contraseña digitada es incorrecta
 */
void accessDenied(){
    lcd.clear();
    if(counterPasswordMistakes < 2){
        color(0, 0, 255);
        lcd.print("Esa no es pelao");
        delay(800);
        counterPasswordMistakes++;
        lcd.clear();
        lcd.print("PIN: ");
    }
    else{
        counterPasswordCorrects = -1;
        input = InBlocked;
    }
}

/**
 * @brief Función para inicializar la entrada al sistema
 */
void fnInit()
{
    lcd.clear();
    counterPasswordNumbers = 0;
    counterPasswordCorrects = 0;
    counterPasswordMistakes = 0;
    lcd.print("PIN: ");
    do {
        color(0, 0, 0);
        char key = keypad.getKey();
        if (key) {
            lcd.print('*');
            passwordNumbersArray[counterPasswordNumbers] = key;
            counterPasswordNumbers++;
            delay(100);
        }
        if (counterPasswordNumbers == 4) {
            counterPasswordCorrects = 0;
            for (int i = 0; i < counterPasswordNumbers; i++) {
                if (password[i] == passwordNumbersArray[i]) {
                    counterPasswordCorrects++;
                }
            }
            if (counterPasswordCorrects == counterPasswordNumbers) {
                accessGranted();
                counterPasswordNumbers = 0;
            } else {
                accessDenied();
                counterPasswordNumbers = 0;
            }
        }
    } while ((counterPasswordCorrects != -1) && (counterPasswordCorrects != 4));
}

/**
 * @brief Función para cuando el input se bloquea
 */
void inputBlocked()
{
    color(255, 0, 0);
    lcd.print("BLOQUEO INMINENTE");
    timeOutTask5.SetIntervalMillis(5000);
    timeOutTask5.Start();
}

/**
 * @brief Función para el manejo de la alarma
 */
void fnAlarma() {
    while (trackCondition == 0)
    {
        lcd.clear();
        color(255, 0, 0);
        lcd.print("ALERTA ATENCION");
        tone(BUZZER, 1000, 5000);
        timeOutTask6.Start();
        trackCondition++;
    }
}

/**
 * @brief Seteo de la StateMachine con acciones y transiciones.
 */
void setupStateMachine()
{
    /* Transiciones */
    stateMachine.AddTransition(initialState, blockedState, []() { return input == InBlocked; });
    stateMachine.AddTransition(initialState, monitorTHState, []() { return input == InMonitorTH; });
    stateMachine.AddTransition(blockedState, initialState, []() { return input == TimeOut5; });
    stateMachine.AddTransition(monitorTHState, alarmState, []() { return input == InAlarm; });
    stateMachine.AddTransition(monitorTHState, monitorLightState, []() { return input == TimeOut5; });
    stateMachine.AddTransition(alarmState, monitorTHState, []() { return input == TimeOut7; });
    stateMachine.AddTransition(monitorLightState, alarmState, []() { return input == InAlarm; });
    stateMachine.AddTransition(monitorLightState, monitorTHState, []() { return input == TimeOut6; });

    /* Acciones */
    stateMachine.SetOnEntering(initialState, inputInit);
    stateMachine.SetOnEntering(blockedState, inputBlocked);
    stateMachine.SetOnEntering(monitorTHState, inputMonitorTH);
    stateMachine.SetOnEntering(alarmState, inputAlarm);
    stateMachine.SetOnEntering(monitorLightState, inputMonitorLight);
    stateMachine.SetOnLeaving(initialState, []() { outputInit(); });
    stateMachine.SetOnLeaving(monitorTHState, []() { outputMonitorTH(); });
    stateMachine.SetOnLeaving(alarmState, []() { outputAlarm(); });
    stateMachine.SetOnLeaving(monitorLightState, []() { outputMonitorLight(); });
}

/**
 * @brief 
 * @param dht Inicialización del sensor DHT11
 * @param MAX_VALUES_NUM Número máximo de valores para el promedio
 * @param averageTemperatureValue Valor promedio para la temperatura
 * @param averageHumidityValue Valor promedio para la humedad
 * @param cleanAverageValue Valor promedio para la limpieza de datos
 */
DHT dht(DHTPIN, DHTTYPE); 
const long MAX_VALUES_NUM = 5;
AverageValue<long> averageTemperatureValue(MAX_VALUES_NUM); 
AverageValue<long> averageHumidityValue(MAX_VALUES_NUM); 
AverageValue<long> cleanAverageValue(MAX_VALUES_NUM); 

/**
 * @brief Función para el monitoreo de la temperatura y la humedad
 */
void fnMonitorTH()
{
    noTone(BUZZER);
    color(0, 0, 0);
    beginingTime = millis();
    lcd.clear();
    counterPasswordCorrects = 0;
    averageTemperatureValue = cleanAverageValue;
    averageHumidityValue = cleanAverageValue;
    counterPasswordNumbers = 0;
    dht.begin();
    while (trackCondition == 0)
    {
        float h = dht.readHumidity();
        float t = dht.readTemperature();
        float f = dht.readTemperature(true);

        if (isnan(h) || isnan(t) || isnan(f)) {
            Serial.println(F("Failed to read from DHT sensor!"));
            return;
        }

        float hif = dht.computeHeatIndex(f, h);
        float hic = dht.computeHeatIndex(t, h, false);

        lcd.setCursor(0, 0);
        lcd.print("Humedad: ");
        lcd.print(h);
        lcd.print("%");

        lcd.setCursor(0, 1);
        lcd.print("Tempe: ");
        lcd.print(t);
        lcd.print(" C");
        delay(900);
        averageTemperatureValue.push(t);
        averageHumidityValue.push(h);
        counterPasswordNumbers++;
        if (counterPasswordNumbers >= 5)
        {
            if (averageTemperatureValue.average() > 30 && averageHumidityValue.average() > 70)
            {
                input = InAlarm;
                counterPasswordCorrects++;
                break;
            }
            else
            {
                currentTime = millis();
                counterPasswordCorrects++;
                timeOutTask5.SetIntervalMillis(5000 - (currentTime - beginingTime));
                timeOutTask5.Start();
                trackCondition++;
            }
            counterPasswordNumbers = 0;
        }
    }
}

/* Valor promedio para la intensidad de la luz */
AverageValue<long> averageLightValue(MAX_VALUES_NUM);

/**
 * @brief Función para la luz del fotorresistor
 */
void fnMonitorLuz()
{   
    beginingTime = millis();
    lcd.clear();
    counterPasswordNumbers = 0;
    averageLightValue = cleanAverageValue;
    while (trackCondition == 0)
    {
        lcd.setCursor(0, 0);
        int analogValue = analogRead(PHOTOCELL_PIN);
        float voltage = analogValue / 1024.0 * 5.0;
        float resistance = 2000.0 * voltage / (1.0 - voltage / 5.0);
        float light = pow(resistanceRL10Value * 1e3 * pow(10, gammaValue) / resistance, (1.0 / gammaValue));
        lcd.print("Luz: ");
        lcd.print(analogValue);
        averageLightValue.push(light);
        delay(500); 
        counterPasswordNumbers++;
        if (counterPasswordNumbers >= 5)
        {
            if (analogValue < 20.0) {
                input = InAlarm;
                break;
            } else {
                currentTime = millis();
                timeOutTask3.SetIntervalMillis(3000 - (currentTime - beginingTime));
                timeOutTask3.Start();
                trackCondition++;
            }
        }
    }
}

/**
 * @brief Función que se llama 1 vez al iniciar el programa
 */
void setup() 
{
    /* Seteo del LCD*/
    lcd.begin(16, 2);
    
    /* Seteo del LED RGB */
    pinMode(redPin, OUTPUT); 
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    pinMode(BUZZER, OUTPUT); 

    /* Seteo de la StateMachine */
    Serial.begin(9600);
    setupStateMachine();	

    /* Estado inicial */
    stateMachine.SetState(initialState, false, true);
}

/**
 * @brief Ejecución principal en ciclo del programa principal
 */
void loop() 
{
    monitorLightTask.Update();
    timeOutTask5.Update();
    timeOutTask3.Update();
    timeOutTask6.Update();
    alarmTask.Update();
    initTask.Update();
    monitorTHTask.Update();
    stateMachine.Update();
}