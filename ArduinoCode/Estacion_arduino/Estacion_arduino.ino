//-------------------------------------------------------------------------------------
/*
 * Atmospheric CO2 Level..............400ppm
 * Average indoor co2.............350-450ppm
 * Maxiumum acceptable co2...........1000ppm
 * Dangerous co2 levels.............>2000ppm
 */
//-------------------------------------------------------------------------------------
/*
 *Puerto A1 luz
 *Puerto A2 Lluvia
 *puerto A3 Leer datos del LDR
 *puerto A4 Mq 9
 *puerto A5 ML5811 OUT
 *puerto A6 calidad de aire
 *puerto A8 ML5811 en
 *puerto A9 boton 
 */
//-------------------------------------------------------------------------------------
//Librerias
//------------------------------------------------------------------------------------
// se usa en I2Cdev.h
#include "Wire.h"
/*#include <SoftwareSerial.h>  probar para abrir serial 
SoftwareSerial mySerial(0, 1); para definir los puntos serial por si las*/

#include <LiquidCrystal_I2C.h>
// I2Cdev y ADXL345deben instalarse como bibliotecas, o bien los archivos .cpp/.h
// for both classes must be in the include path of your project
#include "I2Cdev.h"    //puerto y protocolo de comunicación serial
#include "ADXL345.h"   // biblioteca de acelerometro
#include "L3G4200D.h"  //bibliote de  chip capaz de aportar 3 eje aceleracion, velocidad angular y lecturas de la brújula.
#include "BMP085.h"    //biblioteca barometro
#include "HMC5883L.h"  // bibliote magnetrometo
#include <ML8511.h>
#include <HTU21D.h>
//-------------------------------------------------------------------------------------
// Set the LCD address to 0x27 for a 16 chars and 2 line display
//-------------------------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
//-------------------------------------------------------------------------------------
//Definir pines y Variables 
//-------------------------------------------------------------------------------------
#define co2Zero 55   //calibrated CO2 0 level
#define MQ9pin A4  // sensor mq9
#define ANALOGPIN A5
#define ENABLEPIN A8

HTU21D sensor;

int LDR_pin = A1;  // Leeremos el pin analogico
int LDR_var = A3;  // Variable para leer datos del LDR

const int airQualityPin = A6;  // pin de sensor de calidad de aire

int sensorPin = A2;         // El pin analógico al que está conectado el sensor de lluvia
int sensorValueLluevi = 0;  // Variable para almacenar el valor leído del sensor

float sensorValue2;  //variable para guardar el valor analógico del sensor mq 9
int tiempo = 0;
int cnt = 0;
float v1 = 0;
float v2 = 0;
// acelerar
ADXL345 acelerar;
int16_t ax, ay, az;

// giroscopio
L3G4200D giroscopio;
int16_t avx, avy, avz;

// barometro
BMP085 barometro;
float temperature;
float pressure;
float altitude;
int32_t lastMicros;

// magneto
HMC5883L mag;
int16_t mx, my, mz;

bool blinkState = false;

int button = A9; //Pin para boton para manipular pantalla LCD

int contador = 1;

ML8511 light(ANALOGPIN, ENABLEPIN);
//-------------------------------------------------------------------------------------
void setup() {
  //-------------------------------------------------------------------------------------
  Wire.begin();
  sensor.begin();
  //-------------------------------------------------------------------------------------
  // inicializa la comunicación serial
  Serial.begin(9600);
  //mySerial.begin(9600); configurar punto serial 
  //-------------------------------------------------------------------------------------
  pinMode(airQualityPin, INPUT);  //MQ135 conjunto de alimentación analógica para entrada
  //-------------------------------------------------------------------------------------
  // inicializar dispositivo
  Serial.println("Iniciando Estacion.....");
  acelerar.initialize();
  giroscopio.initialize();
  barometro.initialize();
  mag.initialize();
  //-------------------------------------------------------------------------------------
  //establecer escala completa de giroscopio
  giroscopio.setFullScale(2000);
  //-------------------------------------------------------------------------------------
  //boton
  pinMode(button, INPUT);
  analogReference(1.1);  // pone como referencia interna 1.1V
  //-------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------
void loop() {
  //-------------------------------------------------------------------------------------
  lcd.init();
  /*
  int serial = digitalRead(10);
  mySerial.println(setData);

  punto de envio 
  */
  //-------------------------------------------------------------------------------------
  //Switch para cambiar lo que muestra la pantalla LCD
  //-------------------------------------------------------------------------------------
  bool e = digitalRead(button);
  if (contador == 9) {
    contador = contador - 9;
  }
  if (e == 1) {

    contador++;
  } else {
  }
  datos();
  Pantalla(contador);
}
//-------------------------------------------------------------------------------------
void datos() {  
  //-------------------------------------------------------------------------------------
  //Datos de la SERIAL
  //-------------------------------------------------------------------------------------
  Serial.print(" LDR =, ");
  Serial.print(luz());
  Serial.print(",");
  //-------------------------------------------------------------------------------------
  if (sensor.measure()) {

    Serial.print("Temperature (°C):, ");
    Serial.print(Temperatura());
    Serial.print(",");
    Serial.print("Humidity (%RH): ,");
    Serial.print(humedad());
  }
  //-------------------------------------------------------------------------------------
  Serial.print(",AirQuality=,");
  Serial.print(co2());  //imprime el valor leído
  Serial.print(" ,PPM,");
  //-------------------------------------------------------------------------------------
  Serial.print("Valor del sensor de lluevia: ,");
  Serial.print(lluvia());  // Imprimir el valor leído en el monitor
  Serial.print(",");
  //-------------------------------------------------------------------------------------
  Serial.print("Precion :,");
  Serial.print(barometer());
  Serial.print(",Pm,");
  //-------------------------------------------------------------------------------------
  Serial.print("Valor detectado por el sensor:, ");
  Serial.print(mq9());
  Serial.print(" ,ppm,");
  //-------------------------------------------------------------------------------------
  Serial.print(ML5811());
  Serial.print(" ,mW cm^2,");

  if (ML5811() < 2.80) {
    Serial.println(" Bajo,");
  }
  if ((ML5811() > 2.8) && (ML5811() < 5.7)) {
    Serial.println(" Moderando,");
  }
  if ((ML5811() >= 5.7) && (ML5811() < 7.7)) {
    Serial.println(" Alto,");
  }
  if (ML5811() > 7.7) {
    Serial.println("Extremadamente alto,");
  }
  //-------------------------------------------------------------------------------------

  String setData = "Temperatura=" + String(Temperatura()) + "&Humedad=" + String(humedad()) + "&Intensidad_Luz=" + String(luz()) + "&Presipitacion=" + String(lluvia()) + "&AIR_QUALITY=" + String(co2()) + "&Gases_toxicos=" + String(mq9()) + "&Luz_UV=" + String(ML5811()) + "&Presion_Atmosferica=" + String(barometer());

  //-------------------------------------------------------------------------------------
  delay(0);
  //-------------------------------------------------------------------------------------
  Serial.write(0x0d);
  //-------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------
float luz() {
//-------------------------------------------------------------------------------------
// Se leeran los valores contimuamnete del sensor LDR
//-------------------------------------------------------------------------------------

  LDR_var = analogRead(LDR_pin);
  //-------------------------------------------------------------------------------------
  return LDR_var;
  //-------------------------------------------------------------------------------------
  delay(1000);
  // retardo de 1 segundo
}
//-------------------------------------------------------------------------------------
//Leer datos de senor HTU21D
//-------------------------------------------------------------------------------------
float Temperatura() {
  //-------------------------------------------------------------------------------------
  float temperature = sensor.getTemperature();
  return temperature;
  //-------------------------------------------------------------------------------------
  delay(1000);
}
float humedad(){
  float humidity = sensor.getHumidity();
  return humidity;
  //-------------------------------------------------------------------------------------
  delay(1000);
}
//-------------------------------------------------------------------------------------
//Leer datos de sensor MQ-135
//-------------------------------------------------------------------------------------
float co2() {
  //-------------------------------------------------------------------------------------
  int co2now[10];    //int matriz para lecturas de co2
  int co2raw = 0;    //int para el valor bruto de co2
  int co2ppm = 0;    //int para ppm calculado
  int promedio = 0;  //int para promediar

  for (int x = 0; x < 10; x++)  //muestra co2 10x durante 2 segundos
  {
    co2now[x] = analogRead(A6);
    delay(00);
  }

  for (int x = 0; x < 10; x++)  //agregar muestras juntas
  {
    promedio = promedio + co2now[x];
  }

  co2raw = promedio / 10;     //dividir muestras por 10
  co2ppm = co2raw - co2Zero;  //obtener ppm calculado
  //-------------------------------------------------------------------------------------
  return co2ppm;
  //-------------------------------------------------------------------------------------
  delay(1000);
}
//-------------------------------------------------------------------------------------
//Leer datos de sensor FC-37
//-------------------------------------------------------------------------------------
float lluvia() {
  //-------------------------------------------------------------------------------------

  sensorValueLluevi = analogRead(sensorPin);  // Leer el valor del sensor
  return sensorValueLluevi;
  //-------------------------------------------------------------------------------------
  delay(1000);  // Esperar un segundo antes de leer el valor del sensor nuevamente
}
//-------------------------------------------------------------------------------------
//Leer datos de MÓDULO MULTISENSOR 10 DOF GY-801
//-------------------------------------------------------------------------------------
void accelerometer() {
  //-------------------------------------------------------------------------------------
  // leer medidas de aceleración sin procesar desde el dispositivo
  acelerar.getAcceleration(&ax, &ay, &az);
  //-------------------------------------------------------------------------------------
  // mostrar valores de aceleración x/y/z separados por tabuladores
  Serial.print("aceleracion en eje (X-Y-Z):\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.println(az);
  Serial.println(" ");
  //-------------------------------------------------------------------------------------
  delay(1000);
}
//-------------------------------------------------------------------------------------
//Leer datos de MÓDULO MULTISENSOR 10 DOF GY-801
//-------------------------------------------------------------------------------------
void gyroscope() {
  //-------------------------------------------------------------------------------------
  giroscopio.getAngularVelocity(&avx, &avy, &avz);
  //-------------------------------------------------------------------------------------
  Serial.print("velocidad angular en eje (X-Y-Z) :\t");
  Serial.print(avx);
  Serial.print("\t");
  Serial.print(avy);
  Serial.print("\t");
  Serial.println(avz);
  Serial.println(" ");
  //-------------------------------------------------------------------------------------
  delay(1000);
}
//-------------------------------------------------------------------------------------
//Leer datos de MÓDULO MULTISENSOR 10 DOF GY-801
//-------------------------------------------------------------------------------------
float barometer() {
  //-------------------------------------------------------------------------------------
  // solicitar temperatura
  barometro.setControl(BMP085_MODE_TEMPERATURE);

  // espere el tiempo apropiado para la conversión (retraso de 4.5ms)
  lastMicros = micros();
  while (micros() - lastMicros < barometro.getMeasureDelayMicroseconds());

  //leer el valor de temperatura calibrado en grados Celsius
  temperature = barometro.getTemperatureC();

  // presión de solicitud (modo de sobremuestreo 3x, alto detalle, retraso de 23,5 ms)
  barometro.setControl(BMP085_MODE_PRESSURE_3);
  while (micros() - lastMicros < barometro.getMeasureDelayMicroseconds());

  // leer el valor de presión calibrado en Pascales (Pa)
  pressure = barometro.getPressure();

  // calcular la altitud absoluta en metros en función de la presión conocida
  // (puede pasar un segundo parámetro de "presión a nivel del mar" aquí,
  // de lo contrario usa el valor estándar de 101325 Pa)
  altitude = barometro.getAltitude(pressure);
  //-------------------------------------------------------------------------------------
  return pressure;
  //-------------------------------------------------------------------------------------
  delay(1000);
}
//-------------------------------------------------------------------------------------
//Leer datos de MÓDULO MULTISENSOR 10 DOF GY-801
//-------------------------------------------------------------------------------------
void magneto() {
  //-------------------------------------------------------------------------------------
  // leer medidas de rumbo sin procesar desde el dispositivo
  mag.getHeading(&mx, &my, &mz);
  //-------------------------------------------------------------------------------------
  // mostrar valores de giroscopio x/y/z separados por tabulaciones
  Serial.print("mag en (X-Y-Z):\t");
  Serial.print(mx);
  Serial.print("\t");
  Serial.print(my);
  Serial.print("\t");
  Serial.print(mz);
  Serial.println("\t");
  Serial.println("");

  // Para calcular el rumbo en grados. 0 grados indica el norte
  float heading = atan2(my, mx);
  if (heading < 0)
    heading += 2 * M_PI;
  Serial.print("heading:\t");
  Serial.println(heading * 180 / M_PI);
  //-------------------------------------------------------------------------------------
  delay(1000);
}
//-------------------------------------------------------------------------------------
//Leer datos de sensor mq-9
//-------------------------------------------------------------------------------------
float mq9() {
  //-------------------------------------------------------------------------------------
  sensorValue2 = analogRead(MQ9pin);  // lectura analogica pin
  //-------------------------------------------------------------------------------------
  return sensorValue2;
  //-------------------------------------------------------------------------------------
  delay(1000);
}
//-------------------------------------------------------------------------------------
//Leer datos de sensor GY-ML8511 
//-------------------------------------------------------------------------------------
float ML5811() {
  //-------------------------------------------------------------------------------------
  light.enable();
  float UV = light.getUV();
  light.disable();
  //-------------------------------------------------------------------------------------
  // Turn on the blacklight
  return UV,4;
  //-------------------------------------------------------------------------------------
  delay(1000);
}
//-------------------------------------------------------------------------------------
void Pantalla(int x){
  // Turn on the blacklight
  lcd.backlight();
  lcd.clear();

  switch(x){
    case 1:
      // First row
      lcd.setCursor(0, 0);
      lcd.print(">>> TOMANDO <<<");

      // Second row
      lcd.setCursor(0, 1);
      lcd.print(">>>  DATOS  <<<");
      delay(1000);
    break;
    case 2:
      // First row
      lcd.setCursor(0, 0);
      lcd.print(">>>   LCD   <<<");

      // Second row
      lcd.setCursor(0, 1);
      lcd.print("      ");
      lcd.print(luz());

    break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("T=   ");
      lcd.print(Temperatura());
      lcd.print(" (°C)");


      lcd.setCursor(0, 1);

      lcd.print("RH=  ");
      lcd.print(humedad());
      lcd.print(" (%RH)");

    break;
    case 4:
      // First row
      lcd.setCursor(0, 0);
      lcd.print(">> AirQuality <<");

      // Second row
      lcd.setCursor(0, 1);
      lcd.print("   ");
      lcd.print(co2());
      lcd.print(" ppm");

    break;
    case 5:
      // First row
      lcd.setCursor(0, 0);
      lcd.print("LLUVIA: ");
      // Second row
      lcd.setCursor(0, 1);
      lcd.print(lluvia());

    break;
    case 6:
      // First row
      lcd.setCursor(0, 0);
      lcd.print("Precion Atmos");

      // Second row
      lcd.setCursor(0, 1);
      lcd.print("  ");
      lcd.print(barometer());
      lcd.print(" PC");

    break;
    case 7:
      // First row
      lcd.setCursor(0, 0);
      lcd.print("MQ-9: ");
      lcd.print(mq9());
      lcd.print(" ppm");

      // Second row
      lcd.setCursor(0, 1);
      if (mq9() > 300) {
        lcd.print("GAS detectado !!");
      }

    break;
    case 8:
      lcd.setCursor(0, 0);
      lcd.print(" mW/cm^2: ");
      lcd.print(ML5811());

      // Second row
      lcd.setCursor(0, 1);

      if (ML5811() < 2.80) {
        lcd.print("   Bajo");
      }
      if ((ML5811() > 2.8) && (ML5811() < 5.7)) {
        lcd.print(" Moderando");
      }
      if ((ML5811() >= 5.7) && (ML5811() < 7.7)) {
        lcd.print(" Alto");
      }
      if (ML5811() > 7.7) {
        lcd.print("Demaciado alto");
      }
    break;
    case 9:
      // First row
      lcd.setCursor(0, 0);
      lcd.print(">>>   OFF   <<<");

      // Second row
      lcd.setCursor(0, 1);
      lcd.print("   0  ---  0   ");
      delay(1000);

      break;
    break;
    default:
      break;
  }
}