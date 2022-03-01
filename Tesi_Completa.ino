//Librerie

#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include <DHT_U.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>

//Dichiarazioni

#define ONE_WIRE_BUS 9
#define TdsSensorPin A1
#define VREF 5.0      // voltaggio analogico di riferimento dell'ADC(Analog-Digital Converter)
#define SCOUNT  30           // sum of sample point
#define waterLevel 49   //distanza del sensore SR04 dal ciglio dell'acqua al livello minimo - valore da trovare empiricamente in base alla posizione del sensore
#define SR04alt 75  //distanza del sensore SR04 dal suolo - valore da trovare empiricamente in base alla posizione del sensore

int fotores = A2;
int luminosita;  //valore da calcolare empiricamente in base alla posizione del sensore

DHT dht(13, DHT11);

int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;
static unsigned long previousMillis = 0; // static variable persist beyond the function call
static unsigned long prevTDSMillis = 0;

int pHpin = A0;
float correttivo = 22.14; // correttivo da calcolare ed inserire nel programma. Bisogna aggiornarlo periodicamente
float valoremedio = 0.00; // zona di transito, utilizzata nei calcoli
int tabval[50]; // tabella di memorizzazione di 10 letture consecutive.
int temp = 0; // zona di transito, utilizzata nel loop di ordinamento dei valori letti
byte indice = 0; // indice scorrimento tabella valori
byte indice1 = 0; // indice utilizzato nel loop di ordinamento dei valori letti
float sommavalori = 0; // somma dei 6 valori intermedi rilevati sul pin analogico 0
float PHmedio = 0; // PH calcolato sulla media delle tensioni rilevate sul pin 0

const int triggerPort = 12;
const int echoPort = 11;
int misurazioni[10]; //array di misurazioni livelli dell'acqua
int soglia = 1; //cm soglia oltre i quali attivare l'elettrovalvola

volatile int flow_frequency; //per misurare gli impulsi del sensore di flusso dell'acqua
unsigned int l_hour; //per calcolare i l/ora
unsigned char flowsensor = 19;
unsigned long currentTime;
unsigned long cloopTime;
int arrayPortata[10];
long effPomp[5]; // array che stipa l'efficienza persa della pompa
int flusso[14]; // array che stipa i valori di portata una volta al giorno per due settimane
int indexFlow = 0;

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
DeviceAddress insideThermometer; // arrays to hold device address

const int rs = 8, en = 7, d4 = 6, d5 = 5, d6 = 4, d7 = 3;  //init dell'lcd
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
float dati[7];

int lampada = 23;
int elettrovalvola = 22;

RTC_DS3231 rtc;

boolean avvertenze1, avvertenze2, avvertenze3, avvertenze4, avvertenze5, avvertenze6;  //warnings per l'attivazione delle segnalazioni

void flow () // Interrupt function
{
  flow_frequency++;
}

void setup() {
  Serial.begin(9600);
  if (!rtc.begin()) //condizioni per inizializzare il modulo RTC
  {
    Serial.println("verifica connessioni");
    while (true);
  }
  if (rtc.lostPower()) //la scheda ha perso l'alimentazione
  {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  //prende automaticamente data e ora dal pc quando compila
  }
  dht.begin();  //inizializzazione dht
  sensors.begin(); //inizializzazione ds18b20
  lcd.begin(16, 2);
  lcd.clear();
  pinMode(triggerPort, OUTPUT);
  pinMode(echoPort, INPUT);
  pinMode(pHpin, INPUT);
  pinMode(fotores, INPUT);
  pinMode(lampada, OUTPUT);
  pinMode(elettrovalvola, OUTPUT);
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);
  attachInterrupt(digitalPinToInterrupt(19), flow, RISING);  // Setup Interrupt
  //attachInterrupt(0, flow, RISING); // prima versione
  sei(); // consente l'attivazione di interrupts
  //currentTime = millis();
  //cloopTime = currentTime;

  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // Assign address manually. The addresses below will beed to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  // Note that you will need to use your specific address here
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };

  // Method 1:
  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC);
  Serial.println();
}


void loop() {

  //RTC
  DateTime now = rtc.now();
  Serial.print("Sono le ore: ");
  Serial.println(now.hour(), DEC);

  sensors.requestTemperatures(); // Send the command to get temperatures
  int t = tempe();
  Serial.println(t);
  int lvl = SR04alt - (int)sr04();
  
  //Array dati
  // 0 = pH, 1 = TDS, 2 = Taria, 3 = Umid, 4 = Tacqua, 5 = flusso, 6 = lvlacqua
   dati[0] = pH();
   dati[1] = TDS();
   dati[2] = t;
   dati[3] = umid();
   dati[4] = printTemperature(insideThermometer);
   dati[5] = watflow();
   dati[6] = lvl;
  //pH Sensor
  Serial.print("PH rilevato = ");
  Serial.println(dati[0]);
  //TDS
  Serial.print("TDS----Value:");
  Serial.print(dati[1], 0);
  Serial.println("ppm");
  //DHT11
  Serial.print("temperatura aria: ");
  Serial.println(dati[2]);
  //umidità
  Serial.print("umidità aria: ");
  Serial.println(dati[3]);
  //DS18B20 Sensor
  Serial.print("Temp C: ");
  Serial.println(dati[4]);
  // flow sensor
  Serial.print(dati[5], DEC); // Print litres/hour
  Serial.println(" L/hour");
  // livello acqua
  Serial.print(dati[6]); // Print litres/hour
  Serial.println(" Cm");
  //Fotoresistenza
  int f = analogRead(fotores);
  Serial.print("Luminosità: ");
  Serial.println(f);
  //SR04 Sensor
  long distanza = sr04();
  Serial.print("la distanza è:");
  Serial.println(distanza);
  
  //TDS Sensor 
  Serial.println(" ");
  Serial.println("--------");
  Serial.println(" ");
  //int a = waterMedian(waterArray());
  //Serial.println(a);
   
  //Funzioni di controllo
  
  // pompaOstruita();
  brokenPump();
  //brokenLamp();
  //brokenTank(misurazioni);
  lowHumidity();
  highHumidity();
  
  //Schermo lcd
  DisplayON(dati);

  //Relé
  lampONOFF();
  refill();
}

//------------ SUPPORT FUNCTIONS ------------

//Fotoresistenza

boolean brokenLamp() {  //ritorna true se la lampada è rotta, false se funziona
  DateTime now = rtc.now();
  int ora = (int) now.hour();
  if ((ora >= 7) && (ora < 20))  //da regolare in base al tempo di accensione del led
  {
    int foto = analogRead(fotores);
    if (foto <= luminosita)
    {
      avvertenze1 = true;
      return true;
    }
    else
    {
      avvertenze1 = false;
      return false;
    }
  }
}

//DHT11 Temperatura e Umidità aria

int tempe()
{
  int i = dht.readTemperature();
  return i;
}

int umid()
{
  int i = dht.readHumidity();
  return i;
}

boolean lowHumidity ()  //ritorna true o false se l'umidità è inferiore al 40%
{
  int umidita = umid();
  if (umidita <=40)
  {
    avvertenze5 = true;
    return true;
  }
  else
  {
    avvertenze5 = false;
    return false;
  }
}

boolean highHumidity ()  //ritorna true o false se l'umidità è superiore all'80%
{
  int umidita = umid();
  if (umidita >=80)
  {
    avvertenze6 = true;
    return true;
  }
  else
  {
    avvertenze6 = false;
    return false;
  }
}

//TDS Sensor

int getMedianNum(int bArray[], int iFilterLen)  //
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

float TDS()  // restituisce il valore in ppm rilevato dal sensore TDS
{
  float t = printTemperature(insideThermometer);
  while (analogBufferIndex != SCOUNT) {
    if (millis() - previousMillis > 40)  //ogni 40 millisecondi legge il valore analogico dall'ADC
    {
      previousMillis = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //legge il valore analogico e lo stipa in una coda
      analogBufferIndex++;
    }
    if (millis() - prevTDSMillis > 1200) //ogni 1,2 secondi fa i calcoli
    {
      prevTDSMillis = millis();
      for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * ((float)VREF / 1023.0); // legge il valore più stabile filtrato dall'algoritmo e lo converte in voltaggio
      float compensationCoefficient = 1.0 + 0.02 * (temperature - (float)t); //formula di compensazione della temperatura: fFinalResult(25^C) = fFinalResult(corrente)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge = averageVoltage * compensationCoefficient; //compensazione con la temperatura
      tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //coverte il valore di voltaggio in ppm
    }
  }
  analogBufferIndex = 0;
  return tdsValue;
}

//pH Sensor

float pH()
{
  for (indice = 0; indice < 50; indice++) // memorizza 50 letture consecutive
  {
    tabval[indice] = analogRead(A0);
    delay(60);
  }
  for (indice = 0; indice < 49; indice++) // loop di ordinamento dei valori. Ordina i valori presenti in tabval mettendoli in fila dal piu' basso al piu' alto
  {
    for (indice1 = indice + 1; indice1 < 50; indice1++)
    {
      if (tabval[indice] > tabval[indice1])
      {
        temp = tabval[indice];
        tabval[indice] = tabval[indice1];
        tabval[indice1] = temp;
      }
    }
  }
  sommavalori = 0; // azzera preventivamente la zona di somma dei valori
  for (int indice = 10; indice < 39; indice++) // nel calcolo della media considera solo i valori intermedi, dalla decima alla trentanovesima posizione in tabella tabval.
    sommavalori += tabval[indice]; // media dei sei valori intermedi memorizzati in tabval
  valoremedio = (float)sommavalori * 5.0 / 1024 / 30; // tensione media sul pin analogico 0
  return PHmedio = -5.70 * valoremedio + correttivo; // calcolo del PH, da una formula reperita in rete
}

//DS18B20 Temperatura Acqua

float printTemperature(DeviceAddress deviceAddress)  //funzione che ritorna il valore di temperatura di un dispositivo
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  return tempC;
}

void printAddress(DeviceAddress deviceAddress)  //funzione che printa l'indirizzo di un dispositivo
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

//SR04 Sensor

long sr04 ()  //ritorna la distanza calcolata dal sensore
{
  digitalWrite(triggerPort, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPort, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPort, LOW);
  long durata = pulseIn(echoPort, HIGH);
  long distanza = 0.034 * durata / 2;
  delay(1000);
  return distanza;
}

//Water Flow Sensor

/*
int watflow() { //metodo che restituisce il valore della portata dell'acqua
   currentTime = millis();
   if(currentTime >= (cloopTime + 1000))  // ogni secondo calcola la portata
   {
      cloopTime = currentTime;
      // frequenza di pulsazione (Hz) = 5.5Q, Q è la portata in litri/ora
      l_hour = (flow_frequency * 60 / 5.5); // (frequenza di pulsazione x 60 min) / 5.5Q = portata in l/ora
      flow_frequency = 0; // Reset Counter
      return l_hour;
   }
}
*/

int watflow() { //metodo che restituisce il valore della portata dell'acqua
      // frequenza di pulsazione (Hz) = 5.5Q, Q è la portata in litri/ora
      //sei();
      l_hour = (flow_frequency * 60 / 5.5); // (frequenza di pulsazione x 60 min) / 5.5Q = portata in l/ora
      flow_frequency = 0; // Reset Counter
      delay(1000);
      return l_hour;
}

int portate()
{
  for(int i=0; i<10; i++)
  {
    arrayPortata[i] = watflow();
  }
  return arrayPortata;
}

int mediaPortata(int a[])
{
  int somma = 0;
  for (int j = 0; j < 10; j++)
  {
    somma = somma + a[j];
  }
  return somma / 10;
}

boolean brokenPump()  //metodo per capire se la pompa è rotta
{
  int arr = portate();
  int flusso = mediaPortata(arr);
  if (flusso = 0)
  {
    avvertenze2 = true;
    return true;
  }
  else
  {
    avvertenze2 = false;
    return false;
  }
}

void arrayFlow()  // popola un array di valori della portata d'acqua una volta al giorno per 14 giorni. Scaduto il tempo resetta tutto
{
  //se sono le 12:00
  DateTime now = rtc.now();
  int ora = (int) now.hour();
  if (ora == 12)
  {
    flusso[indexFlow] = mediaPortata(portate());
    indexFlow++;
    if (indexFlow = 14)
    {
      for (int i = 0; i < 14; i++)
      {
        flusso[i] = -1;
      }
      indexFlow = 0;  //reset
    }
  }
}

int efficiency(int a[])  //ritorna la perdita di efficienza
{
  int minPos = 0;
  int maxPos = 0;
  for (int i = 0; i < 14; i++)
  {
    if ((a[i] != -1) && (a[i] != 0))
    {
      if (a[i] <= a[minPos])
      {
        minPos = i;
      }
      if (a[i] >= a[maxPos])
      {
        maxPos = i;
      }
    }
  }
  return a[minPos] / a[maxPos];
}

boolean pompaOstruita ()  //ritorna true o false se la pompa ha una perdita di efficienza maggiore o uguale al 5%
{
  int efficienza = efficiency(flusso);
  if (efficienza <= 0.05 && efficienza != 0)
  {
    avvertenze4 = true;
    return true;
  }
  else
  {
    avvertenze4 = false;
    return false;
  }
}

//Water level control

boolean checkLevel ()  //ritorna true o false se l'acqua dei pesci è più bassa di una certa altezza
{
  //int misu = waterArray();
  //long distanza = waterMedian(misu);
  int distanza = sr04();
  if (distanza - waterLevel >= soglia) return true;
  else return false;
}

int waterArray ()  //costruisce un array con i valori di livello dell'acqua
{
  for (int i = 0; i < 10; i++)
  {
      misurazioni[i] = sr04();
  }
  return misurazioni;
}

long waterMedian (int a[])  //calcola il livello medio dell'acqua
{
  long somma = 0;
  for (int j = 0; j < 10; j++)
  {
    somma = somma + a[j];
  }
  return somma / 10;
}

boolean brokenTank(int a[])  //ritorna true o false se il livello dell'acqua scende sotto una soglia massima in poco tempo
{
  if (a[4] - a[0] > 0.3)
  {
    avvertenze3 = true;
    return true;
  }
  else
  {
    avvertenze3 = false;
    return false;
  }
}

//Relé

void lampONOFF () { //accende o spegne la lampada
  DateTime now = rtc.now();
  int ora = (int) now.hour();
  if ((ora >= 7) && (ora < 20)) digitalWrite(lampada, HIGH);   //da regolare in base alle necessità della pianta
  else digitalWrite(lampada, LOW);
}

void refill () { //attiva o disattiva l'elettrovalvola
  bool check = checkLevel();
  if (check==true)
  {
    digitalWrite(elettrovalvola, HIGH);
  }
  else digitalWrite(elettrovalvola, LOW);
}

//Schermo LCD
void DisplayON(float data[])
// 0 = pH, 1 = TDS, 2 = Taria, 3 = Umid, 4 = Tacqua, 5 = flusso, 6 = lvlacqua
{
  if ((avvertenze1 || avvertenze2 || avvertenze3 || avvertenze4 || avvertenze5 || avvertenze6) == false)
  {
    for (int i = 0; i < 4; i++)
    {
      if (i = 1)
      {
        lcd.setCursor(4, 0);
        lcd.print("pH:");
        float a = data[0];
        lcd.print(a);
        lcd.setCursor(2, 1);
        lcd.print(" TDS:");
        lcd.print(data[1], 1);
        lcd.print("ppm");
        delay(5000);
        lcd.clear();
      }
      if (i = 2)
      {
        lcd.setCursor(3, 0);
        lcd.print("T aria:");
        lcd.print(data[2]);
        lcd.print((char)223);
        lcd.setCursor(1, 1);
        lcd.print(" Umidita:");
        lcd.print(data[3]);
        lcd.print("%");
        delay(5000);
        lcd.clear();
      }
      if (i = 3)
      {
        lcd.setCursor(2, 0);
        lcd.print("T acqua:");
        lcd.print(data[4]);
        lcd.print((char)223);
        lcd.setCursor(0, 1);
        lcd.print("litri/ora:");
        lcd.print(data[5]);
        delay(5000);
        lcd.clear();
      }
      if (i = 4)
      {
        lcd.setCursor(1, 0);
        lcd.print("Livello acqua:");
        lcd.setCursor(6, 1);
        lcd.print(data[6]);
        lcd.print("cm");
        delay(5000);
        lcd.clear();
      }
    }
  }
  else
  {
    /* if (pompaOstruita()==true)
       {
       lcd.setCursor(3, 0);
       lcd.print("Attenzione!");
       lcd.setCursor(0, 1);
       lcd.print("Pulire la pompa");
       delay(5000);
       lcd.clear();
       }
    */
    if (brokenTank(misurazioni) == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("La vasca perde");
      delay(5000);
      lcd.clear();
    }
    if (brokenLamp() == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("Il led e' rotto");
      delay(5000);
      lcd.clear();
    }
    if (brokenPump() == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("La pompa e' rotta");
      delay(5000);
      lcd.clear();
    }
    if (lowHumidity() == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("Umidita' bassa");
      delay(5000);
      lcd.clear();
    }
    if (highHumidity() == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("Umidita' alta");
      delay(5000);
      lcd.clear();
    }
  }
}
// old fx 01/03/2022
void DisplayONOld()
{
  if ((avvertenze1 || avvertenze2 || avvertenze3 || avvertenze4 || avvertenze5 || avvertenze6) == false)
  {
    for (int i = 0; i < 4; i++)
    {
      if (i = 1)
      {
        lcd.setCursor(4, 0);
        lcd.print("pH:");
        lcd.print(pH());
        lcd.setCursor(2, 1);
        lcd.print(" TDS:");
        lcd.print(TDS(), 1);
        lcd.print("ppm");
        delay(5000);
        lcd.clear();
      }
      if (i = 2)
      {
        lcd.setCursor(3, 0);
        lcd.print("T aria:");
        lcd.print(tempe());
        lcd.print((char)223);
        lcd.setCursor(1, 1);
        lcd.print(" Umidita:");
        lcd.print(umid());
        lcd.print("%");
        delay(5000);
        lcd.clear();
      }
      if (i = 3)
      {
        sensors.requestTemperatures();
        float temper = printTemperature(insideThermometer);
        lcd.setCursor(2, 0);
        lcd.print("T acqua:");
        lcd.print(temper);
        lcd.print((char)223);
        lcd.setCursor(0, 1);
        int wf = watflow();
        lcd.print("litri/ora:");
        lcd.print(wf);
        delay(5000);
        lcd.clear();
      }
      if (i = 4)
      {
        lcd.setCursor(1, 0);
        lcd.print("Livello acqua:");
        lcd.setCursor(6, 1);
        int i = SR04alt - (int)sr04();
        lcd.print(i);
        lcd.print("cm");
        delay(5000);
        lcd.clear();
      }
    }
  }
  else
  {
    /* if (pompaOstruita()==true)
       {
       lcd.setCursor(3, 0);
       lcd.print("Attenzione!");
       lcd.setCursor(0, 1);
       lcd.print("Pulire la pompa");
       delay(5000);
       lcd.clear();
       }
    */
    if (brokenTank(misurazioni) == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("La vasca perde");
      delay(5000);
      lcd.clear();
    }
    if (brokenLamp() == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("Il led e' rotto");
      delay(5000);
      lcd.clear();
    }
    if (brokenPump() == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("La pompa e' rotta");
      delay(5000);
      lcd.clear();
    }
    if (lowHumidity() == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("Umidita' bassa");
      delay(5000);
      lcd.clear();
    }
    if (highHumidity() == true)
    {
      lcd.setCursor(3, 0);
      lcd.print("Attenzione!");
      lcd.setCursor(0, 1);
      lcd.print("Umidita' alta");
      delay(5000);
      lcd.clear();
    }
  }
}
