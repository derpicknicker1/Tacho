#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
// ###################################################
// #               RAD-EINSTELLUNGEN                 #
// ###################################################
#define Umfang                    2225                    // RadUmfang in mm
#define Impulse                   28                      // Impulse vom Dynamo pro Umdrehung (28 Pole => 14 Perioden pro Umdrehung => 28 Flankenwechsel)
// ###################################################
// #             HARDWARE-EINSTELLUNGEN              #
// ###################################################
#define BOARD                     1                       // Hardware definieren (0 = Arduino Nano, 1 = Arduino Micro)
// ###################################################
// #             SPEICHER-EINSTELLUNGEN              #
// #     ACHTUNG: nur wenn du weißt was du tust      #
// ###################################################
#define MaxSpeicherabstand        60000                   // Maximaler Zeitabstand in Millisek. zwischen zwei Speichervorgängen
#define MinSpeicherabstand        5000                    // Mindest-Zeitabstand in Millisek. zwischen zwei Speichervorgängen
#define SpeicherabstandReserve    120000                  // Mindest-Zeitabstand in Millisek. zwischen zwei Reserve-Speichervorgängen
#define Bereiche                  7                       // Anzahl der Geschwindigkeitsbereiche
#define RotSize                   8                       // Größe des Rotationsspeichers in Byte. (Größer=>weniger Abnutzung, aber weniger Platz für Merker)
#define MaxKmh                    30                      // Max. Geschwindigkeit der Geschwindigkeitsbereiche. Sollte am besten durch 'Bereiche' teilbar sein.
#define KmhSicher                 10                      // Min. Geschwindigkeit um sicher in den Reserve-Speicher zu schreiben
// ###################################################
// #                 !!!ACHTUNG!!!                   #
// #             AB HIER NICHTS ÄNDERN               #
// ###################################################
// ###################################################
// #             HARDWARE-EINSTELLUNGEN              #
// ###################################################
#define ClockPin                  2                       // Eingangspin vom Dynamo (2 = INT0 [Nano] = INT1 [Micro])
#if BOARD == 0 // NANO (hat nur eine LED. Wird für beide Funktionen genutzt)
  #define IntNum                  0                       // Nummer des externen Interrupts (0 = INT0 = Nano, 1 = INT1 = Micro)
  #define LED1Pin                 5                       // Pin für LED1 (zeigt Geschwindigkeits-Update an (1Hz), 5 = Micro/Nano)
  #define LED1Port                PORTB                   // Port für LED1 (PORTB = Nano, PORTD = Micro)
  #define EESize                  1024                    // Größe des EEPROM in Byte
#elif BOARD == 1 //MICRO
  #define IntNum                  1                       // Nummer des externen Interrupts (0 = INT0 = Nano, 1 = INT1 = Micro)
  #define LED1Pin                 5                       // Pin für LED1 (zeigt Geschwindigkeits-Update an (1Hz), 5 = Micro/Nano)
  #define LED2Pin                 0                       // Pin für LED2 (zeigt das Speichern der aktuellen Werte an, 5 = Nano, 0 = Micro)
  #define LED1Port                PORTD                   // Port für LED1 (PORTB = Nano, PORTD = Micro)
  #define LED2Port                PORTB                   // Port für LED2 (PORTB = Nano/Micro)
  #define EESize                  1024                    // Größe des EEPROM in Byte
#else
  #error "FEHLER: Ungueltige Board ID in den Hardware-Einstellungen"
#endif
// ###################################################
// #               SPEICHER-BEREICHE                 #
// ###################################################
#if (Bereiche * RotSize * 3 + 1) > EESize
  #error "FEHLER: Nicht genügend Speicher fuer Kilometerstaende. In den Einstellungen 'Bereiche' und 'RotSize' einstellen."
#endif
#define ErsterSpeicher            (Bereiche * RotSize * 0 + 1)  // Start-Adresse Normaler Eeprom Speicherbereich (56 Byte groß) (Nie bei 0 anfangen!!)
#define ZweiterSpeicher           (Bereiche * RotSize * 1 + 1)  // Start-Adresse Zweiter  Eeprom Speicherbereich (56 Byte groß) als Reserve
#define ReserveSpeicher           (Bereiche * RotSize * 2 + 1)  // Start-Adresse Reserve Eeprom Speicherbereich (56 Byte groß), der nur bei genügend Spannung schreiben
#define MerkerSpeicher            (Bereiche * RotSize * 3 + 1)  // Start-Adresse MerkerSpeicher zum Ablegen der Fahrradwartungsinformationen
#define MerkerSize                (15 + Bereiche * 3)     // Größe eines Merkers in Byte (15Byte für Name-String, Bereiche * 3Byte für Werte)
#define MerkerMax                 ((EESize-MerkerSpeicher) / MerkerSize)
#if MerkerMax <= 0
  #warning "WARNUNG: Kein Platz für Merker-Speicher. Funktion deaktiviert."
#endif
// ###################################################
// #                 HELFER-MAKROS                   #
// ###################################################
#define ON                        false                   // für LED an
#define OFF                       true                    // für LED aus
#define MMProI                    ((float)Umfang / Impulse)                                   // Zurückgelegte mm pro Dynamo-Impuls
#define RUNDEN_BYTE(d)            ((byte) ((d) + ((d) > 0 ? 0.5 : -0.5)) )                        // Rundet Float zu Byte
#define KMH_TO_IMP(k)             (RUNDEN_BYTE( ( (k) * 1000000.0 / 60.0 / 60.0 ) / MMProI ) )  // km/h in Impulse pro Sekunde umrechnen
#define BEREICH(b)                (KMH_TO_IMP( MaxKmh / (Bereiche - 1) * (b) ))                 // Berechnen der Impulse für die Geschwindigkeits-Bereiche
#define BEREICH_KMH(b)            ( MaxKmh / (Bereiche - 1) * (b) )
// ###################################################
// #                   VARIABLEN                     #
// ###################################################
enum{                                                     // Aufzählung für Menü-Wahl
      HauptMenu,
      MerkerMenu,
      EinstllMenu
    };
const    byte             GeschwBereiche[Bereiche]        // Grenzen für die Geschwdigkeitsbereiche in Impulse / Sek
                          = {
                            BEREICH(6), 
                            BEREICH(5), 
                            BEREICH(4), 
                            BEREICH(3), 
                            BEREICH(2), 
                            BEREICH(1), 
                            0
                            };  
volatile unsigned long    Geschwindigkeit[Bereiche];      // Zählspeicher für die einzelnen Kilometerstände in Meter
volatile unsigned int     Geschwindigkeitmm[Bereiche];    // Zählspeicher für die einzelnen Kilometerstände-Reste in Millimeter                                                        
volatile byte             SpeicherRot[Bereiche];          // Speichert den Offset der Ringspeicher
volatile unsigned int     DynImpulse;                     // Die Dynamoimpulse seit dem letzten Auslesen
volatile unsigned long    Time;                           // Vergangene Zeit in Millisek, wird vom WDT geführt, Überlauf nach 49 Tagen 
volatile unsigned long    Time2;                          // Zeitspeicher zum regelmäßigen Berechnen der Geschwindigkeit
volatile unsigned long    Time3;                          // Zeitspeicher zum regelmäßigen Speichern der Geschwindigkeit
volatile unsigned long    Time4;                          // Zeitspeicher zum Messen der Zeit seit dem letzten Impuls
volatile unsigned long    Time5;                          // Zeitspeicher zum Messen der Zeit seit dem letzten Speichern im ReserveSpeicher
volatile byte             Pruefsumme;                     // Hilfsvariabel zum Berechnen einer Prüfsumme 
volatile bool             LEDStatus1;                     // Hilft beim Blinken der LED
volatile bool             LEDStatus2;                     // Hilft beim Blinken der LED
volatile bool             ERROR = false;                  // Für Fehler beim Speicher-Laden in setup()
volatile unsigned int     i,j,k,l;                        // Schleifenvariablen
volatile byte             Temp,Temp2;                     // Variablen für Zwischenergebnisse
volatile unsigned long    TempLong;
volatile float            TempFloat;
// ###################################################
// #                   Funktionen                    #
// ###################################################
void InputFilter();
void WriteEEprom(byte EEprombereich);
bool ReadEEprom(byte EEprombereich, bool debug = false);
void SerielleKomunikation();
bool menuAnzeigen(byte menu);
void zeigeAktuell();
#if MerkerMax > 0
  void uebrtrageMerker();
  void speichereMerker();
  void loescheMerker();
  void cpMerker(byte src, byte dest);
  void delMerker(byte merker);
  void delAllMerker();
#endif
void testEEprom();
void loescheZaehler(); 
void delZaehler();
void initEEPROM();
void spacesdec(unsigned long Zahl, byte laenge);
// ###################################################
// #                Initialisierung                  #
// ###################################################
void setup(){
  // Pins vorbereiten
  pinMode(ClockPin, INPUT_PULLUP);     // Eingang Dynamo-Impulse
  bitWrite(LED1Port, LED1Pin, OFF);    //LEDs aus
  #if BOARD == 1
    bitWrite(LED2Port, LED2Pin, OFF);  // Nur bei Arduino Micro Pro
  #endif
  // Alte Daten aus dem Speicher auslesen
  if(!ReadEEprom(ErsterSpeicher))
    if(!ReadEEprom(ZweiterSpeicher))
      if(!ReadEEprom(ReserveSpeicher))
        ERROR = true;   // Gelangt das Programm an diese Stelle, sind alle 3 Speicher ungültig
  // Wenn Modul nicht am Dynamo angeschlossen oder im Stand (500ms kein LOW-Signal), dann serielle Verbindung aufbauen
  if(pulseIn(ClockPin, 0, 5000000) == 0)
    SerielleKomunikation();
  
  ADCSRA = 0;                                   // ADC ausschalten zum Strom sparen
  MCUSR = 0;
  WDTCSR = _BV (WDCE) | _BV (WDE);              // Um den WDT umstellen zu können beide Bits leeren, danach 4 Clk-Zycl Zeit.
  WDTCSR = _BV (WDIE);                          // Setzt WDIE  und 16 Millisekunden Verzögerung
  wdt_reset();                                  // Resette den WDT
  attachInterrupt(IntNum, InputFilter, CHANGE); // Das Interrupt vom ClockPin aktivieren
}
// ###################################################
// #                 Hauptprogramm                   #
// ###################################################
void loop(){
  // Wenn kein Speicher geladen werden konnte nur blinken
  if(ERROR){
    if(Time > 300){
      Time = 0;
      LEDStatus2 = ON;
    }
  }
  // Alle 'MaxSpeicherabstand' Sekunden die Stände speichern
  // Außerdem, wenn der letzte Impuls länger als 50ms (langsamer 5,45km/h) her ist
  // Aber nicht öfter als MinSpeicherabstand und nur wenn nicht im Stand (dynimpulse>0)
  else if((Time - Time4 > 50 && Time - Time3 >= MinSpeicherabstand && DynImpulse > 0) || Time - Time3 >= MaxSpeicherabstand){
    // Braucht etwa 110ms
    WriteEEprom(ErsterSpeicher);
    WriteEEprom(ZweiterSpeicher);
    LEDStatus2 = ON;  // blink Speicher-LED
    Time3 = Time - (Time % 1000);     // Speicherzeitpunkt aktualisieren, (Time % 1000), damit nur volle Sekunden gespeichert werden
  }
  // Zu jeder vollen Sekunde die Impulse dem Geschwindigkeitsbereich hinzuaddieren
  else if(Time - Time2 >= 1000){ // Braucht etwa 680us
    for(i=0; i<Bereiche; i++){
      if(DynImpulse > GeschwBereiche[i]){
        Geschwindigkeit[i] +=   (unsigned int)(DynImpulse * MMProI) / 1000;   // Zähler um die nötigen Meter erhöhen
        Geschwindigkeitmm[i] += (unsigned int)(DynImpulse * MMProI) % 1000;   // Die restlichen Millimeter auch speichern (kommen aber nicht in den EEPROM)
        if(Geschwindigkeitmm[i] >= 1000){                 // Wenn 1000mm über sind,...
          Geschwindigkeit[i]++;                           // dann zu den Metern zurechnen...
          Geschwindigkeitmm[i] -= 1000;                   // und mm zurücksetzen
        }
        break;
      }
    }
    // ReserveSpeicher wird seltener geschrieben (weniger Abnutzung) und nur bei voller Spannung (KmhSicher) => Größere Datensicherheit
    if(DynImpulse > KMH_TO_IMP(KmhSicher) && Time - Time5 >= SpeicherabstandReserve){
      WriteEEprom(ReserveSpeicher);
      LEDStatus2 = ON;  // blink Speicher-LED
      Time5 = Time;
    }
    DynImpulse = 0;
    LEDStatus1 = ON;  // blink Update-LED
    Time2 += 1000;
  }
  // Nach jedem Durchlauf schlafen legen
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  noInterrupts ();
  interrupts ();    // Muss direkt vor sleep_cpu aufgerufen werden, sonst kein Aufwachen mehr
  sleep_cpu ();     // hier findet die Pause statt #####
  sleep_disable();  // hier wieder aufgewacht
}
// ###################################################
// #       Abtasten des Eingangspins (Dynamo)        #
// ###################################################
void InputFilter(){
  DynImpulse++;
  Time4 = Time;
}
// ###################################################
// #               watchdog interrupt                #
// ###################################################
ISR (WDT_vect)
{
  Time += 16;   // WDT läuft alle 16ms über
  #if BOARD == 0
    bitWrite(LED1Port, LED1Pin, (LEDStatus1 || LEDStatus2))
  #elif BOARD == 1
    bitWrite(LED1Port, LED1Pin, LEDStatus1);  // Schaltet die LED nach 16ms wieder aus
    bitWrite(LED2Port, LED2Pin, LEDStatus2);  // Schaltet die LED nach 16ms wieder aus
  #endif
  LEDStatus1 = LEDStatus2 = OFF;
}
// ###################################################
// #          Schreibt die Daten ins Eeprom          #
// ###################################################
void WriteEEprom(byte EEprombereich){
  // Geschwindigkeiten vom Type unsigned long (4 byte) speichern.
  // Variabel wird in 7 Bit Teile zerlegt.
  // Das erste  Byte besteht aus 'true  + 0-7Bit'
  // Das zweite Byte besteht aus 'false + 8-14Bit'
  // Das dritte Byte besteht aus 'false + 15-21Bit'
  // Das vierte Byte besteht aus 'false + 22-28Bit'
  // Das fünfte Byte besteht aus 'false + Prüfsumme'
  for(i=0; i<Bereiche; i++){
    // Der Offset wird berechnet aus: '(Meter / 4096) % 8' (14,13,12 Bit)
    SpeicherRot[i] = Geschwindigkeit[i] >> 12 & B111;
    Pruefsumme = 0;
    for(j=0; j<RotSize; j++){
      if(j == 0){
        // Erste zelle wird durch die 1 im 7 Bit gekennzeichnet
        Temp = (byte(Geschwindigkeit[i] >> 7*j) | B10000000);
        Pruefsumme ^= Temp;
      }
      else if(j <= 3){
        // Die folgenden Zellen werden durch 0 im 7.Bit gekennzeichnet
        Temp = (byte(Geschwindigkeit[i] >> 7*j) & B01111111);
        Pruefsumme ^= Temp;
      }
      else if(j == 4)        // Speichern von 7 Bit der Prüfsumme, das 7. Bit wird zu 0 gesetzt
        Temp = Pruefsumme & B01111111;
      else        // Die weiteren Zellen haben keinen Inhalt, sondern nur die 0 im 7.Bit
        Temp = B00000000;
      // Speicherzelle im Ringspeicher berechnen, SpeicherRot[i] gibt erste Stelle an
      // Durch & B111 wird Überlauf bei 8 erzeugt
      Temp2 = ((SpeicherRot[i]+j) & B111);
      // Speichert nur, wenn sich der Zelleninhalt auch wirklich ändern muss.
      if(EEPROM.read((i*RotSize)+Temp2+EEprombereich) != Temp)
        EEPROM.write((i*RotSize)+Temp2+EEprombereich, Temp);
    }
  } 
}
// ###################################################
// #         Liest die Daten aus dem EEPROM          #
// ###################################################
bool ReadEEprom(byte EEprombereich, bool debug){
  // Offset des Ringspeichers im EEprom finden
  for(i=0; i<Bereiche; i++)
    for(j=0; j<RotSize; j++){
      Temp  = EEPROM.read((i*RotSize)+j+EEprombereich);
      if(bitRead(Temp,7))
        SpeicherRot[i] = j;
    }
  // Daten aus dem EEprom zurücklesen
  for(i=0; i<Bereiche; i++){
    if(!debug)
      Geschwindigkeit[i] = 0;
    Pruefsumme = 0;
    for(j=0; j<5; j++){
      // Speicherzelle im Ringspeicher berechnen, SpeicherRot[i] gibt erste Stelle an
      // Durch & B111 wird Überlauf bei 8 erzeugt
      Temp = ((SpeicherRot[i]+j) & B111);
      if(j < 4) {
        Temp2 = EEPROM.read((i*RotSize)+Temp+EEprombereich);
        if(!debug)
          Geschwindigkeit[i] += ((unsigned long)Temp2 & B01111111) << 7*j;
        Pruefsumme ^= Temp2;
      }
      else{
        Pruefsumme &= B01111111;
        // Wenn die Prüfsumme nicht stimmt, dann wird false zurückgegeben
        if(Pruefsumme != EEPROM.read((i*RotSize)+Temp+EEprombereich))
          if(debug)
            Serial.print("       ERR");
          else
            return false;
        else if(debug){
          Serial.print("        OK");
        }
      }
    }
  }
  if(debug)
    Serial.println();
  return true;
}
// ###################################################
// #          AB HIER NUR SERIELLER MODUS            #
// ###################################################
// ###################################################
// #         Starte das serielle Interface           #
// ###################################################
void SerielleKomunikation(){
  Serial.begin(9600);
  #if BOARD == 1
    while(!Serial); // wait for serial port to connect. Needed for Leonardo/Micro only
  #endif
  Serial.setTimeout(1000000);
  // Menüschleife
  while(true){
    menuAnzeigen(HauptMenu);
  }
}
// ###################################################
// #                 Zeige Menü                      #
// ###################################################
bool menuAnzeigen(byte menu){
  Serial.println();
  if(menu == HauptMenu){
    Serial.println("### MENUE ###");
    Serial.println("a - aktueller Zaehlerstand"); 
    #if MerkerMax > 0
      Serial.println("m - Merker");
    #endif
    Serial.println("e - Einstellungen");
  }
  else if(menu == MerkerMenu) {
    Serial.println("### MERKER ###");
    Serial.println("l - Merker loeschen");
    Serial.println("n - Neuen Merker setzen");
    Serial.println("x - Zurueck");
  }
  else if(menu == EinstllMenu) {
    Serial.println("### EINSTELLUNGEN ###");
    Serial.println("d - Speicher-Diagnose");
    Serial.println("z - Zaehlerstand zuruecksetzen");
    #if MerkerMax > 0
      Serial.println("r - Alle Merker resetten");
    #endif
    Serial.println("i - Initialisieren (z & r)");
    Serial.println("x - Zurueck");
  }
  Serial.println();
  while(Serial.read()>=0) ; // Liest alle alten Zeichen aus dem Buffer
  while(Serial.available() == 0);  // Wartet auf Zeichen
  switch(Serial.read()){
    case 'a': if(menu == HauptMenu) zeigeAktuell(); break;
    #if MerkerMax > 0
      case 'm': if(menu == HauptMenu) uebrtrageMerker(); break;
      case 'r': if(menu == EinstllMenu) delAllMerker(); break;
    #endif
    case 'e': if(menu == HauptMenu) while(menuAnzeigen(EinstllMenu)); break;
    case 'n': if(menu == MerkerMenu) speichereMerker(); break;
    case 'l': if(menu == MerkerMenu) loescheMerker(); break;
    case 'z': if(menu == EinstllMenu) loescheZaehler(); break;
    case 'd': if(menu == EinstllMenu) testEEprom(); break;
    case 'i': if(menu == EinstllMenu) initEEPROM(); break;
    case 'x': if(menu == MerkerMenu || menu == EinstllMenu) return false; break;
    default: break;
  } 
  return true;  
}
// ###################################################
// #        Ausgabe-Überschriften generieren         #
// ###################################################
void printBereiche(){
  for(i=Bereiche;i>0;i--){
    if(i==1){
      Serial.print("   ");
      if(BEREICH_KMH(1) <10)
        Serial.print(" ");
      Serial.print("<");
      Serial.print(BEREICH_KMH(1));
      Serial.print("km/h");
    }
    else{
      Serial.print("  ");
    if(BEREICH_KMH(i-1) <100)
      Serial.print(" ");
    if(BEREICH_KMH(i-1) <10)
      Serial.print(" ");
    Serial.print(">");
    Serial.print(BEREICH_KMH(i-1));
    Serial.print("km/h");
    }
    
  }
}
// ###################################################
// #         Zeigt aktuellen Zählerstand             #
// ###################################################
void zeigeAktuell(){
  Serial.println("Aktueller Zaehlerstand:");
  printBereiche();
  Serial.println("   Summe  (in Kilometer)");
  TempLong = 0;
  for(i=0; i<Bereiche; i++){
    spacesdec(Geschwindigkeit[i]/1000, 10-4);
    Serial.print((float)Geschwindigkeit[i]/1000, 3);
    TempLong += Geschwindigkeit[i];
    delay(10);
  }
  Serial.print(" = ");
  Serial.println((float)TempLong/1000, 3); 
}
//###################
#if MerkerMax > 0  // Beginn Merker Menü-Funktionen
//###################
// ###################################################
// #           Zeige Liste der Merker                #
// ###################################################
void uebrtrageMerker(){
  do{
    Serial.println();
    Serial.print("Nr. Merker          ");
    printBereiche();
    Serial.println("         Summe");
    // start bei 196
    // Ein Merker ist 15 + 7 * 3 = 36 Byte, max 20 Merker
    for(i=0; i<MerkerMax; i++){
      spacesdec(i, 3);
      Serial.print(i);
      Serial.print(' ');
      for(j=0; j<15; j++){
        Temp = EEPROM.read((MerkerSpeicher+i*MerkerSize)+j);
        if(Temp == 0) Temp = ' ';
        Serial.print((char)Temp);
      }
      Serial.print(' ');
      unsigned long TempLong2;
      TempLong2 = 0;
      for(k=0; k<Bereiche; k++){
        TempLong = 0;
        for(l=0; l<3; l++)
          TempLong += (unsigned long)(EEPROM.read((MerkerSpeicher+i*MerkerSize)+j+(k*3)+l)) << 8*l;
        TempFloat = (float)TempLong / 10;
        spacesdec(TempFloat, 10-2);
        delay(1);
        Serial.print(TempFloat,1);
        TempLong2 += TempLong;
      }
      spacesdec(TempLong2/10, 14-2);
      delay(1);
      Serial.println((float)TempLong2/10,1);
    } 
  } while(menuAnzeigen(MerkerMenu));
}
// ###################################################
// #             Speichert einen Merker              #
// ###################################################
// Ein Merker ist 15 + 7 * 3 = 36 Byte, max 20 Merker
// Original ein 28 Bit Zähler, es werden aber nur 100m Abstufungen gespeichert
// 28 Bit  => 2,68435456e8, ANS/100 = 2,68435456e6, 24 Bit => 1,67772160e7 : Der Speicher reicht
void speichereMerker(){
  // Ersten freien Speicher suchen gekennzeichnet durch leerzeichen (32) oder 0
  for(i=MerkerSpeicher; i<(MerkerSpeicher+(MerkerMax*MerkerSize)); i+=MerkerSize){
    Temp2 = EEPROM.read(i);
    if(Temp2 == ' ' || Temp2 == 0){
      char buffer[16];
      Serial.println("Merker eingeben (max. 15 Zeichen. Merker, die mit einem Leerzeichen beginnen werden beim naechsten Mal automatisch wieder ueberschrieben.):");
      delay(100);
      while(Serial.available()) Serial.read();  // Liest alle alten Zeichen aus dem Buffer
      Temp = Serial.readBytesUntil('\n', buffer, 16);
      Serial.println("Eintrag speichern...");
      for(j=0; j<15; j++)
        if(j<Temp)
          EEPROM.write(i+j, buffer[j]);
        else
          EEPROM.write(i+j, ' ');
      for(k=0; k<Bereiche; k++)
        for(l=0; l<3; l++)
          EEPROM.write(i+j+(k*3)+l, byte((int)(Geschwindigkeit[k]/100) >> 8*l));
      Serial.println("Gespeichert");
      return;
    }
  }
  Serial.println("ERR: Speicher voll");
}
// ###################################################
// #              Löscht einen Merker                #
// ###################################################
void loescheMerker(){
  // Ein Merker ist 15 + 7 * 3 = 36 Byte, max 20 Merker
  Serial.println("Welcher Merker soll geloescht werden?");
  Temp = Serial.parseInt();
  Serial.println("Merker loeschen...");
  if(Temp < 0 || Temp > (MerkerMax-1)) {
    Serial.println("Eintrag nicht vorhanden");
    return;
  }
  for(i=Temp; i<MerkerMax; i++){
    Temp2 = EEPROM.read((MerkerSpeicher+i*MerkerSize));
    if(i<(MerkerMax-1) && Temp2 != 0)
      cpMerker(i+1,i);
    else{
      delMerker(i);
      break;
    }
  }
  Serial.println("Geloescht");
  return;
}
// ###################################################
// #             Kopiert einen Merker                #
// ###################################################
void cpMerker(byte src, byte dest){
  for(j=0; j<15; j++)
    EEPROM.write(MerkerSpeicher+dest*MerkerSize+j, EEPROM.read(MerkerSpeicher+(src)*MerkerSize+j));
  for(k=0; k<Bereiche; k++)
    for(l=0; l<3; l++)
      EEPROM.write(MerkerSpeicher+dest*MerkerSize+j+(k*3)+l, EEPROM.read(MerkerSpeicher+(src)*MerkerSize+j+(k*3)+l));
}
// ###################################################
// #         Löscht einen Merker [NO TEXT]           #
// ###################################################
void delMerker(byte merker){
  for(j=0; j<15; j++)
    EEPROM.write(MerkerSpeicher+merker*MerkerSize+j, 0);
  for(k=0; k<Bereiche; k++)
    for(l=0; l<3; l++)
      EEPROM.write(MerkerSpeicher+merker*MerkerSize+j+(k*3)+l, 0);
}
// ###################################################
// #               Löscht alle Merker                #
// ###################################################
void delAllMerker(){
  Serial.println();
  Serial.println("Alle Merker wirklich zuruecksetzen? (j/n)");
  while(Serial.read()>=0) ; // Liest alle alten Zeichen aus dem Buffer
  while(Serial.available() == 0);  // Wartet auf Zeichen
  if(Serial.read() == 'j'){
    Serial.print("Merker löschen...");
    for(i=0; i<MerkerMax; i++)
      delMerker(i);
    Serial.println("OK");
  }
  else
    Serial.println("Abbruch");
  Serial.println();
}
//##########
#endif // Ende Merker Menü-Funktionen
//##########
// ###################################################
// #      Teste Prüfsummen der Speicherbereiche      #
// ###################################################
void testEEprom(){
    Serial.println("Speicher-Pruefsummen testen...");
    Serial.println();
    Serial.print("Speicherbereich ");
    printBereiche();
    Serial.println();
    Serial.print("Speicher 1      ");
    ReadEEprom(ErsterSpeicher, true);
    Serial.print("Speicher 2      ");
    ReadEEprom(ErsterSpeicher, true);
    Serial.print("Reserve         ");
    ReadEEprom(ErsterSpeicher, true);
    Serial.println("Test beendet. "); 
}
// ###################################################
// #            Löscht den Zählerstand               #
// ###################################################
void loescheZaehler(){
  Serial.println();
  Serial.println("Gesamt-Zaehler wirklich zuruecksetzen? (j/n)");
  while(Serial.read()>=0) ; // Liest alle alten Zeichen aus dem Buffer
  while(Serial.available() == 0);  // Wartet auf Zeichen
  if(Serial.read() == 'j'){
    Serial.print("Zaehler loeschen...");
    delZaehler();
    Serial.println("OK");
  }
  else
    Serial.println("Abbruch");
  Serial.println();
  zeigeAktuell();
}
// ###################################################
// #        Löscht den Zählerstand [NO TEXT]         #
// ###################################################
void delZaehler(){
  for(i=0; i<Bereiche; i++)
    Geschwindigkeit[i] =0;
  WriteEEprom(ErsterSpeicher);
  WriteEEprom(ZweiterSpeicher);
  WriteEEprom(ReserveSpeicher);
}
// ###################################################
// #              EEPROM initialisieren              #
// ###################################################
void initEEPROM(){
  Serial.println();
  Serial.println("EEPROM wirklich initialisieren (vorhandene Daten gehen verloren)? (j/n)");
  while(Serial.read()>=0) ; // Liest alle alten Zeichen aus dem Buffer
  while(Serial.available() == 0);  // Wartet auf Zeichen
  if(Serial.read() == 'j'){
    Serial.println("Initialisieren...");
    Serial.print("Zaehler...");
    delZaehler();
    Serial.println("OK");
    #if MerkerMax > 0
      Serial.print("Merker...");
      for(i=0; i<MerkerMax; i++)
        delMerker(i);
      Serial.println("OK");
    #endif
    Serial.println("Fertig");
  }
  else
    Serial.println("Abbruch");
  Serial.println();
}
// ###################################################
// #    Zum ausgerichteten darstellen der Zahlen     #
// ###################################################
void spacesdec(unsigned long Zahl, byte laenge){
  while(Zahl >= 10){
    laenge--;
    Zahl /= 10;
  }
  for(;laenge>1;laenge--) Serial.print(" ");
}
