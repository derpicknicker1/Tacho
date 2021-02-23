> The original website is down.
> Found this on [web.archive.org](https://web.archive.org/web/20170228192853/http://www.elektroniker-bu.de/arduinofahrradtacho.htm)

# Dauerhafter Kilometerzähler fürs Fahrrad
Bei meinem etwa zehn Jahre alten Fahrrad wurden mittlerweile bis auf den Rahmen fast alle Teile erneuert. Alles verschleißt, mehrere Fahrradketten, Zahnkränze, zwei durchgebremste Hohlkammerfelgen... Um mal die Haltbarkeit der Teile zu überprüfen, brauche ich einen Kilometerzähler, der immer mitläuft. Also keinen normalen Tacho, bei dem mal die Batterien leer sind, das Tachokabel beschädigt ist oder der mal wieder Zuhause liegt.

Deshalb entstand die Idee aus einem Arduino Nano einen Kilometerzähler zu bauen, der dauerhaft mitfährt und durch den Wechselstrom des Nabendynamos seine Stromversorgung und auch die Geschwindigkeitsinformation bekommt. Der Arduino kann dann eine Kilometerstatistik erzeugen und diese bei bedarf per USB zum PC übertragen.

Die Arduino Nano Hardware ist für diese Aufgabe nicht perfekt, durch die Größe und die USB-Schnittstelle aber ein guter Anfang.

Zusammengestecktes Arduino-Board mit der Schaltungserweiterung.

## Die Spannungsversorgung
Der Arduino wird mit der Wechselspannung des Nabendynamos versorgt. Diese beträgt bei normaler Fahrt etwa 6,5V AC. Nach dem Gleichrichten kann man so 8,4V DC erhalten. Genau passend für den 5V Spannungsregler auf dem Arduino. Allerdings soll die Schaltung auch schon bei Schrittgeschwindigkeit laufen, wo nur etwa 3V AC zur Verfügung stehen.

Um auch bei dieser Geschwindigkeit eine ausreichende Spannung zu haben, kann stattdessen eine Ladungspumpe eingesetzt werden. Diese braucht allerdings recht große Kapazitäten, um bei der niedrigen Frequenz des Wechselstroms ausreichend zu arbeiten. Bei Schrittgeschwindigkeit sind etwa 3V bei 20Hz zu erwarten. Mit der gezeigten Schaltung und den verwendeten Werten können so etwa 10mA bei 5km/h bereitgestellt werden. Für größere Ströme wären größere Kondensatoren nötig. Der 1000 µF Kondensator sorgt dafür, dass auch nach dem Bremsen noch Zeit bleibt die Daten im EEPROM zu speichern.

Es wird eine Brown-Out-Schaltung benötigt, damit es nicht zu Speicherfehlern im EEPROM kommt. Diese hält den Mikrocontroller resettet, solange die Spannung nicht ausreichend ist. Der MEGA32 besitzt eine eingebaute Brown-Out-Schaltung. Für den Arduino Nano sind die Fuses der Brown-Out-Schaltung auf 2,7 Volt gesetzt. Da die 2,7V laut Datenblatt etwas zu wenig für die verwendete Taktrate und den EEPROM-Betrieb sind muss darauf beim Programm besonders geachtet werden.

## Der Stromverbrauch
Der Arduino Nano verbrauch etwa 25mA bei ausgeschalteten LEDs (bis auf die Power-LED) und arbeitendem Controller. Das ist an sich nicht viel Strom, aber leider noch zu viel für die passive Ladungspumpe, die zur Spannungserhöhung eingesetzt wird.

Der Stromverbrauch setzt sich zusammen aus:

3,4mA für die Power-LED
2-3mA für den 5V-Regler
17mA für den MEGA32u4
Wobei der Stromverbrauch des MEGA32u4 stark Spannungsabhängig ist. Da das Bord selber nicht zu stark verändert werden soll, ist der Strom für den Spannungsregler fest, und es muss der Stromverbrauch des MEGA32 durch sparsame Programmierung verringert werden. Im Mittel ist der Gesamtverbrauch durch möglichst lange Schlafzyklen so auf etwa 7mA im Mittel absenkbar.

## Die gesamte Schaltung
Das folgende Schaltbild zeigt alle Komponenten der Schaltung im Gesamten. Beachte, dass keine besondere Schutzschaltung vorgesehen ist. Die Dynamospannung darf nicht über ca. 7 AC steigen. Der 5V Linearregler kann durch eine mögliche Rückwärtspannung beschädigt werden. Die Ladungspumpe selbst kann durch ihren Aufbau keine Rückwärtsstrom durch den Regler erzeugen. Es darf aber keine weitere Last an der Ladungspumpe vor dem Linearregler angeschlossen sein, dann ist der Betrieb zuverlässig.

Der Optokoppler ist nötig, da durch die Ladungspumpe die Schaltung nicht auf GND-Potential liegt. Zudem ist dieser wohl die einfachste Möglichkeit aus der Dynamospannung ein Tachosignal zu generieren.

Schaltplan zeigt die Komponenten, um die der Arduino erweitert werden muss.

Das folgende Bild zeigt meinen Aufbau der Schaltung auf einer kleinen Lochrasterplatine.

Foto des Arduinoboards und der Aufsteckplatine.

## EEPROM Abnutzung
Die EEPROM-Zellen im ATMEGA32 Chip haben 100 Tausend garantierte Schreibzyklen. Das ist hundertmal weniger, als im PIC16F84, in dem die Zellen mindestens 10 Millionen Zyklen halten. Es werden im Tacho Zähler verwendet, die einen Zählerstand von 50 Millionen Metern erreichen sollen (5000km pro Jahr, 10 Jahre Betrieb). Der Zähler besteht aus 4 ∙ 7 Bit von denen immer 7 Bit in einem Byte des EEPROM unterkommen. Man kann vereinfacht sagen, dass pro Zählerstand zwei Schreibvorgänge in den EEPROM nötig sind, da ein Überlauf, bei dem auch die obersten Zellen geschrieben werden müssen im Verhältnis dazu selten vorkommt.

Da für jeden Zähler eine Prüfsumme verwendet wird, die ebenfalls jedes Mal geschrieben werden muss sind somit meistens drei Schreibvorgänge pro Zählerstand nötig.

Um eine lange Lebensdauer zu erreichen, wird die Belastung auf mehrere Zellen verteilt. Der 4 Byte lange Zähler und die Prüfsumme wird deshalb auf 8 EEPROM-Zellen verteilt. Mit diesen zusammen können im ATMEGA32 schon 800 Tausend Schreibzyklen erreicht werden. Es können also ca. 260 Tausend Zählerstände gespeichert werden, bevor der EEPROM abgenutzt ist.

Die einzelnen Zähler zählen nun aber nicht mit der kleinsten Auflösung, sondern je nach der zugeordneten Geschwindigkeit in unterschiedlich großen Schritten, zudem wird nur zu bestimmten Zeitpunkten gespeichert. Liegt der typische Speicherabstand bei 60 Sekunden so erreicht jeder Zähler eine Lebensdauer von 41600 Stunden.



Die oben beschriebene gleichmäßige Verteilung der Belastung auf mehrere Zellen wird über eine Art Ringspeicher realisiert. Die 8 Speicherzellen werden dazu als geschlossene Schleife verwendet. Der Speicherinhalt nimmt 5 Byte ein. Diese 5 Byte werden immer in der gleichen Reihenfolge gespeichert. Nur der Offset im Ringspeicher variiert. Das erste Byte im Ringspeicher wird durch eine 1 im 7ten Bit gekennzeichnet, alle anderen Bytes Tragen eine 0 im 7tem Bit. In regelmäßigen Abständen (ca. alle 4km) werden die 5 Bytes im Ringspeicher weitergeschoben. Dadurch verteilt sich die Belastung gleichmäßig auf alle 8 Zellen.

## Der Sketch
Das Programm wurde für den Arduino Nano geschrieben, es kann aber leicht auch für andere Boards, wie bsw. den Micro angepasst werden.

Im Folgenden werde ich die wichtigsten Funktionen des Sketches kurz erklären. Für mehr Details lese bitte die Kommentare die im Sketch stehen. Es sind nur die Hauptfunktionen gut Kommentiert. Die Funktionen für die Datenbearbeitung über die serielle Schnittstelle sind noch nicht komplett ausgereift und nur spärlich kommentiert.

**void setup()**
In der »setup«-Funktion wird zunächst der Pin der LED (Pin 13) sowie der Eingang für das Taktsignal vom Tacho (ClockPin) konfiguriert. Anschließend wird der alte Zählerstand aus dem EEPROM ausgelesen, dabei wird bei Fehlern in einem der Speicher automatisch auf einen Reservespeicher zugegriffen. Mit den so eingelesenen Zählerständen geht das Programm entweder in den Modus zur seriellen Kommunikation, oder in den normalen Tachobetrieb über. Als Entscheidungsmerkmal wird der ClockPin für 500ms auf Flankenwechsel überwacht um den Tachobetrieb vom Betrieb am Computer zu unterscheiden. Am Ende der »setup«-Funktion werden unnötige Komponenten ausgeschaltet und das Interrupt vorbereitet.

**void loop()**
In der eigentlichen Hauptfunktion erfolgt jede volle Sekunde die Auswertung der Geschwindigkeit. Dazu wird aus den Impulsen in der Variabel »DynImpulse« und der bekannten Dynamo/Reifen-Kombination die Durchschnittsgeschwindigkeit der letzten Sekunde berechnet und die zurückgelegten Meter dem passenden Zähler zugeordnet.
Zusätzlich wird von der Hauptfunktion aus die Speicherung im EEPROM organisiert. Es gibt dabei drei Bedingungen, die eine Speicherung auslösen. Zunächst wird mindestens alle 60 Sekunden gespeichert. Zusätzlich wird gespeichert, wenn der letzte Dynamoimpuls länger als 50ms her ist (Geschwindigkeit < 5,5km/h). Um den Speicher zu schonen wird aber niemals öfter als alle 5 Sekunden gespeichert.
Als zusätzliche Sicherheit wird bei ausreichend großer Geschwindigkeit alle 2 Minuten der Zählerstand in einem speziellen Speicherbereich gesichert. Dieser hat weniger Abnutzung und wird durch die Mindestgeschwindigkeit auch nur bei ausreichender Spannung geschrieben.
Die LED Blitzt zu jeder Sekunde schwach und nach jedem Speichern stark auf. Somit zeigt das helle Aufblitzen deutlich an, ob die Energie beim Bremsen noch zum Speichern gereicht hat.

**void InputFilter()**
Der InputFilter wird als Interrupt-Funktion bei jedem Flankenwechsel am ClockPin aufgerufen. Es wird der DynImpulse-Zähler erhöht und in Time4 die Zeit des letzten Impulses gespeichert. Der Wert in Time4 wird benötigt um beim Bremsen noch rechtzeitig die Datenspeicherung zu veranlassen.

**void WDTWait()**
Die »WDTWait«-Funktion sorgt dafür, dass der ATMEGA in den Sleep-Modus geht. Durch den WDT wird er nach 16ms, oder beim nächsten Flankenwechsel am ClockPin wieder aufgeweckt.

**void ISR (WDT_vect)**
Alle 16ms wird das WDT-Interrupt ausgelöst. Dann wird die Variabel Time erhöht. Das Zählen der Zeit erfolgt hier nicht mir der Arduino-Funktion millis(), da diese durch den Sleep-Modus angehalten wird. Der Tacho verlässt sich auf die Genauigkeit des internen WDT-Oszillators.

**void WriteEEprom(byte EEprombereich)**
Die Funktion zum Beschreiben des EEPROMs kümmert sich um alles was benötigt wird um die einzelnen Zähler in die Ringspeicher abzulegen. Der Funktion wird dazu die Adresse der ersten Speicherzelle des zu verwendenden Blocks übergeben. Ein Block besteht aus sieben Zählern, die alle 8 Byte belegen, entsprechend ist ein Block 56 Byte lang.
Die Funktion berechnet aus dem Zählerstand den benötigen Offset für die einzelnen Ringspeicher und übernimmt das Schreiben. Eine Zelle wird nur geschrieben, wenn sich der Inhalt der Zelle auch wirklich ändern muss. Zusätzlich wird für jeden Zähler eine Prüfsumme berechnet und ebenfalls gespeichert.

**boolean ReadEEprom(byte EEprombereich)**
Das Lesen des EEPROMs wird von der »ReadEEprom«-Funktion übernommen. Es wird jeweils der Speicherblock (56 Byte) an der angegeben Adresse ausgelesen, der Ringoffset rekonstruiert und der Speicherwert berechnet. Wenn die Prüfsummen aller 7 Zählerstände fehlerfrei war gibt die Funktion »true« zurück.

## Im Betrieb
Mittlerweile hängt der Arduino seit einer Zeit am Rad und hat fleißig 3200 Kilometer erfasst. Vor kurzen stand ein Kettentausch an, die Ritzel und die Kettenblätter wurden auch erneuert, ich werde die Lebensdauer genau beobachten. So sieht die Ausgabe am PC aus, die Namen für gespeicherte Ereignisse können nur 15 Zeichen lang sein, deshalb die komische Schreibweise.

  | >30km/h | >25km/h  | >20km/h  | >15km/h  | >10km/h  |  >5km/h  |  <5km/h  | Summe  (in Kilometer)  |
  |---------|----------|----------|----------|----------|----------|----------|------------------------|
  |   5.353 |   38.168 |  849.494 | 3818.789 |  769.580 |  171.191 |   36.009 |= 5688.584              |
  
  
  |Nr. Ereignis|>30km/h|>25km/h|>20km/h|>15km/h|>10km/h|>5km/h|<5km/h|Summe|
  |---|---|---|---|---|---|---|---|---|
  |0 |29.4.13WarStart|        1.1  |     5.5  |   103.9  |  242.8   |   40.0  |     8.3   |    1.2    |     402.8|
  | 1 |Pedale   31.05   |      1.1   |    6.6    | 134.4  |   344.4    |  58.1  |    13.2    |   1.5   |      559.3|
  |   2 |15.08KeteKasete   ||     2.9   |   18.4  |   364.1 |   1120.5  |   231.2  |    46.8   |    9.9     |   1793.8|
  |   3 |16.08Kettenblat   |     3.0   |   18.7  |   369.7  |  1144.0  |   235.7 |     47.4   |    9.9    |    1828.4|
  |   4 |17.11MantelBack   |     4.4   |   30.5  |   649.9 |   2420.1  |   475.1   |  104.2   |   26.8    |    3711.0|
  |   5| 10.12BremseH      |     4.7    |  33.0  |   684.7  |  2722.7  |   551.0  |   120.2  |    30.4    |    4146.7|
  |   6 |1.1StattelUStue   |     5.0   |   34.5  |   727.7  |  2894.9  |   580.3   |  127.5   |   31.6    |    4401.5|
  |   7 |                  |     0.0   |    0.0   |    0.0   |    0.0  |     0.0   |    0.0   |    0.0    |       0.0|
  |   8 |                   |    0.0    |   0.0   |    0.0   |    0.0   |    0.0   |    0.0   |    0.0    |       0.0|
  |   9 |                   |    0.0   |    0.0   |    0.0   |    0.0    |   0.0   |    0.0    |   0.0    |       0.0|
  
Erstellt im August 2013.
