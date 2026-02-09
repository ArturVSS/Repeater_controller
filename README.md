Platforma: ESP32-WROOM-32

![dev_kit_pinout-1](https://github.com/user-attachments/assets/e2475e20-813c-4056-b423-607389fa5d9f)


Logika: Gdy przemiennik jest wyłączony oczekuje sygnału carrier i 1750Hz przez jedną sekundę. Po tym czasie włącza PTT na 10 sekund i zaczyna mrugać dioda wskazująca odliczanie czasu. Po zaniknięciu carrier generuje roger-beep. Po kolejnych 3 sekundach generuje znamiennik przemiennika. Po odliczeniu 10 sekund od ostatniego sygnału carrier wyłącza PTT ale sekundę wcześniej generuje ton wyłączenia. Każde pojawienie się sygnału carrier resetuje licznik czasu do wyłączenia PTT. Znamiennik generowany jest minimalnie co 60 sekund. Gdy podczas generowania znamiennika pojawi się sygnał carrier dalsze generowanie znamiennika zostanie przerwane.

![www1-1](https://github.com/user-attachments/assets/ec1f65c4-a1a0-4c3a-bb9e-f5935d73da61)
![www2-2](https://github.com/user-attachments/assets/224e18ef-805c-498e-a942-97aa57806221)

TODO (w kolejności):

obsługa playera MP3 – znamiennik głosowy
dekoder DTMF – sterowanie przemiennikiem
pomiar RSSI i komunikaty głosowe, sterowanie radiotelefonami
moduł GSM SIM800 do zdalnego sterowania i odsłuchu przemiennika
strona WWW do wizualizacji stanu (pomiary SWR, napięć i temperatur) i edycji parametrów
wysyłanie beacona APRS do serwera + telemetria.
obsługa dwóch przemienników (2m/70cm) z możliwością pracy crossband

Schemat:

![schemat-2](https://github.com/user-attachments/assets/59322ca1-ef93-4701-9eea-59b4f9c8cffc)

Opis wejść i wyjść:

Wejście nośnej (carrier) – GPIO34 (pull-up 10kom do VCC)
Wejście tonu 1750Hz – GPIO35 (pull-up 10kom do VCC)
Wyjście PTT – GPIO32 (stan wysoki po zadziałaniu)
Wyjście dźwięku – GPIO23
Dioda wskazująca odliczanie czasu – GPIO4 (LED do masy)
Dioda wskazująca podłączenia do wifi – GPIO2 (LED do masy)
SDA dla ADS1115 – GPIO21
SCL dla ADS1115 – GPIO22

Wejścia CARRIER i 1750 są podciągnięte do VCC poprzez rezystory 10kom. Zmiana stanu wymaga zwarcia do masy. Na schemacie na dole znajduje się układ z transoptorem CNY17. Z tym układem oddzielamy galwanicznie wyjścia z radia do procesora. Zmienia się wtedy logika, na transoptor (pin 1) podajemy 5V. Wyjście z transoptora z pin 5 podajemy na wejście procesora, tak jak pokazuje strzałka.

Dźwięk z GPIO252 można wygładzić poprzez zastosowanie filtra RC na wyjściu IN: rezystor R1-100om i kondensator C1-470nF:
<img width="320" height="230" alt="8006216500_1605259069" src="https://github.com/user-attachments/assets/3f625cbd-0eba-4ac7-a6b6-db1b1138dfc2" />

Gdy nie wykorzystujesz sygnału tonu 1750Hz zewrzyj pin na stałe do masy. Dekoder 1750Hz przy wykorzystaniu NE567:

![NE567-768x434](https://github.com/user-attachments/assets/e033460d-ae35-4caa-be19-ca8e51a5f097)


Moduł ADS1115:

![ADS1115-BOARD](https://github.com/user-attachments/assets/347323f1-f0e0-48e2-a014-ca8fa6089b72)

Podłaczenie do procesora:

![ads1115-1](https://github.com/user-attachments/assets/ba7c4efd-7165-46d3-8c87-c205f4e28a96)

ADS1115 to 4-kanałowy przetwornik ADC 16-bitowy z interfejsem I2C. Wykorzystałem moduł z linku poniżej. Płytka jest podłączona do ESP32 dwoma przewodami: SDA do portu 21 a SCL do portu 22. Zasilany napięciem 5V. Pin ADR jest podłączony do pinu SDA wymuszając adres 0x4A. Jak chcesz inny adres podłącz odpowiednio ten pin i zmień w programie:

static const uint8_t ADS1115_ADDR = 0x4A

Układ pracuje w trybie pomiaru +/-6.144V. Cztery wejścia analogowe z rezystorami pulldown do masy (wartość od 47kom do 100kom) służą do pomiaru:

wejście 1 – pomiar RSSI – napięcie pobierane z odbiornika (patrz informacje niżej)
wejście 2 – pomiar napięcia SWR – fala padająca
wejście 3 – pomiar napięcia SWR – fala odbita
wejście 4 – pomiar napięcia zasilającego cały układ i radiotelefony (z dzielnikiem napięcia 1/4)    

    <img width="512" height="466" alt="dzielnik" src="https://github.com/user-attachments/assets/1cfc0ce3-aa36-42f2-8639-63790ed04d9b" />

Poniżej pomiary napięć na wyjściu RSSI w Motoroli GM900 w zależności od poziomu sygnału wejściowego w wartościach skali S w raportach radioamatorskich. Będzie to służyć do wyświetlania poziomu sygnału na WWW a docelowo na informowaniu głosem na przemienniku. Sygnał podawany z analizatora radiokomunikacyjnego Motorola R2600, napięcie mierzone miernikiem SANWA MIER-PC7000. Wartości powinny być zbliżone w innych modelach radiotelefonów Motorola. Sygnał podłączony do wejścia pierwszego ADS1115.

![RSSI-1024x442](https://github.com/user-attachments/assets/09cf49e2-a1c7-42ec-89c5-ea8642111c7d)

W programie umieszczono tabelę odpowiadająca za wyświetlanie na stronie WWW poziomu w dBm i S-metra na podstawie powyższych pomiarów. W zależności jaki używasz odbiornik wykonaj pomiary napięcia w stosunku do poziomów sygnału i wpisz odpowiednie wartości w tabelę w programie. W kolejności: napięcie RSSI, siła sygnału S, poziom w dBm.

static const CalPoint calTable[] = {
{2.87f, 1.0f, -121.0f},
{3.10f, 2.0f, -115.0f},
{3.41f, 3.0f, -109.0f},
{3.67f, 4.0f, -103.0f},
{3.78f, 5.0f, -97.0f},
{3.85f, 6.0f, -91.0f},
{3.93f, 7.0f, -85.0f},
{4.06f, 8.0f, -79.0f},
{4.19f, 9.0f, -73.0f},
{4.56f, 10.0f, -63.0f},
{4.90f, 20.0f, -53.0f}
};

Poniżej 2,5V na stronie WWW pojawia się informacja „no signal”. Jeżeli w twoim odbiorniku brak sygnału generuje inne napięcie zmień wartość w linii:

// Próg „no signal” dla wejścia 1
static const float NO_SIGNAL_VOLT = 2.50f;


YT: https://youtu.be/HffPlTuqox8


  

Moduły wykorzystane w projekcie:

Player MP3 – DFPlayer: https://pl.aliexpress.com/item/1005007823454899.html

Dekoder DTMF: https://pl.aliexpress.com/item/1005009121093164.html

Moduł GSM SIM800: https://pl.aliexpress.com/item/1005009593121681.html

Wzmacniacz m.cz. PAM8403 2W: https://pl.aliexpress.com/item/1005006582698705.html

Przetwornik ADC 16-bit 4 kanały ADS1115 https://pl.aliexpress.com/item/1005006143923238.html










