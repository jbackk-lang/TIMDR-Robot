# TIMDR-Robot

Szkielet warstwy TIMDR dla robota wieloosiowego (np. ramienia) i jego
podsystemow (chwytak, podstawa mobilna, kamera, zasilanie) - wykrywanie
anomalii na podstawie syntetycznych danych z enkoderow/IMU/czujnikow, z
(symulowanym) mostkiem do sterownika robota, agregacja ponad wieloma
robotami (flota), i szkielet mostow integracyjnych (ROS2/MQTT/OPC-UA) +
strumieniowa analiza w oknie kroczacym.

**STATUS: SZKIELET + DANE SYNTETYCZNE.** Zero testow na prawdziwym
robocie/sprzecie w chwili napisania tego kodu. Zgodnie z ustalonym
podejsciem ("najpierw szkielet + dane syntetyczne, potem realne testy")
ten etap celowo NIE laczy sie z zadnym prawdziwym sterownikiem, magistrala
(CAN/EtherCAT/ROS) ani czujnikiem. Wszystkie liczby w dashboardzie i
demo pochodza z generatorow syntetycznych (`timdr_robot/sensor_bus.py`,
`timdr_robot/subsystems.py`), nie z pomiarow. Mosty integracyjne
(`timdr_robot/bridges/`) sa stubami kontraktu w trybie dry-run - nic nie
laczy sie z prawdziwym brokerem/serwerem.

## Architektura (5 warstw)

Zgodnie z dokumentem architektury, ktory zainicjowal ten projekt:

1. **Robot Hardware** — enkodery, IMU, czujniki sily/momentu. W tym
   repo: `timdr_robot/sensor_bus.py` (SYNTETYCZNE dane, nie sterownik
   sprzetu).
2. **Robot Control** — profil ruchu osi (tu: uproszczony ruch
   sinusoidalny zamiast pelnego trapezu predkosci - patrz ograniczenia
   nizej).
3. **TIMDR Core** — `timdr_robot/core.py` (model harmoniczny +
   `timdr_robot/geometry.py` (torsja Freneta-Serreta portretu fazowego) +
   `timdr_robot/ringdown.py` (rezonans po zdarzeniu).
4. **TIMDR Integration** — `timdr_robot/status.py` (mapowanie metryk na
   `AxisHealth`) + `timdr_robot/control_bridge.py` (`get_axis_health()`,
   `subscribe_events()`, symulowane reakcje).
5. **System Logic / HMI** — `timdr_robot/hmi.py` (opisy w jezyku
   naturalnym, bez LLM, budowane wylacznie z policzonych statystyk) +
   `api.py`/`static/index.html` (dashboard) + `timdr_robot/fleet.py`
   (agregacja ponad wieloma robotami).

## Podsystemy (poza osiami ramienia)

`timdr_robot/subsystems.py` + `timdr_robot/subsystem_core.py` dodaja
cztery kolejne typy komponentow, kazdy z detektorem DOPASOWANYM do
ksztaltu wlasnego sygnalu (nie mechanicznie tym samym, co dla oscylujacej
pozycji osi - patrz uzasadnienie w docstringach obu plikow):

- **Chwytak**: sila chwytu w cyklu chwyc-trzymaj-pusc. Detektor:
  `baseline_residual()` (reszta wzgledem ruchomej mediany w SZEROKIM
  oknie) + `anomalies()`. `defect_type="slip_event"` (poslizg trzymanego
  przedmiotu).
- **Podstawa mobilna**: dwa kola, prawo kinematyczne napedu
  rozniczkowego (predkosc obrotu z enkoderow) vs niezalezny pomiar
  zyroskopu. `defect_type="wheel_slip"` (utrata przyczepnosci kola -
  enkoder "klamie", zyroskop nie).
- **Kamera/wizja**: blad sledzenia celu (bliski zeru, stacjonarny) -
  `anomalies()` WPROST, bez posredniego modelu. `defect_type=
  "tracking_loss"`.
- **Zasilanie/bateria**: napiecie + prad + temperatura. Laczy limit
  BEZWZGLEDNY napiecia (jak EN 50160 w TIMDR-Grid-Monitor) z adaptacyjna
  reszta (`baseline_residual`) dla zapadow napiecia i z
  `thermal_drift_score()` (dopasowanie liniowego trendu na oknie
  kalibracyjnym, wykrywanie PRZYSPIESZAJACEGO wzrostu) dla
  `defect_type="thermal_runaway"`.

Generyczne detektory w `subsystem_core.py`, reuzywalne poza tymi czterema
podsystemami:

- **`baseline_residual(x, window)`**: reszta wzgledem ruchomej mediany w
  SZEROKIM oknie (>> szerokosc oczekiwanej anomalii) - dla anomalii
  LOKALNYCH (impuls/dip), nie dla dryftu trendu.
- **`thermal_drift_score(x, dt, calib_frac)`**: dopasowanie liniowego
  trendu na oknie kalibracyjnym, reszta rosnaca dla PRZYSPIESZAJACEGO
  odchylenia od tego trendu - dla anomalii TRENDU, nie impulsu.
- **`spectral_energy_drift(x, dt, window)`**: windowed FFT, udzial
  energii wysokoczestotliwosciowej w czasie - GENERYCZNY proxy dla
  "analizy widma wibracji lozyska"/"sygnatury pradowej silnika".
  **JAWNE OGRANICZENIE:** to NIE jest certyfikowana metoda diagnostyki
  lozysk (brak formul BPFO/BPFI wymagajacych geometrii lozyska) ani
  prawdziwa MCSA (brak analizy konkretnych wstazek bocznych wokol
  czestotliwosci sieciowej) - to przesiewowy wskaznik "czy udzial
  wysokich czestotliwosci rosnie", nie diagnoza konkretnej usterki.

## Flota robotow

`timdr_robot/fleet.py`: `RobotUnit` (jeden robot = slownik `StatusEvent`
per komponent) + `Fleet` (wiele `RobotUnit`, agregacja najgorszego
statusu per robot i dla calej floty). Czysto agregujaca warstwa - nie
liczy niczego nowego z sygnalow. Demo: `demo/fleet_demo.py` (4 roboty: 1
zdrowy, 3 z roznymi wstrzykniętymi wadami), `/api/fleet` w dashboardzie.

## Mosty integracyjne (stuby) i streaming

`timdr_robot/bridges/` (`ros2_stub.py`, `mqtt_stub.py`, `opcua_stub.py`):
kontrakt integracji z ROS2/MQTT/OPC-UA. Kazdy uzywa DEFENSYWNEGO importu
opcjonalnej biblioteki (`rclpy`/`paho-mqtt`/`asyncua`, wzorzec identyczny
z obsluga h5py w fusion-tools) - bez zainstalowanej biblioteki dziala w
trybie **dry-run**: `published_log`/`node_values` pokazuja dokladnie, CO
zostaloby wyslane, ale nic nie laczy sie z prawdziwym brokerem/serwerem.

`timdr_robot/streaming.py`: `SlidingWindowAnalyzer` przyjmuje probki
JEDNA PO DRUGIEJ (`push()`), trzyma bufor o stalej dlugosci
(`collections.deque`), re-analizuje go pelnym `core.analyze_axis()` co
`recompute_every` probek. **Jawne ograniczenie:** kalibracja `omega^2` w
oknie kroczacym korzysta z NAJSTARSZYCH zachowanych probek (ktore ciagle
sie przesuwaja), nie z jednego stalego "zdrowego poczatku" jak w analizie
wsadowej calej serii - jesli wada byla obecna wystarczajaco dlugo, moze
czesciowo zaniżyc czulosc detekcji. Zweryfikowane empirycznie: replay
probka-po-probce scenariusza z `backlash` poprawnie przechodzi OK->DEFECT
niedlugo po wstrzykniętym zdarzeniu (patrz `tests/test_streaming.py`).

## Jak to dziala (pipeline `analyze_axis()`)

Dla kazdej osi mamy trzy zsynchronizowane sygnaly: pozycja (kat
enkodera), predkosc katowa, przyspieszenie katowe.

1. **Model harmoniczny** (`core.harmonic_law_residual()`): dopasowuje
   `omega^2` w prawie `przyspieszenie = -omega^2 * pozycja` (prosty
   oscylator harmoniczny) na POCZATKOWYM oknie kalibracyjnym (domyslnie
   pierwsze 30% probek), po czym liczy reszte tego prawa na calej serii.
   Dla gladkiego ruchu reszta jest bliska zeru; luz mechaniczny (backlash)
   czy impuls rezonansowy wprowadzaja przyspieszenie, ktorego to proste
   prawo nie tlumaczy - stad skoki reszty.
2. **Geometria fazowa** (`geometry.frenet_serret_curvature_torsion()`):
   krzywizna i torsja (standardowe wzory Freneta-Serreta) portretu
   fazowego (pozycja, predkosc, przyspieszenie) jako krzywej w R^3. To
   jest GEOMETRYCZNA torsja krzywej - NIE fizyczne skrecenie mechaniczne
   walu. Skoki tej torsji sa dodatkowym, niezaleznym sygnalem anomalii.
3. **Rezonans** (`ringdown.ringdown_resonance()`): dla najwiekszego
   wykrytego zdarzenia sprawdza, czy powrot RESZTY modelu harmonicznego do
   poziomu odniesienia jest oscylacyjny (mozliwe "dzwonienie"
   mechaniczne) czy monotoniczny.
4. **Status** (`status.compute_axis_status()`): mapuje powyzsze na
   `AxisHealth`: `OK` -> `SUSPECT` -> `RESONANCE` -> `DEFECT` (priorytet
   rosnaco, `DEFECT` wygrywa gdy zachodzi wiecej niz jeden warunek).
5. **ControlBridge**: publikuje `StatusEvent`, wywoluje subskrybentow i
   (SYMULOWANE, bez prawdziwego I/O) reakcje: `SUSPECT` -> tylko log,
   `RESONANCE` -> `reduce_speed()`, `DEFECT` -> `reduce_speed()` +
   `increase_damping()` + `stop_axis()` + `alarm()`.

## Scenariusz demonstracyjny

`demo/run_demo.py` (lub dashboard pod `run.bat`) generuje ramie
6-osiowe, gdzie **os 3** ma wstrzykniety rosnacy luz mechaniczny
(backlash) zaczynajacy sie w 60% serii, a pozostale osie sa czyste
(tylko szum pomiarowy). Oczekiwany wynik: os 3 -> `DEFECT`, pozostale
osie -> `OK` lub sporadyczne `SUSPECT` (pojedyncze skoki torsji w
granicach szumu - to normalne, nie usterka).

Uruchomienie:
```
python demo/run_demo.py          # drukuje wynik do konsoli + zapisuje data/demo_run.csv, data/demo_events.json
run.bat                          # (Windows) instaluje zaleznosci i uruchamia dashboard pod http://127.0.0.1:8000
```

## Negative control

`core.negative_control_check()` uruchamia `analyze_axis()` na wielu
CZYSTYCH (bez wstrzykniętej wady) powtorzeniach tej samej osi i liczy,
jak czesto pipeline mimo to zglasza anomalie (false positive rate).
**To NIE jest formalny test permutacyjny/null-model** (brak
pre-rejestracji, Bonferroniego, itd.) - to podstawowy test poczytalnosci.
Aktualny wynik na danych syntetycznych: `false_positive_rate ≈ 0.0` po
poprawce opisanej w Historii poprawek (patrz ponizej) - przed poprawka
wynosil `1.0` (kazda "czysta" os byla falszywie flagowana).

## Ograniczenia (jawne, nie ukryte)

- **Ruch osi**: syntetyczny model to czysta sinusoida (`amplitude *
  sin(omega*t)`), nie prawdziwy trapez predkosci uzywany w realnych
  sterownikach robotow. Model harmoniczny (`omega^2` z jednym
  dopasowaniem) zaklada w przyblizeniu jednoczestosciowy ruch - bardziej
  zlozony, wielosegmentowy profil ruchu wymagalby innego modelu
  referencyjnego (np. dopasowanego wielomianu per-segment albo filtra
  Kalmana ze znanym modelem trajektorii) - POZA zakresem tego szkieletu.
- **Kalibracja `omega^2`** dziala tylko jesli os na POCZATKU analizowanej
  serii jest zdrowa. Wada obecna od pierwszej probki nie zostanie
  wykryta tym mechanizmem.
- **`ringdown_resonance()`** zaklada z grubsza JEDEN dominujacy tryb
  oscylacji - superpozycja kilku trybow (typowa dla realnych ukladow
  mechanicznych wieloosiowych pod obciazeniem) moze byc bledine
  zinterpretowana.
- **ControlBridge**: wszystkie reakcje (`reduce_speed`,
  `increase_damping`, `stop_axis`, `alarm`) sa SYMULOWANE (tylko log) -
  brak jakiegokolwiek prawdziwego I/O do sterownika/magistrali.
- **Mosty integracyjne** (`bridges/`) sa stubami dry-run - zaden nie
  laczy sie z prawdziwym ROS2/MQTT/OPC-UA. Wypelnienie `_publish_real()`/
  `_update_real()`/`_init_real_*()` w kazdym pliku to jawnie oznaczone
  TODO na przyszlosc.
- **`SlidingWindowAnalyzer`** (streaming) re-kalibruje `omega^2` na
  NAJSTARSZYCH probkach w oknie kroczacym, nie na jednym stalym "zdrowym
  poczatku" - patrz ograniczenie opisane w `streaming.py`.
- **`baseline_residual()`** uzywa okna SYMETRYCZNEGO (patrzy w obie
  strony w czasie) - dobre do analizy juz zakonczonej serii, ZLE do
  prawdziwego strumieniowania w czasie rzeczywistym bez modyfikacji
  (potrzebowalby wersji wylacznie-wstecznej, jak `_rolling_percentile_
  spread()` w `core.py`).
- **`spectral_energy_drift()`** to generyczny przesiewowy wskaznik, NIE
  certyfikowana diagnostyka lozysk (BPFO/BPFI) ani prawdziwa MCSA - patrz
  ograniczenie w docstringu `subsystem_core.py`.

## Historia poprawek

1. **Falszywe wykrywanie defektu na KAZDEJ, w tym czystej, osi** (znaleziono
   podczas pierwszego uruchomienia `demo/run_demo.py` po napisaniu
   szkieletu - `negative_control_check` dawal `false_positive_rate=1.00`
   na wszystkich 6 osiach). Przyczyna: pierwsza wersja `analyze_axis()`
   uzywala `defect()` (port z TIMDR-Grid-Monitor, zaprojektowany dla
   sygnalow w przewazajacej czesci PLASKICH, np. napiecie sieci) wprost
   na surowej, oscylujacej pozycji osi w ruchu - lokalny rozstep roznic w
   oknie 20 probek byl tego samego rzedu co sam normalny krok ruchu, wiec
   prog wychodzil mniejszy niz zwykly krok i ~50% probek bylo falszywie
   flagowanych. Naprawiono zastepujac `defect()` na pozycji modelem
   fizycznym (`harmonic_law_residual()` - prawo oscylatora harmonicznego
   dopasowane na oknie kalibracyjnym), ktorego reszta jest bliska zeru
   dla gladkiego ruchu niezaleznie od amplitudy. Po poprawce:
   `false_positive_rate=0.00` na tych samych danych. Zweryfikowane
   automatycznym testem regresji (`tests/test_core.py::
   test_clean_moving_axis_has_no_harmonic_anomalies` i
   `test_negative_control_false_positive_rate_is_low_on_clean_axis`).
2. **`ringdown_resonance()` nie wykrywal wstrzykniętego rezonansu na
   niektorych ziarnach losowosci** (znaleziono pisac test
   `test_resonance_burst_is_detected_and_is_oscillatory` z innym ziarnem
   niz uzyte w rozwoju - `is_oscillatory` wychodzilo `False` mimo
   wyraznie wstrzykniętego impulsu rezonansowego). Przyczyna: funkcja
   byla wywolywana na SUROWYM przyspieszeniu, ktore samo w sobie oscyluje
   (ruch osi) - okno przed-zdarzeniowe (`pre_event_window`) nie mialo
   stabilnego poziomu odniesienia, ktorego funkcja wymaga (byla
   zwalidowana na sygnalach z plaskim tlem przed zdarzeniem), przez co
   `noise_floor` byl zawyzony przez naturalna krzywizne sinusoidy ruchu.
   Naprawiono uruchamiajac `ringdown_resonance()` na RESZCIE modelu
   harmonicznego (ktora jest ~0 poza zdarzeniami) zamiast na surowym
   przyspieszeniu. Po poprawce: czestotliwosc odzyskiwana konsekwentnie
   w zakresie 14.3-15.1 Hz (prawdziwa: 15 Hz) na 4 niezaleznych ziarnach.
3. **`core.defect()` (domyslne okno) nie wykrywal `slip_event` (chwytak)
   ani `voltage_sag` (zasilanie)** - znaleziono empirycznie od razu przy
   pierwszym teście tych detektorow. Przyczyna: wstrzykniety impuls byl
   SZEROKI wzgledem domyslnego okna `defect()` (window=20) - lokalny
   rozstep roznic w tym oknie byl juz "wewnatrz" samego impulsu, wiec sam
   siebie maskowal (rozne zrodlo tego samego rodzaju problemu co punkt 1,
   ale tym razem szerokosc okna, nie ksztalt sygnalu). Naprawiono
   `baseline_residual()` z DLUZSZYM oknem (>> szerokosc impulsu) - reszta
   wzgledem szerokiej ruchomej mediany nie ma tego problemu.
4. **`anomalies()`/MAD-z falszywie alarmowal na `spectral_energy_drift()`
   na czystych danych** (6 z 30 okien flagowanych na czystym sygnale).
   Przyczyna: przy malej liczbie okien (~30) i silnie skosnej, bliskiej
   zeru statystyce (udzial energii wysokoczestotliwosciowej) MAD jest
   mikroskopijny, wiec zwykle wahania miedzy oknami daja z-score > 4.
   Naprawiono progiem WZGLEDNYM (`ratio_factor * mediana`, z jawna
   podloga) zamiast MAD-z dla tej konkretnej statystyki.
5. **Wstrzykniety `slip_event` w generatorze chwytaka psul realizacje
   szumu PRZED wystapieniem wady** - test porownujacy "przed onsetem
   trajektorie musza byc identyczne" wykryl ~0.75 roznicy prawie 1000
   krokow przed wstrzykniętym zdarzeniem. Przyczyna: `rng.choice()` uzyty
   do losowania centrow impulsow zuzywal/przesuwal stan GLOWNEGO `rng`,
   przez ktory pozniej losowany byl szum pomiarowy - "czysta" i
   "uszkodzona" wersja tej samej trajektorii dostawaly WIEC rozne
   realizacje szumu nawet w okresie, gdzie mialy byc identyczne.
   Naprawiono uzywajac NIEZALEZNEGO RNG (`seed + 1_000_000`) dla wyboru
   centrow zdarzen, nie ruszajacego stanu glownego `rng`.

## Testy

```
pip install -r requirements.txt
pytest tests/ -q
```

106 testow, wszystkie zielone w chwili napisania tej sekcji README -
patrz `tests/` po pelna liste: geometria/Freneta-Serreta na helisie
znanej analitycznie, ringdown na syntetycznym oscylatorze tlumionym,
sensor_bus/subsystems (4 nowe podsystemy), core.py i subsystem_core.py
wliczajac wszystkie regresyjne testy z Historii poprawek powyzej,
status.py (w tym generyczny `compute_component_status`/
`compute_power_status`), control_bridge.py, fleet.py + fleet_demo.py,
bridges/ (3 stuby, wszystkie w trybie dry-run w tym srodowisku),
streaming.py (replay probka-po-probce), api.py (w tym test braku CDN,
`/api/subsystems`, `/api/component/{id}`, `/api/fleet`).

## Co dalej (POZA zakresem tego szkieletu)

- Podlaczenie prawdziwego sterownika/magistrali (ROS, CAN, EtherCAT) w
  miejsce `sensor_bus.py`/`subsystems.py` - `bridges/` daje kontrakt, nie
  dzialajaca integracje (patrz TODO w kazdym pliku `bridges/*_stub.py`).
- Prawdziwe strumieniowanie z sprzetu (obecnie: replay syntetycznej,
  wczesniej wygenerowanej serii probka-po-probce przez
  `SlidingWindowAnalyzer` - krok w strone streamingu, nie polaczenie z
  zywym sprzetem).
- Model referencyjny ruchu wykraczajacy poza pojedyncza sinusoide
  (prawdziwy trapez predkosci, wielosegmentowe trajektorie) - dotyczy
  zarowno `analyze_axis()` jak i kalibracji w oknie kroczacym.
- Formalny test negative control (permutacyjny/null-model) zamiast
  obecnego podstawowego testu poczytalnosci.
- Kalibracja progow (`torsion_factor`, `harmonic_anomaly_factor`,
  `harmonic_anomaly_threshold`, progi podsystemow) na realnych danych z
  prawdziwego robota - obecne wartosci sa dobrane tak, by dzialac na TYM
  konkretnym syntetycznym scenariuszu, nie zwalidowane szerzej.
- Certyfikowana diagnostyka lozysk (BPFO/BPFI z geometrii lozyska) i
  prawdziwa MCSA zamiast obecnego generycznego `spectral_energy_drift()`.
- Dashboard: wykresy dla floty (obecnie tylko kafelki statusu, bez
  wykresow czasowych per robot).
