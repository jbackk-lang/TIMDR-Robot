# TIMDR-Robot

Szkielet warstwy TIMDR dla robota wieloosiowego (np. ramienia) - wykrywanie
anomalii na poszczegolnych osiach na podstawie syntetycznych danych z
enkoderow/IMU/czujnikow sily, z (symulowanym) mostkiem do sterownika
robota.

**STATUS: SZKIELET + DANE SYNTETYCZNE.** Zero testow na prawdziwym
robocie/sprzecie w chwili napisania tego kodu. Zgodnie z ustalonym
podejsciem ("najpierw szkielet + dane syntetyczne, potem realne testy")
ten etap celowo NIE laczy sie z zadnym prawdziwym sterownikiem, magistrala
(CAN/EtherCAT/ROS) ani czujnikiem. Wszystkie liczby w dashboardzie i
demo pochodza z `timdr_robot/sensor_bus.py` (generator syntetyczny), nie
z pomiarow.

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
   `api.py`/`static/index.html` (dashboard).

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
- **Brak strumieniowania w czasie rzeczywistym**: `SensorBus` trzyma
  cala wygenerowana z gory serie, nie strumieniuje probek na biezaco z
  prawdziwego sprzetu - to kolejny krok, jesli/gdy projekt przejdzie do
  fazy realnych testow.

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

## Testy

```
pip install -r requirements.txt
pytest tests/ -q
```

44 testy, wszystkie zielone w chwili napisania tego README - patrz
`tests/` po pelna liste (geometria/Freneta-Serreta na helisie znanej
analitycznie, ringdown na syntetycznym oscylatorze tlumionym, sensor_bus,
core.py wliczajac oba regresyjne testy z Historii poprawek powyzej,
status.py, control_bridge.py, api.py wliczajac test braku CDN).

## Co dalej (POZA zakresem tego szkieletu)

- Podlaczenie prawdziwego sterownika/magistrali (ROS, CAN, EtherCAT) w
  miejsce `sensor_bus.py`.
- Strumieniowe (nie z gory wygenerowane) przetwarzanie probek w czasie
  rzeczywistym.
- Model referencyjny ruchu wykraczajacy poza pojedyncza sinusoide
  (prawdziwy trapez predkosci, wielosegmentowe trajektorie).
- Formalny test negative control (permutacyjny/null-model) zamiast
  obecnego podstawowego testu poczytalnosci.
- Kalibracja progow (`torsion_factor`, `harmonic_anomaly_factor`,
  `harmonic_anomaly_threshold`) na realnych danych z prawdziwego robota -
  obecne wartosci sa dobrane tak, by dzialac na TYM konkretnym
  syntetycznym scenariuszu, nie zwalidowane szerzej.
