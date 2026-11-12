# ARM – Laboratorium 5: Robot poszukujący

## 1. Opis projektu

Celem projektu jest autonomiczne przeszukiwanie środowiska przez robota mobilnego TurtleBot3 Waffle oraz wyznaczanie położenia pięciu losowo rozmieszczonych kostek oznaczonych znacznikami ArUco.

### Działanie programu

Program składa się z dwóch głównych węzłów:

- `mission_commander.py` – odpowiada za poruszanie się robota między wyznaczonymi punktami na mapie, wykonywanie obrotów o 360°, zatrzymywanie robota do pomiaru oraz wznawianie ruchu.
- `vision_detector.py` – odpowiada za wykrywanie znaczników ArUco na obrazie z kamery, wyznaczanie ich położenia za pomocą `solvePnP`, transformację współrzędnych do układu mapy przy użyciu TF2 oraz zapis wyników do pliku CSV.

Robot porusza się po trasie składającej się z 12 punktów nawigacyjnych. Po osiągnięciu każdego punktu wykonuje obrót o 360° w celu obserwacji otoczenia. Po wykryciu nowej kostki robot przerywa aktualną akcję i zatrzymuje się. Następnie na podstawie jednej nowej klatki obrazu wyznaczane jest położenie znacznika. Po zapisaniu współrzędnych robot wznawia przerwaną akcję. Misja kończy się po zlokalizowaniu wszystkich pięciu kostek.

## 2. Przeprowadzone próby

### Próba 1 – wyznaczanie pozycji podczas ruchu

W pierwszej wersji programu położenie kostki wyznaczano na podstawie trzech kolejnych pomiarów, z których obliczano medianę współrzędnych. Robot nie zatrzymywał się podczas wykonywania pomiarów. Uzyskany średni błąd lokalizacji wyniósł **1,93 m**.

### Próba 2 – zatrzymanie robota do pomiaru

W drugiej wersji programu rozdzielono wykrywanie znacznika od właściwego pomiaru jego położenia. Po wykryciu nowej kostki robot zatrzymywał się, a następnie wykonywano pomiar na podstawie jednej świeżej klatki obrazu. Transformację TF2 pobierano dla czasu wykonania tej klatki. Pomimo zatrzymywania robota średni błąd lokalizacji wyniósł **2,11 m**. Wskazywało to, że przyczyny niedokładności należy szukać również w parametrach wykorzystywanych do wyznaczania pozycji znacznika.

### Próba 3 – poprawienie rozmiaru znacznika

W trzeciej próbie przeanalizowano teksturę kostki w celu ustalenia rzeczywistego rozmiaru znacznika ArUco.

W pliku `model.sdf` wymiar całej kostki określono jako:

```xml
<box>
    <size>0.09 0.09 0.09</size>
</box>
```
Na podstawie analizy tekstury otrzymano następujące wartości:

| Parametr | Wartość |
| --- | ---: |
| Bok kostki | 90 mm |
| Rozmiar tekstury | 755 × 755 px |
| Rozmiar znacznika ArUco | 588 × 588 px |

Rzeczywisty bok znacznika obliczono z proporcji:

$$
\frac{L_{ArUco}}{L_{kostki}} = \frac{588}{755}
$$

$$
L_{ArUco} = 0.09 \cdot \frac{588}{755}
$$

$$
L_{ArUco} = 0.07009\ \mathrm{m} \approx 7\ \mathrm{cm}
$$

Wynika z tego, że wcześniej przyjęty rozmiar znacznika był zawyżony, co powodowało systematyczne przeszacowanie odległości od kostek.

W kodzie zmieniono rozmiar znacznika wykorzystywany przez `solvePnP` na 0,07 m:

```python
h = 0.07 / 2

self.object_points = np.array([
    [-h,  h, 0],
    [ h,  h, 0],
    [ h, -h, 0],
    [-h, -h, 0]
], dtype=np.float32)
```
Zachowano zatrzymywanie robota przed wykonaniem pomiaru oraz transformację TF2 odpowiadającą chwili wykonania obrazu.

Uzyskany średni błąd lokalizacji wyniósł **0,355 m**.

### Porównanie prób

| Próba | Rozmiar znacznika | Metoda pomiaru | Średni błąd XY [m] |
| --- | ---: | --- | ---: |
| 1 | 0.09 m | Mediana trzech pomiarów podczas ruchu | 1.930 |
| 2 | 0.09 m | Zatrzymanie robota, jedna klatka | 2.110 |
| 3 | 0.07 m | Zatrzymanie robota, jedna klatka, narożniki subpikselowe | 0.355 |

W kolejnych próbach kostki były rozmieszczane losowo, dlatego uzyskane wyniki nie stanowią bezpośredniego porównania metod dla identycznych warunków początkowych.

## 3. Wyniki końcowe – próba 3

Błąd lokalizacji wyznaczono jako odległość euklidesową pomiędzy pozycją rzeczywistą a pozycją oszacowaną w płaszczyźnie XY:

$$
e_i = \sqrt{(\hat{x}_i-x_i)^2+(\hat{y}_i-y_i)^2}
$$

| ID | X rzeczywiste [m] | Y rzeczywiste [m] | X oszacowane [m] | Y oszacowane [m] | Błąd XY [m] |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | 1.1973 | -4.6916 | 0.5128 | -4.6577 | 0.685 |
| 1 | 7.5948 | -1.8386 | 6.8850 | -1.9912 | 0.726 |
| 2 | 1.3740 | -0.3642 | 1.3411 | -0.3679 | 0.033 |
| 3 | 9.9114 | 0.4996 | 9.8116 | 0.2358 | 0.282 |
| 4 | 3.7023 | -0.3309 | 3.7505 | -0.3396 | 0.049 |

**Podsumowanie wyników:**

- Liczba zlokalizowanych kostek: 5/5.
- Średni błąd lokalizacji: 0.355 m.
- Minimalny błąd lokalizacji: 0.033 m.
- Maksymalny błąd lokalizacji: 0.726 m.

Pliki z wynikami eksperymentu:

- [Pozycje oszacowane – detected_aruco.csv](results/test_04/detected_aruco.csv)
- [Pozycje rzeczywiste – ground_truth.txt](results/test_04/ground_truth.txt)

## 4. Uruchomienie programu

Kompilacja paczki:

```bash
colcon build --symlink-install --packages-select arm05_sim
```

Uruchomienie symulacji Gazebo i Nav2:

```bash
ros2 launch arm05_sim world.launch.py
```

Uruchomienie detektora ArUco:

```bash
ros2 run arm05_sim vision_detector.py --ros-args -p use_sim_time:=true
```

Uruchomienie programu odpowiedzialnego za realizację misji:

```bash
ros2 run arm05_sim mission_commander.py --ros-args -p use_sim_time:=true
```
