
# Mobilná robotika – Zadania 1, 2, 3, 4, 5

Projekt implementuje lokalizáciu, navigáciu mapovanie, plánovanie trajektórie a Monte Carlo lokalizáciu mobilného robota Kobuki pomocou LiDAR senzora.

---

# Obsah

1. Úloha 1 – Lokalizácia a polohovanie
2. Úloha 2 - Navigácia
3. Úloha 3 – Mapovanie priestoru
4. Úloha 4 – Plánovanie trajektórie
5. Úloha 5 – Monte Carlo lokalizácia (Particle Filter)

---
Úloha 1 – Lokalizácia a polohovanie
---
Funkcia:
```
void robot::uloha_1(const TKobukiData &robotdata)
```
Popis

Úloha implementuje:

odometriu robota
výpočet polohy:
x
y
fi
reguláciu pohybu robota do cieľa
rampovanie rýchlosti
uchovávanie histórie trajektórie
Výpočet orientácie

Orientácia robota sa získava z gyroskopu:

```
double gyro_angle = ((robotdata.GyroAngle / 100.0) / 360.0) * (2 * M_PI);
```

Následne sa:
normalizuje do intervalu <-π, π>
vypočíta zmena uhla:

```
delta_fi = fi_now - fi_prev;
```

Výpočet prejdenej vzdialenosti

Z encoderov sa zistí:
```
deltaLeft
deltaRight
```
Prepočet na metre:
```
lengthLeft = deltaLeft * tickToMeter;
lengthRight = deltaRight * tickToMeter;
```
Celkový posun:
```
length = (lengthLeft + lengthRight) / 2.0;
```
Aktualizácia polohy

Výpočet novej polohy:
```
x += length * cos(fi);
y += length * sin(fi);
```
Regulátor pohybu

Použitý je združený regulátor:

Lineárna chyba
```
l_error = sqrt(dx² + dy²)
```
Uhlová chyba
```
w_error = atan2(dy, dx) - fi
```
Regulácia rýchlosti

Použité P regulátory:
```
P_v = 500
P_w = 5
```
Výstup:
```
aim_v
aim_w
```
Rampovanie

Rýchlosti sa nemenia okamžite.

Používa sa:
```
maxAccV
maxAccW
```
čo zabezpečuje plynulé zrýchlenie.

História trajektórie

Robot si uchováva históriu pozícií:
```
std::vector<Pose> poseHistory;
```
Používa sa pre:

mapovanie
interpoláciu
synchronizáciu LiDAR dát

## Úloha 2 - navigácia

Na lokálnu navigáciu používame VFH+ histrogram, na ktorý nepotrebujeme poznať mapu, funguje iba na základe lidaru.

Algoritmu slúži ako doplnok k navigáciu na známej mape, keďže sám o sebe nedokáže nájsť cieľ v zložitejšej mape, ale dokáže obísť prekážky, ktoré sa nenachádzajú na mape. 

### Fungovanie algorimtu

Priestor okolo robota rozdeľujeme do n = 20 sektorov, ktoré následne zisťujeme či sú obsadené, alebo nie:


```
pre každý laser v lidare: 
    ak je meraná vzdialenosť v špecifikovanom rozsahu: 
        rozšír veľkosť bodu
        priraď jeho vzdialenosť do každého sektoru, kde zasahuje 

pre každý sektor:
    ak je nad hornou hranicou:
        nastav jeho hodnotu na obsadený
    ak je pod spodnou hranocou:
        nastav jeho hodnotu na voľný
```

Potom určíme kandidátske smery, kde kandidátsky smer funguje ako čiastkový cieľ vždy 1 meter od robota:

```
ak je smer k finálnemu cieľu voľný:
    vytvor kandidátsky smer na cieľ 

pre každý sektor: 
    ak je voľný sektor: 
        začni novu oblasť, alebo rožšír existujúcu
    ak je obsadený: 
        skonči oblasť
        vytvor kandidátke smery tak, že: 
            ak je oblasť menšia-rovná ako 3 sektory: 
                vytvor jeden kandidátsky smer v strede
            ak je oblasť väčšia:
                vytvor 2 kandidátske smery pri krajoch
```

Nakoniec vyberieme minimalizačnou funkciou kandidátsky smer, ktorý budeme používať: 

```
f = k1 * natočenie_na_aktuálne_natočenie_robota + k2 * natočenie_na_predchádzajúci_cieľ + k3 * natočenie_na_finálny_cieľ; 
```

Keď sa robot dostane dostatočne blízko cieľa, navigáciu vypneme a robot dokončí pohyb základným polohovaním. 

Algoritmus beži v reálnom čase, vyhodnocuje sa vo frekvencii robota, čím zabezpečujeme plynulosť pohybu. 


Úloha 3 – Mapovanie priestoru
---
Funkcia:
```
void robot::uloha_3(const std::vector<LaserData>& laserData)
```
Popis

Úloha vytvára occupancy grid mapu pomocou LiDAR dát.

Mapa:
```
map[280][280]
```
Rozlíšenie:

5 cm / bunka
Spracovanie LiDAR dát

Každý bod:

sa prevedie do globálnych súradníc
zapíše sa do mapy

Transformácia:
```
tx = xk + dist * cos(fi - angle)
ty = yk + dist * sin(fi - angle)
```
Interpolácia pozície kubická

```
double interpolate(double p0, double p1, double p2, double p3, double t)
```
Vďaka tomu sú LiDAR dáta synchronizované s pohybom robota.

Tvorba occupancy mapy

Ak sa bod opakovane objaví:
```
map_temp[i][j]++
```
Po prekročení limitu:
```
map[i][j] = 1;
```
Bunka sa označí ako prekážka.

Export mapy

Mapa sa exportuje do CSV:
```
exportMapToCSV("final_mapa.csv");
```
Formát:
```
0,0,0,1,1,0,...
```
Vizualizácia mapy

Funkcia:
```
vykresliMapu()
```
Vykresľuje:
```
čierna = steny
modrá = particles
zelená = odhadnutá poloha
červená = cieľ
```
Import mapy

Funkcia:
```
importMapFromCSV(...)
```
Načíta mapu zo súboru CSV späť do programu.

Úloha 4 – Plánovanie trajektórie
---
Funkcia:
```
void robot::uloha_4()
```
Popis

Implementované je:

rozšírenie stien
flood fill algoritmus
plánovanie trajektórie
zjednodušenie trajektórie
Rozšírenie prekážok

Steny sa zväčšia:
```
n = 4
```
kvôli bezpečnej navigácii robota.

Flood Fill algoritmus

Cieľ dostane hodnotu:

2

Následne sa šíri vlna:
```
aktualna_vlna++
```
kým sa nenájde robot.

Extrakcia trajektórie

Po nájdení cesty sa:

prechádza späť po menších hodnotách
ukladajú sa waypointy
```
planovana_cesta.push_back(...)
```
Zjednodušenie trajektórie

Odstraňujú sa zbytočné body na priamke.

Používa sa determinant:

plocha

Ak:
```
plocha ≈ 0
```
bod sa odstráni.

Navigácia po trajektórii

Robot:

sleduje waypointy
po priblížení:
```
vzdialenost < 0.10
```
prejde na ďalší bod.

Úloha 5 – Monte Carlo lokalizácia
---
Funkcia:
```
void robot::uloha_5(const std::vector<LaserData>& laserData)
```
Popis

Implementovaný je Particle Filter.

Používa:

particles
váhy
resampling
motion model
sensor model
Inicializácia particles

Vygenerujú sa náhodné particles:
```
Particle p;
```
Obsahujú:
```
x
y
fi
weight
```
Sensor model

Pre každú particle:

sa simuluje LiDAR
porovná sa s reálnym skenom

Používa sa:
```
expectedRangeFromMapBresenham(...)
```
Výpočet váhy

Použitá je priemerná chyba:
```
avg_error
```
Váha:
```
weight = exp(-avg_error / 0.1)
```
Normalizácia

Všetky váhy sa normalizujú:
```
weight /= totalWeight;
```
Odhad polohy

Najlepšia particle:
```
best_particle
```
určuje:
```
estimatedX
estimatedY
estimatedFi
```
Resampling

Použitý je:

ruletový výber
jitter
náhodní prieskumníci
Motion model

Funkcia:
```
uloha_5_pohyb(...)
```
Simuluje:
```
transláciu
rotáciu
šum pohybu
```
Bresenham raycasting

Funkcia:
```
expectedRangeFromMapBresenham(...)
```
Simuluje laserový lúč v mape.

Používa:

Bresenham algoritmus
occupancy grid mapu
Prepnutie na Monte Carlo pozíciu

Funkcia:
```
useMonteCarloPose()
```
Prepíše:
```
x
y
fi
```
na:
```
estimatedX
estimatedY
estimatedFi
```
