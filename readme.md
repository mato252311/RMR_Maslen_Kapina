


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