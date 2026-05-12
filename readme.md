


## Úloha 2 - navigácia

Na lokálnu navigáciu používame VFH+ histrogram, na ktorý nepotrebujeme poznať mapu, funguje na základe lidaru.  

### Fungovanie algorimtu

Priestor okolo robota rozdeľujeme do 20 sektorov, ktoré následne zisťujeme či sú obsadené, alebo nie:


```

```

Následne spracujeme dynamické obmedznenia robota, teda schopnosť robota obísť prekážku, aj keď je voľný smer:

```

```

Potom určíme kandidátske smery:

```

```

Nakoniec vyberieme minimalizačnou funkciou kandidátsky smer, ktorý budeme používať.

```

```