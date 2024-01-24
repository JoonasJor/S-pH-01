# pH mittari S-pH-01B anturia ja ESP32 käyttäen

#### Käytetyt komponentit:

-S-pH-01B pH anturi - https://www.seeedstudio.com/RS485-pH-Sensor-S-pH-01B-p-4632.html  
-SparkFun Thing Plus ESP32 WROOM (USB-C) - https://www.sparkfun.com/products/20168  
-MAX485 UART <-> RS485 muunnin/lähetin-vastaanotin  
-Painonappi ja piuhat  

Käyttöohjeet, komennot, yms. - https://files.seeedstudio.com/products/101990666/res/RS485%20&%200-2V%20pH%20Sensor%20(S-pH-01)%20-%20User%20Guide%20v2.0.pdf

HUOM. Käyttöohjeissa lukee, että kalibroinnissa pitää kirjoittaa anturin rekisteriin "0xFFFF", mutta tämä ei toimi ja vain sekoittaa anturin. Oikea arvo on "0x7FFF".

### Mittarin kalibrointi:
1. Upota anturi pH 4 liuokseen.
2. Odota hetki kun pH arvo tasoittuu.
3. Siirrä painonapille menevä piuha ESP32 pinniin 15.
4. Paina painonappia ja konsoli ilmoittaa, että pH 4 on kalibroitu.
5. Tee sama pH arvoille 7 ja 10, ainoana erona se, että pH 7 pinni on 32 ja pH 10 pinni on 14.
