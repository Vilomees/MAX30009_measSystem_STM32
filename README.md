# MAX30009 impedantsi mõõtesüsteem STM32 platvormil

See repositoorium sisaldab lõputöö raames loodud tarkvara MAX30009 BioZ analoogesiosa juhtimiseks ja impedantsi mõõtmiseks STM32WB55 mikrokontrolleriga. Süsteem koosneb kahest põhilisest osast:

- **STM32 firmware** – mikrokontrolleri kood MAX30009 seadistamiseks, mõõtmise käivitamiseks ja RAW-andmete edastamiseks.
- **PC GUI** – Python/PyQt5 põhine arvutipoolne juhtpaneel mõõtmiste seadistamiseks, käivitamiseks, kuvamiseks ja salvestamiseks.

## Projekti üldine eesmärk

Tarkvara võimaldab juhtida MAX30009 BioZ AFE kiipi, muuta mõõtesagedust, ergutusvoolu, võimendust ja filtreid ning koguda impedantsi mõõtmisega seotud real- ja imaginaarosa RAW-väärtusi. Andmed edastatakse STM32-lt arvutisse UART-liidese kaudu ning arvutipoolne GUI võimaldab neid mõõtetulemusi mugavamalt jälgida ja salvestada.

## Soovituslik repositooriumi struktuur

```text
MAX30009_measSystem_STM32/
│
├── firmware/
│   ├── Src/
│   │   ├── main.c
│   │   ├── MR_MAX30009.c
│   │   ├── stm32wbxx_hal_msp.c
│   │   ├── stm32wbxx_it.c
│   │   ├── syscalls.c
│   │   ├── sysmem.c
│   │   └── system_stm32wbxx.c
│   │
│   ├── Inc/
│   │   ├── main.h
│   │   ├── MR_MAX30009.h
│   │   ├── MR_MAX30009_DEF.h
│   │   ├── stm32wbxx_hal_conf.h
│   │   └── stm32wbxx_it.h
│   │
│   ├── MAX30009_stm32.ioc
│   ├── STM32WB55RGVX_FLASH.ld
│   └── STM32WB55RGVX_RAM.ld
│
├── pc_gui/
│   └── max30009_gui.py
│
├── requirements.txt
└── README.md
```

Kui projekt on juba STM32CubeIDE poolt loodud täisprojektina, võivad lisaks olemas olla ka `Drivers/`, `Startup/`, `.project` ja `.cproject` failid. Neid võib hoida firmware kaustas, kui eesmärk on, et projekt oleks teises arvutis kohe STM32CubeIDE-s avatav ja kompileeritav.

## Riistvara

Projekt on koostatud järgmise riistvara jaoks:

- STM32WB55RGVx mikrokontroller
- MAX30009 BioZ AFE
- SPI-liides MAX30009 registrite ja FIFO lugemiseks
- UART-liides arvutipoolse GUI-ga suhtlemiseks
- MAX30009 INT-signaal FIFO/status katkestuse tuvastamiseks

Peamised ühendused firmware konfiguratsiooni järgi:

| Signaal | STM32 viik | Kirjeldus |
|---|---:|---|
| SPI1 SCK | PA5 | MAX30009 SPI kell |
| SPI1 MISO | PA6 | MAX30009 andmed STM32 suunas |
| SPI1 MOSI | PA7 | STM32 andmed MAX30009 suunas |
| Z_CS | PA4 | MAX30009 chip select |
| MAX30009_INT | PB0 | MAX30009 katkestuse sisend |
| USART1_TX | PB6 | UART saatmine arvutisse |
| USART1_RX | PB7 | UART vastuvõtt arvutist |

## Firmware kirjeldus

Firmware põhifailid asuvad `firmware/Src` ja `firmware/Inc` kaustades.

Olulisemad failid:

| Fail | Kirjeldus |
|---|---|
| `main.c` | Põhiprogramm, UART käsuliides, mõõtmistsükkel ja RAW skaneeringu juhtimine |
| `MR_MAX30009.c` | MAX30009 registrite, sageduste, ergutusvoolu, võimenduse, filtrite ja FIFO lugemise funktsioonid |
| `MR_MAX30009.h` | MAX30009 funktsioonide deklaratsioonid |
| `MR_MAX30009_DEF.h` | MAX30009 registrite, bittide, struktuuride ja konstantide definitsioonid |
| `main.h` | STM32 viikude definitsioonid, sh CS ja INT signaalid |
| `MAX30009_stm32.ioc` | STM32CubeMX konfiguratsioonifail |
| `STM32WB55RGVX_FLASH.ld` | Flash-mälu linker script |
| `STM32WB55RGVX_RAM.ld` | RAM linker script |

Firmware töötab põhimõttel, et arvuti saadab UART kaudu tekstipõhiseid käske. STM32 muudab nende põhjal MAX30009 seadistust, käivitab mõõtmise ja saadab tulemused tagasi tekstiridadena.

## UART käsud

Firmware toetab järgmisi käske:

| Käsk | Tähendus |
|---|---|
| `F<idx>` | Määrab sagedusindeksi, vahemik 0...59 |
| `I<idx>` | Määrab ergutusvoolu indeksi, vahemik 0...15 |
| `G<idx>` | Määrab võimenduse indeksi, vahemik 0...3 |
| `H<idx>` | Määrab analoogse kõrgpääsfiltri seadistuse |
| `D<dhpf> <dlpf>` | Määrab digitaalse kõrg- ja madalpääsfiltri |
| `A<rge>` | Määrab BioZ võimendi vahemiku |
| `W<bw>` | Määrab BioZ võimendi ribalaiuse |
| `C<0/1>` | Lülitab DC restore seadistust |
| `E<0/1>` | Lülitab välise kondensaatori seadistust |
| `Q<hex_addr>` | Loeb MAX30009 registri väärtuse |
| `X<hex_addr> <hex_val>` | Kirjutab MAX30009 registrisse väärtuse |
| `S` | Käivitab mõõtmise |
| `P` | Peatab mõõtmise |
| `R[a [b]]` | Käivitab RAW skaneeringu sagedusvahemikus `a...b` |
| `?` | Kuvab aktiivse konfiguratsiooni ja olulisemad registrid |
| `#` | Kuvab käsulisti |

RAW skaneeringu väljund kasutab CSV-laadset formaati, näiteks:

```text
SCAN_START,...
RAW,freq_idx,freq_hz,sample_no,real,imag,flags
FREQ_DONE,...
SCAN_END
```

## PC GUI kirjeldus

Arvutipoolne juhtpaneel asub failis:

```text
pc_gui/max30009_gui.py
```

GUI on kirjutatud Pythonis ning kasutab PyQt5 kasutajaliidest. GUI eesmärk on lihtsustada STM32/MAX30009 mõõtesüsteemi juhtimist ja mõõtetulemuste salvestamist.

GUI põhifunktsioonid:

- jadapordi valimine ja ühenduse loomine;
- mõõtekonfiguratsiooni muutmine;
- ühe sageduse mõõtmise käivitamine;
- spektroskoopia ehk mitme sageduse RAW skaneeringu käivitamine;
- UART logi vaatamine;
- mõõtetulemuste eksport CSV-faili;
- kalibreerimisega seotud töövoogude toetamine.

## Python sõltuvuste paigaldamine

GUI käivitamiseks on soovitatav kasutada virtuaalkeskkonda.

macOS/Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## GUI käivitamine

Kui GUI fail asub `pc_gui` kaustas:

```bash
python pc_gui/max30009_gui.py
```

Kui GUI fail asub repositooriumi juurkaustas:

```bash
python max30009_gui.py
```

## Firmware kasutamine STM32CubeIDE-s

1. Ava projekt STM32CubeIDE-s.
2. Vajadusel ava `MAX30009_stm32.ioc` fail ja genereeri projekt uuesti.
3. Kontrolli, et SPI1, USART1 ja GPIO viigud vastaksid kasutatavale riistvarale.
4. Kompileeri projekt.
5. Laadi firmware STM32 plaadile.
6. Ava PC GUI ja vali õige jadaport.

UART seadistus firmware põhjal:

```text
Baudrate: 1000000
Data bits: 8
Parity: none
Stop bits: 1
Flow control: none
```

## GitHubi mitte lisatavad failid

Repositooriumi ei ole mõistlik lisada build'i ja ajutisi faile, näiteks:

```text
Debug/
Release/
*.elf
*.bin
*.hex
*.map
*.o
*.d
.DS_Store
__pycache__/
.venv/
```

Need failid ei ole lähtekoodi mõttes vajalikud ja tekitavad GitHubis liigset müra.

## Märkus

See repositoorium on koostatud lõputöö tarkvaralise osa dokumenteerimiseks. Firmware ja GUI moodustavad koos mõõtesüsteemi, kus STM32 juhib MAX30009 kiipi ning arvutipoolne Python GUI võimaldab mõõtmist mugavalt seadistada, käivitada ja tulemusi salvestada.

