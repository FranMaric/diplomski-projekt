# Git, Docker i letimo

## Git

Sustav za **verzioniranje koda**. Neizbježan alat ako više ljudi razvija programski kod. Najčešće se koristi u terminalu, ali postoje i GUI aplikacije.

Najčešće naredbe (nemojte ih štrebat, susret ćete ih previše puta):
- `git clone git@github.com:AutorImeRepozitorija` - klonira repozitorij lokalno na vaše računalo,
- `git pull origin ImeGrane` - preuzimate kod (i promijene u njemu) s Git-a s grane ImeGrane,
- `git status` - pregled promijena koje ste radili naspram onima koje ste preuzeli s Git-a,
- `git add ImeDatoteke` - priprema sve promijenjene datoteke koje želite poslati na Git,
- `git commit -m "Poruka"` - U navodnicima ide poruka što rade promijene (preporuka u imperativu),
- `git push origin ImeGrane` - Šaljete (pushate) svoje promijene na Git na željenu granu ImeGrane,
- `git checkout ImeGrane` - promijena na drugu granu,
- `git branch -b ImeNoveGrane` - stvaranje nove grane (budite oprezni trebate pushati tu promijenu da bude vidljiva).

Postoji još puno naredbi koje možete potražiti na internetu. Za početak predlažem mergeanje izravno na githubu.

Postavljanje ssh ključa. Olakšava korištenje git-a. [HOWTO!](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
- ```bash 
  ssh-keygen -t ed25519 -C "ivo.ivic@gmail.com"
  ```
- ```bash 
  eval "$(ssh-agent -s)"
  ```
- ```bash
  ssh-add ~/.ssh/id_ed25519
  ```
- ```bash
  cat ~/.ssh/id_ed25519.pub
  ```
- string koji je iskočio kopirajte i zalijepite u githubu ssh-key settings.

**NOTA BENE**
- nakon promijena uvijek je dobro pregledati koje su stvari izmjenile ili su nove s naredbom:
    
    ```bash
    git status
    ```
- nakon svojih promijena uvijek pushajte svoj kod, najčešća špranca prilikom pushanja je da pushate sve svoje promijene:
    
    ```bash
    git add .
    git commit -m "Add new ROS node for path planning"
    git push origin master
    ```
- ako znate da je netko drugi radio promijene preuzmite (pullajte) kod, pogotovo ako radite na istoj grani:

    ```bash
    git pull origin master
    ```

Česte prepreke:
- konflikti prilikom mergeanja,
- pushanje, a nije napravljen pull,

## Docker

Docker je detaljnije objašnjen u [wiki](https://github.com/larics/docker_files/wiki). Predlažem da prethodni link pročitate u slobodno vrijeme.

Trenutno na ovoj razini možete svhatiti docker kao vrstu virtualne mašine (iako krivo dovoljno dobro prezentira ideju). Koristi se da na brz, efikasan i poprilično lagan način prenosime sustav s jednog robota na drugog. 

Primjer, na letjelici smo razvijali sustav cijeli preddiplomski projekt i instalirali smo puno novih biblioteka, posložili puno raznih skripti u sustavu i dva tjedna prije predaje letjelica padne u vodu i sve izgubimo. Koliko nam treba vremena da ponovo posložimo sve to? Sjećamo li se uopće svih promijena? Rješenje: Ako sve te stvari spremimo u docker, na novu letjelicu stavimo sustav u manje od 5min. 

![](./figure1.png)

Najčešće naredbe:
- `docker ps -a` - ispiše sve instalirane kontejnere te kad je koji pokrenut,
- `docker image -ls` - ispiše sve docker image koji postoje na računalu,
- `docker build` - iz Dockerfilea stvaramo Imagem,
- `docker run` - iz Imagea stvarmo Container,
- `docker exec -it ImeContainera bash` - pokrećemo container.

Na ovoj razini dovoljno je pratiti naredbe koje će biti u sljedećem poglavlju. Docker postaje/je neizbježan alat u svim granama računarstva, telekomunikacija, robotike i sl.

## Hands-on #1

1. Kloniranje repozitorija:
    ```bash
    git clone git@github.com:JakobDomislovic/PredDiplProj2024.git
    cd PredDiplProj2024/
    ```
2. Podijela u tri tima:
    - gripper
    - trajectory
    - vision
3. Svaki tim (voditelj tima) neka napravi direktorij (folder) unutar repozitorija:
    ```bash
    mkdir gripper
    ```
4. Uđite u repozitorij, napravite README.md te u njega zapišite ime time i članove:
    ```bash
    cd Gripper
    nano README.md
    ```
5. Pushajte svoje promijene na github (NB: Morate natrag ući u glavni direktorij PredDiplProj2024 jer je tamo file .git koji zna kako da se to prebaci na github, drugi repozitoriji to nemaju!):
    ```bash
    cd .. # prebacivanje u direktorij ispod
    git status # pregledajte svoje promijene
    git add . 
    git commit -m "Add Gripper team!"
    git push origin master
    ```
6. README.md unutar svojeg direktorija koristite kao dnevnik rada (weekly report). Ne napraviti ništa je isto ok ako postoji valjani razlog :-). 
7. U nastavku ćemo stvoriti docker container koji će se koristiti za simulaciju, sve skripte su unaprijed pripremljene, samo ih pokrenite.
8. U terminal upišite naredbu (radi uštede vremena ovo napravite doma prije sami):
    ```bash
    docker pull lmark1/uav_ros_simulation:focal-bin-0.2.1
    ```
9. Stvaranje slike iz Dockerfilea, to jest buildanje:
    ```bash
    ./docker_build.sh
    ```
10. Pokretanje containera (PRVI PUT OBAVEZNO, dalje preporuceno koristiti ovu skriptu):
    ```bash
    ./docker_run.sh
    ```

## Flight stack
U progamiranju ćete često doći do termina *stack*, to ustvari prestavlja programske blokove koji mogu raditi samostalno, ali kada se povežu čine jedan veći sustav. U nastavku će primjer takvog stacka biti LARICS-ov stack za autonomno letenje.

Stack je stavljen u docker container te se može lako preuzeti, u njemu su osnovni dijelovi koji su potrebni da bi letjelica letila. U nastavku je slika koja prikazuje dijelove stacka, ne trebate ih znati sve nego je samo dato kao primjer.
![](./stack.png)

S obzirom da su roboti skupi, a i uvijek smo kratki s vremenom, praksa je da se u robotici sve prvo isproba u simulaciji. Zato ćemo mi koristiti isti taj stack, ali već pretvoren u simulaciju.

### TMUX
[Tutorail za tmux](https://github.com/larics/uav_ros_simulation/blob/main/HOWTO.md).

## Hands-on #2 
1. Simulacija se pokreće u startup-u. Startup koristimo za različite situacije, npr. testiramo u simulaciji, nakon toga isti kod želimo testirati u laboratoriju i onda opet taj isti kod negdje u šumi. U startup-u ćemo efektivno mjenjati "svijet" i ponašanje letjelice.

    ```bash
    ./startup/simulation/start.sh
    ```

2. Gibajte se kroz tmux da pohvatate komande.

3. Na topic `/UAV_NAMESPACE/tracker/input_pose` posaljite tocku na koju zelite da letjelica ide. Koji je UAV_NAMESPACE?

3. Ugasite tmux.

4. Timovi (voditelji timova) *trajectory* i *vision* neka od svojih direktorija naprave ROS paket. Primjer:
    ```bash
    catkin_create_pkg gripper std_msgs rospy roscpp
    ```
5. Uđite u novo napravljeni paket iz koraka 4.
    ```bash
    cd gripper
    ```
6. Paket je potrebno buildati:
    ```bash
    catkin build --this
    ```
7. Da bi globalno mogli vidjeti svoj paket, moramo ga sourceati:
    ```bash
    roscd sim_ws
    source devel/setup.bash
    ```
8. Testirajte je li dobro sourceano sa (ako nalazi je ok, ako ne imamo grešku):
    ```bash
    roscd gripper
    ```
9. Timovi koji su radili paket sada trebaju pushati svoje promijene. Ponavljam **.git** datoteka se nalazi u glavnom direktoriju PredDiplProj2024 i stoga se uvijek morate pozicionirati tamo kada nešto pushate.
    ```bash
    git status # pogledajte promijene
    git add .
    git commit -m "create ROS package for team Gripper"
    git push origin master
    ```

10. Nagradno samo za tim trajecotry. Premjestiti startup u vaš ROS paket i pushati svoje promijene na github. 

Citirati:
```@article{Markovic2023TowardsAS,
  title={Towards A Standardized Aerial Platform: ICUAS’22 Firefighting Competition},
  author={Lovro Markovic and Frano Petric and Antun Ivanovic and Jurica Goricanec and Marko Car and Matko Orsag and Stjepan Bogdan},
  journal={Journal of Intelligent \& Robotic Systems},
  year={2023},
  volume={108},
  pages={1-13},
  url={https://api.semanticscholar.org/CorpusID:259503531}
}
```