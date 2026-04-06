Tema A – Evitare cu recuperare
Descriere

Această temă extinde comportamentul reactiv de oprire la obstacol implementat în cerința 3.4 prin introducerea unei mașini de stări care permite robotului să se recupereze atunci când întâlnește un obstacol.

În loc să rămână blocat în fața unui obstacol, robotul execută o manevră automată de ieșire:

merge înapoi pentru o perioadă scurtă
efectuează o rotație aleatorie
reia deplasarea înainte

Scopul este demonstratrea faptului că robotul poate continua navigarea autonomă fără a rămâne blocat permanent.

Arhitectura comportamentului

Controlul robotului este implementat folosind o mașină de stări finite (Finite State Machine) cu trei stări principale:

FORWARD

Robotul se deplasează înainte cu viteză constantă.

Tranziție:

dacă distanța detectată de senzorii frontali este mai mică decât STOP_DISTANCE, robotul trece în starea BACKWARD.
BACKWARD

Robotul merge înapoi pentru aproximativ 1 secundă pentru a se îndepărta de obstacol.

Tranziție:

după expirarea timpului, robotul trece în starea TURNING.
TURNING

Robotul execută o rotație de aproximativ 90° într-o direcție aleasă aleator:

stânga
dreapta

Tranziție:

după terminarea rotației robotul revine în starea FORWARD.
Diagrama de stări
        obstacol
FORWARD ---------> BACKWARD
   ^                  |
   |                  |
   |              1 secunda
   |                  v
   +----------- TURNING
                     |
                     | rotire ~90°
                     v
                 FORWARD
Senzori utilizați

Pentru detectarea obstacolelor sunt folosiți senzorii frontali ai robotului Pioneer P3-DX:

[2, 3, 4, 5]

Acești senzori detectează obstacolele aflate în fața robotului.

Distanța minimă detectată este utilizată pentru a decide dacă robotul trebuie să se oprească sau să execute manevra de recuperare.

Parametri importanți
Parametru	Valoare	Descriere
V_FORWARD	2.0 rad/s	viteza de deplasare înainte
V_BACKWARD	-2.0 rad/s	viteza de deplasare înapoi
STOP_DISTANCE	0.5 m	distanța la care se detectează obstacol
BACKWARD_TIME	1 s	durata mersului înapoi
TURN_TIME	~1.57 s	durata rotației aproximative de 90°
Rezultatul comportamentului

Robotul:

merge înainte atunci când drumul este liber
detectează obstacolele folosind senzorii ultrasonici
execută automat o manevră de recuperare atunci când este blocat
continuă navigarea fără a rămâne blocat permanent

Acest comportament demonstrează un exemplu simplu de robot autonom cu comportament reactiv bazat pe senzori.

Observații
Controlul este realizat într-o buclă de control de aproximativ 20 Hz.
Direcția rotației este aleasă aleator pentru a evita situațiile repetitive în care robotul rămâne blocat.
Parametrii de timp și viteza pot fi ajustați experimental pentru o mișcare mai stabilă.
Concluzie

Prin utilizarea unei mașini de stări și a datelor provenite de la senzori, robotul poate reacționa la obstacole și poate continua navigarea autonomă fără a rămâne blocat. Această abordare reprezintă un exemplu simplu de comportament reactiv utilizat în robotică mobilă.