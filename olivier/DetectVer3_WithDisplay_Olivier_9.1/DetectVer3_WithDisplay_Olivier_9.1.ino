

/*
  Readout of the Yan Xu YX-ADboard V1.0 6 channels AD7656 simultaneous ADC by an Arduino DUE board based on ARM CORTEX M3 32 bit processor
  4x20 newhaven oled display  4x20 newhaven oled display

 *****************************************************************newhaven oled display
  OLED pin 1 (Vss)    to Arduino pin ground
  OLED pin 2 (VDD)    to Arduino pin 3.3V
  OLED pin 3 (REGVDD) to Arduino pin ground
  OLED pin 4 to 6     to Vss ground
  OLED pin 7 (SCLK)   to Arduino pin D76 (SCK)
  OLED pin 8 (SID)    to Arduino pin D75 (MOSI)
  OLED pin 9 (SOD)    to Arduino pin D74 (MISO)
  OLED pin 10 to 14   to Vss ground
  OLED pin 15 (/CS)   to Arduino pin D8
  OLED pin 16 (/RES)  to Arduino pin D9
  OLED pin 17 (BS0)   to Vss ground
  OLED pin 18 (BS1)   to Vss ground
  OLED pin 19 (BS2)   to Vss ground
  OLED pin 20 (Vss)   to Vss ground

  ********************************************AD7656***********************************
  DB0 = PC1, 33
  DB1 = PC2, 34
  DB2 = PC3, 35
  DB3 = PC4, 36
  DB4 = PC5, 37
  DB5 = PC6, 38
  DB6 = PC7, 39
  DB7 = PC8, 40
  DB8 = PC9, 41
  DB9 = PC12, 51
  DB10 = PC13, 50
  DB11 = PC14, 49
  DB12 = PC15, 48
  DB13 = PC16, 47
  DB14 = PC17, 46
  DB15 = PC18, 45 //sign

  RESET = PA15, 24
  /CONVST = PD0, 25
  BUSY3 = PD3, 28   //all those Arduino BUSY are the only BUSY of the AD7656, jumper selection
  BUSY2 = PD2, 27
  BUSY1 = PD1, 26
  /CS3 = PA7, 31   //all those Arduino CS are the only CS of the AD7656, jumper selection
  /CS2 = PD9, 30
  /CS1 = PD6, 29
  /READ = PD10, 32

  Counters uses :
  TC0 = hard event counter on pin 22
  TC1 = time measuring counter 
  TC2 = free
  TC3 = sendbuffer at 100Hz Transmission mode for test
  TC4 = convert adc at 100Hz Software convert mode for test
  TC5 = free
  TC6 = free
  TC7 = free
  TC8 = convert adc at 250 kHz (every 4µs)

/////////////////////////////////////////// DOCUMENTATION ///////////////////////////////////////////////////////////////////////
Explications sur le programme : 

Attention, c'est un programme test pour l'utilisation des compteurs et du PWM, 
il fonctionne sans ADC, ni écran, ni usb natif pour le moment.

Pin 53 : sortie PWM commandable via la console. 
Pin 22 : entrée du compteur d'évènements hard (pour test à relié au PWM pin 53)
Pin 11 : sortie de simulation d'event (Test-T-MCP), pulse toutes les 10 ms
Pin 12 : entrée d'interrupetion T_MCP, à relier à Test_T_MCP pin 11 pour test
Pin 10 : sortie de simulation d'un signal busy de l'adc (TEST_BUSY)----- pas utillisé pour le moment
Pin 28 : entrée d'interruption BUSY, à relier à BUSY pin 10 pour test----- pas utillisé pour le moment

Commande console :
0 : stop le PWM
valeur >= 2 : démarre le PWM avec la valeur en Hz entrée en console

Affichage d'information sur la console :
Power up : Premier démarrage
New command : nouvelle valeur de fréquence de sortie pour le PWM (attention toutes les valeurs ne sont pas disponible, ça fonctionne
              par pas de fréquence qui sont de plus en plus grand jusqu'à 42MHz)
Start : démarre le PWM et la simulation Test_T_MCP toutes les 10ms
Stop : Arrête tout
Reset counters : reset all counters
Events software counter : compteur soft incrémenté par la routine d'intérruption MCP_EVENT_ISR (int pin T_MCP), +1 toutes les 10ms
Events hardware counter : compteur hard incrémenté via pin 22, valeur lue par la routine d'intérruption MCP_EVENT_ISR (int pin T_MCP)
                          incrémenté en fonction de la fréquence de sortie du PWM
Time counter value : valeur instantanée du compteur de temps
Number of cycle : nombre de cycle de mesures depuis le démarrage du µc (ici max 5 pour les tests)
Tableau[software counter][hardware counter]
Overflow detected : détected d'un overflow sur le compteur de temps ( après 102 secondes je crois)

START -> PWM -> Hard counter ++
      -> TEST_T_MCP(10ms) -> MCP_EVENT_ISR(T_MCP)-> Time counter reading
                                                 -> Hard counter reading
                                                 -> Soft counter ++
                                                 -> Tempo counter 4µs -> CONVERT_ADC_ISR

Notes diverses :
le reset du compteur hard à un décalage, c'est normal, je vais regarder comment le contourner.

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
// long int est un entier signé sur 32 bit c'est donc un int32_t (8 lettres hexa)
// pour écrire en 32 bit sur le port B   :     REG_PIOB_ODSR = 0x45a12bb2;
// pour lire le port B en 32 bit               uint_32   bid =REG_PIOB_PDSR;
// La lettre B est changée en A, B, C... pour les autres ports

#include <SPI.h>
#include <NewhavenOLED.h>
#include <DueTimer.h>
#include <Arduino.h>

#define SOFTWARE_ACQUISITION false // activate an automated CONV signal at 10Hz
#define TRANSMISSION_ON  false // activate an automated buffer transmission at 10Hz on native USB
#define DEBUG true // activate the debug mode with serial monitor, not implemented for now

#define Buff_char_size 1024 //16384   // 14 bit en char soit 8192 mots de 16 bits, soit 1365.33 evenements de six valeurs ou 2048 ev de 4 valeurs
#define SettoLow_(i) &(~(1<<i))  // utilisé pour forcer un bit à zéro le bit numéro i quelque soit la taille du mot
#define SettoHigh_(i) |(1<<i)    // utilisé pour forcer un bit à un le bit numéro i quelque soit la taille du mot

unsigned long callNumber = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long nextMillis = 0;
unsigned long softISRChangeTime = 0;
unsigned long dataChangeTime = 0;
const long    interval = 1000;



#define Read_A  REG_PIOA_PDSR
#define Read_B  REG_PIOB_PDSR
#define Read_C  REG_PIOC_PDSR
#define Read_D  REG_PIOD_PDSR
#define Write_A REG_PIOA_ODSR
#define Write_B REG_PIOB_ODSR
#define Write_C REG_PIOC_ODSR
#define Write_D REG_PIOD_ODSR
const  uint32_t Un=1;

const int  Pin_Prob1 = 22;    // PORT B26
#define Pin_Prob1_high Write_B |= Un<<26
#define Pin_Prob1_low  Write_B &= ~(Un<<26)

const int  Pin_Prob2 = 23;    // PORT A14
#define Pin_Prob2_high Write_A |= Un<<14
#define Pin_Prob2_low  Write_A &= ~(Un<<14)

const int RESET = 24;
const int CONVST = 25;
const int BUSY_3 = 26;
const int BUSY_2 = 27;
const int BUSY_1 = 28;
const int CS_3 = 29;
const int CS_2 = 30;
const int CS_1 = 31;
const int READ  = 32;

const int T_MCP = 12; // event incomming
const int TEST_T_MCP = 11; // Simulation d'un événement, à relier à T_MCP
const int TEST_BUSY = 10; // Simulation d'un Busy, à relier à BUSY_1

const long maskHigh28 = 1 << 28; // comme les 16bit de l'ADC ne se suivent pas exactement sur les 32bits du port, il faudra faire un regroupement sur 16 bit
const long maskLow28 =  ~maskHigh28;
const long maskHighCS = 1 << 25;
const long maskLowCS = ~maskHighCS;
const long mask1 = 0x0003F800;
const long mask2 = 0x000001FF;

/*********************************************** OLED  ***************************************************/
const byte ROW_N = 4;                 // Number of display rows de l'affichage OLED
const byte COLUMN_N = 20;             // Number of display columns
const byte SCLK = 76;                 // Arduino's pin assigned to the SCLK line
const byte SDIN = 75;                 // Arduino's pin assigned to the SID line
const int  CS_SCREEN = 8;
const int  RST_SCREEN = 9;
byte TEXT1[4][21] = {"--------ISMO--------",
                     "-AD7656 test module-",
                     " 6 x 16 bits SAR ADC",
                     "Trying native  USB  "
                    };         // Strings to be displayed
byte TEXT2[4][21] = {"Events:             ",
                     "Bytes:              ",
                     "                    ",
                     "                    "
                    };         // Strings to be displayed
/********************************************************************************************************/

uint16_t freqInPulse = 65535; //max
uint16_t freqInPulseCmd = 65535; //max
int sendConsign = false;
int incomingByte = 0;
uint32_t freq = 10; //Hz
uint16_t PrescalerList[11] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
uint16_t pwmPrescalerIndex = 11;
boolean newCmd = true;
boolean startAcq = false;
boolean newDataISR = true;
String inString = "";
uint32_t eventHardCounter = 0;  // N_event_hard
uint32_t eventSoftCounter = 0; // N_event_soft
uint32_t eventTimeValue = 0;     // T_event
uint32_t tc0Channel1Status = 0; // for overflow detection

uint32_t cycleNumber = 0;  // only reseted at the µC power up or reset
uint32_t CycleAcquisitionData [2][1000]; // software events counter, hardware events counter

/**************************************************** Buffer de transmission *********************************************************/
uint8_t Buffer[2][Buff_char_size];     // array to hold the data before sending it by packet through USB to the PC

uint16_t* v1 = (uint16_t*) (&(Buffer[0][0]));
uint16_t* vmax1 = v1 + (Buff_char_size / 2); // on doit diviser par deux car l'addition de pointeur tient compte du type, ici uint16_t prends deux octets

uint16_t* v2 = (uint16_t*) (&(Buffer[1][0]));
uint16_t* vmax2 = v2 + (Buff_char_size / 2); // on doit diviser par deux car l'addition de pointeur tient compte du type, ici uint16_t prends deux octets

uint16_t* v = v1    ;
uint16_t* vmax = vmax1 ;

uint16_t* v12[2] = {v1, v2};        // un tableau de pointeur pour swicher de buffer lorsque l'un est plein en attendant la transmission
uint16_t* vmax12[2] = {vmax1, vmax2}; // un tableau de pointeur pour swicher de buffer lorsque l'un est plein en attendant la transmission

int num_actuel = 0;
int num_autre = 1;
bool BUFFERS_FULL = false;
long int testmille = 0;
bool Bufferfull[2] = {false, false};            // are the buffers full ?
int byteswritten = 0;               // count the number of bytes written for display
int exclusions = 0;                 // Counts the number of unaccepted events
/***************************************************************************************************************************************/

long t0 = 0;
long t = 0;
byte NewLine[4] = {0x80, 0xA0, 0xC0, 0xE0};               // DDRAM address for each line of the display
char Tab[4][21];
int iScreen = 0;
int jScreen = 0;
NewhavenOLED oled(ROW_N, COLUMN_N, SDIN, SCLK, CS_SCREEN, RST_SCREEN);
/* USB */
int x;
int b;
int i;

/* PWM : find best prescaler */
static uint16_t FindPrescaler(uint32_t frequency, uint32_t mck)
{
  uint32_t prescaler[11] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
  uint8_t prescalerIndex = 0;
  uint32_t freq1000 = 0;
  /* Find prescalervalue */
  if ((frequency * 1000) <= 84000000)
    freq1000 = frequency * 1000;
  else
    freq1000 = 84000000;
  while (mck / prescaler[prescalerIndex] > (freq1000))
  {
    prescalerIndex++;
  }
  /* Return result */
  if ( prescalerIndex < 11 )
    return prescalerIndex;
  else
    return 10 ;
}


//======================================================================SETUP==============================================================
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); //D13
  pinMode(CS_SCREEN, OUTPUT);
  pinMode(RST_SCREEN, OUTPUT);
  pinMode(READ, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(CONVST, OUTPUT);
  pinMode(CS_1, OUTPUT);
  pinMode(CS_2, OUTPUT);
  pinMode(CS_3, OUTPUT);
  pinMode(BUSY_1, INPUT_PULLUP);
  pinMode(BUSY_2, INPUT_PULLUP);
  pinMode(BUSY_3, INPUT_PULLUP);
  pinMode(T_MCP, INPUT_PULLUP);
  pinMode(TEST_T_MCP, OUTPUT);
  pinMode(TEST_BUSY, OUTPUT);

  pinMode(Pin_Prob1, OUTPUT);
  pinMode(Pin_Prob2, OUTPUT);

  /*PWM, outputDisplay on pin 53 (2Hz to 42MHz), beware frequency steps increases with frequency*/
  pmc_enable_periph_clk (ID_PWM);  // turn on clocking to PWM unit
  PWMC_ConfigureChannel (PWM, 2, 0, 0, PWM_CMR_CPOL); // PWM channel 0, clock = MCLK = 84MHz
  PWMC_SetPeriod (PWM, 2, freqInPulse);  // period sur 16bits, il faut changer la fréquence d'horloge et le prescaler/divieur en fonction de la fréquence pwm voulue !!
  PWMC_SetDutyCycle (PWM, 2, freqInPulse / 2); // duty set to 50

  // Configure pin 53 (PB14) to be driven by peripheral B (PWM channel 2 H)
  // Configure pin 22 (PB26) as TCLK0 peripheral B (input for events to count)
  // //Configure pin DAC1 (PB16) as TCLK5 peripheral A
  //REG_PIOB_PDR = 0x04014000;  // disable PIO control (= enable peripherals control)
  //REG_PIOB_IDR = 0x04014000;   // disable PIO interrupts
  //REG_PIOB_ABSR = 0x04004000;  // Assigns to B peripheral
  REG_PIOB_PDR = 0x04004000;  // disable PIO control (= enable peripherals control)
  REG_PIOB_IDR = 0x04004000;   // disable PIO interrupts
  REG_PIOB_ABSR = 0x04004000;  // Assigns to B peripheral
  PWMC_EnableChannel (PWM, 2);   // enable

  /* TC0 Channel 0 for event counting on pin 22 (PB26) TCLK0*/
  pmc_enable_periph_clk (ID_TC0);  // turn on clocking to TC unit
  TC_Configure(TC0, 0, 5); // TC0 channel 0 mode XC0 (TCLK0), falling edge (13), rising edge (5) (external clock input on pin 22)
  TC_Start(TC0, 0);

  /* TC0 Channel 1 for time mesuring, */
  pmc_enable_periph_clk (ID_TC1);  // turn on clocking to TC unit : TC0 channel 1 = TC1 for ID number
  TC_Configure(TC0, 1, 0); // TC0 channel 1 mode 42MHz, rising edge,
  TC_Start(TC0, 1);

  digitalWrite (LED_BUILTIN, LOW);
  delay(2);
  digitalWrite (CS_1, HIGH);    //idle high, take low for transfers
  digitalWrite(READ, HIGH);
  digitalWrite(CONVST, HIGH); //idle high, pulse low and back to high to initiate conversion, should be high during the reset pulse
  digitalWrite(RESET, LOW);   //start low
  delay(1);
  digitalWrite(RESET, HIGH); //reset pulse
  delay(1);
  digitalWrite(RESET, LOW);
  digitalWrite(TEST_T_MCP, LOW);
  digitalWrite(TEST_BUSY, HIGH);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  oled.begin();                    // Setup control pins and initialize OLED controller

  digitalWrite(RST_SCREEN, HIGH);
  if (ROW_N == 2)                  // Update DDRAM address for 2-line mode
    NewLine[1] = 0xC0;            // 3- and 4-line use row addresses as defined above

  // Init display screen
  Output_Display();
  delay(3000);
  for (iScreen = 0; iScreen < 20; iScreen++)
    for (jScreen = 0; jScreen < 4; jScreen++)
      Tab[jScreen][iScreen] = TEXT2[jScreen][iScreen]; //32;
  iScreen = 0; jScreen = 0;


//  SerialUSB.begin(2000000);
//  delay(2000); // 10s ????
//
//  if (SerialUSB) // remplpacer par un while pour attendre la connexion
//    SerialUSB.println("USB native port connected");
//  else
//    SerialUSB.println("Unable to connect to USB native port after 10s");

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Power up");
#endif

#ifdef TRANSMISSION_ON
  Timer3.attachInterrupt(Send_Buffer).setFrequency(100).start();  //2000 Check to send the buffers if full at a given frequency
#endif

#ifdef SOFTWARE_ACQUISITION
  Timer4.attachInterrupt(Convert_ADC).setFrequency(100).start();
#endif

  attachInterrupt(digitalPinToInterrupt(T_MCP), MCP_EVENT_ISR, RISING); // go to MCP_Event_ISR() when rising edge on T_MCP
  attachInterrupt(digitalPinToInterrupt(BUSY_1), ADC_READOUT_ISR, FALLING); // go to ADC_READOUT_ISR() when falling edge on BUSY
  Timer8.attachInterrupt(CONVERT_ADC_ISR).setFrequency(250000); // when started call CONVERT ADC ISR after 4µs

  /* IRQ priority setting*/ //-------------- mettre à jour les prioritées !!!!!!!!!
  NVIC_SetPriority (TC3_IRQn, 3);        // Send_Buffer on timer 3 priority 3
}


//==============================================================================LOOP====================================================================================
void loop()
{
  /*Console read*/
  while (Serial.available() > 0) // Read serial port while there is something to read, for testing
  {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    if (inChar == '\n') {
      freq = inString.toDouble();
      inString = "";
      newCmd = true;
    }
  }

  /*Console commande order interpreter*/
  if (newCmd)
  {
    Serial.println("---------------------------------");
    Serial.println("new command");

    if (freq >= 2) // 2Hz
    {
      startAcq = true;
      Serial.println("start");
      Start_Acquisition();
    }
    else
    {
      startAcq = false;
      Serial.println("Stop");
      Stop_Acquisition();
      Serial.print("Number of cycle : ");
      Serial.println(cycleNumber);
      for (int ii = 0; ii < 2; ii++)
        for (int jj = 0; jj < 5; jj++)
          Serial.println(CycleAcquisitionData[ii][jj]);
    }
    Serial.println("---------------------------------");

    eventSoftCounter = 0;
    eventHardCounter = 0;
    eventHardCounter = REG_TC0_CV0;
    eventTimeValue = REG_TC0_CV1;
    Serial.println("Reset counters");
    /* Software counter*/
    Serial.print("Events software counter : ");
    Serial.println(eventSoftCounter);
    /* TC0 Channel 0 for event counting on pin 22*/
    Serial.print("Events hardware counter : ");
    Serial.println(eventHardCounter);
    Serial.print("Time counter value : ");
    Serial.println(eventTimeValue);


    /* PWM */
    PWMC_DisableChannel (PWM, 2);   // disable
    /* Frequency control for 50% duty cycle possibility*/
    if (freq > 42000000)
    {
      freq = 42000000;
    }
    if (freq < 2)
    {
      freq = 2;
    }
    pwmPrescalerIndex = FindPrescaler(freq, 84000000);
    PWMC_ConfigureChannel (PWM, 2, pwmPrescalerIndex, 0, PWM_CMR_CPOL) ;
    freqInPulseCmd = (84000000 / PrescalerList[pwmPrescalerIndex]) / freq;
    PWMC_SetPeriod (PWM, 2, freqInPulseCmd) ;  // une période égale deux coups d'horloge
    PWMC_SetDutyCycle (PWM, 2, freqInPulseCmd / 2) ; // duty set to 50% (1 coup d'horloge)
    //Serial.print("PWM Frequency on pin 53: ");
    //Serial.print(freq);
    //Serial.println(" Hz");
    PWMC_EnableChannel (PWM, 2) ;   // enable
    sendConsign = false;
    newCmd = false;
  }


  // Déclenche une simulation d'ISR sur T_MCP toutes les 10ms
  if ((unsigned long)(millis() - softISRChangeTime) > 10)
  {
    /* ISR software start for testing*/
    if (startAcq == 1)
    {Pin_Prob1_high;
    Pin_Prob1_low;
    Pin_Prob2_high;
    Pin_Prob2_low; 
      digitalWrite(TEST_T_MCP, HIGH); // event incoming simulation
      digitalWrite(TEST_T_MCP, LOW);
    }
    softISRChangeTime = millis();
  }

  /* Console display data for testing*/
  if (newDataISR)
  {
    Serial.println("---------------------------------");
    /* Software counter*/
    Serial.print("Events software counter : ");
    Serial.println(eventSoftCounter);
    /* TC0 Channel 0 for event counting on pin 22*/
    Serial.print("Events hardware counter : ");
    Serial.println(eventHardCounter);

    /* Overflow wathdog*/
    tc0Channel1Status = TC_GetStatus(TC0, 1);
    if ((tc0Channel1Status & 1) == 1)//overflow detected
    {
      Serial.println(tc0Channel1Status, BIN);
      Serial.println("Overflow detected");
      Serial.println("Reset counters");

      TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG; //Software trigger on TC0 channel 0 : reset the counter if counter is started only
      TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; //Software trigger on TC0 channel 1 : reset the counter if counter is started only
      eventSoftCounter = 0;
      eventHardCounter = 0;
      Stop_Acquisition();
    }
    else // no overflow
    {
      Serial.print("Time counter value : ");
      Serial.println(eventTimeValue);
    }
    newDataISR = false;
  }

  /* Display on oled screen */
  currentMillis = millis();
  if (currentMillis != previousMillis) // affichage d'un charactère toutes les millisecondes (pas terrible)
  {
    previousMillis = currentMillis;
    // oled.data(Tab[jScreen][iScreen]);
    oled.write(iScreen, jScreen, Tab[jScreen][iScreen]);
    iScreen++;
    if (iScreen >= 20) {
      iScreen = 0;
      jScreen++;
    }
    if (jScreen >= 4) {
      jScreen = 0;  //oled.setCursor(0,0);
      New_Fill_Display();
    }
  }
}


//======================================================================== ISR definitions============================================================
void TEST_ISR()  // pas utilisé
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
//=======================================================================
void MCP_EVENT_ISR()  // appelé par interruption sur pin 12 : T_MCP
{
  Pin_Prob1_high;
  // Read time counter
  eventTimeValue = REG_TC0_CV1;  // read the time counter value of TC0 Channel 1
  // Read events hard counter
  eventHardCounter = REG_TC0_CV0 + 1; // TCLK0,  +1 because after a reset le counter pass to 0 on the first valid edge
  // Increase events sofware counter
  eventSoftCounter ++;
  // Start 4µs counter for adc conversion
 // Timer8.start();

  // for test
  newDataISR = true;
  Pin_Prob1_low;
}
//=======================================================================
void CONVERT_ADC_ISR()     // start Software triggered convertion for testing interrupt on a DC signal. The normal way is via BUSY interrupt
{
  // Stop the 4µs counter
  Timer8.stop();
  // Start ADC conversion
  digitalWrite(CONVST, LOW);  //pulse CONVST low then high again, this triggers the call to fastread(); just below
  digitalWrite(CONVST, HIGH);
}
//================================================
void ADC_READOUT_ISR()     // normalement si on arrive ici c'est que l'on est pas encore en BUFFERS_FULL, le premier buffer à envoyer est num_autre
{
  callNumber++; // compteur global du nombre d'appels
  // v=v1;  // philippe aout 2019 pour test uniquement, toutes les données sont réécrites au début

  if (v + 6 >= vmax)
  { // Attention à vérifier les bornes et égalités
    Bufferfull[num_actuel] = true;
    if (!Bufferfull[num_autre])
    {
      v = v12[num_autre];
      vmax = vmax12[num_autre];
      int a = num_actuel;
      num_actuel = num_autre;
      num_autre = a;
    }
    else
    { // plus de place, il faut attendre
      exclusions++;               // BUFFERS_FULL=true;
      BUFFERS_FULL = true;
    }
  }

  // Arrivé ici on est sur d'avoir de la place, reste à lire les data
  // Reading Procedure, une boucle de 6 lectures du port C de 32 bit tronquée aux 16 bit significatifs de l'ADC
  if (!BUFFERS_FULL)
  { // Si il y a de la place  dans le buffer
    unsigned long brutC = REG_PIOC_PDSR;      //
    unsigned long brutB = REG_PIOB_PDSR;      //
    REG_PIOB_ODSR = brutB SettoLow_(25);      //niveau bas sur Chip select (Carte A)
    REG_PIOC_ODSR = brutC & maskLow28;        //Niveau bas sur Read (Carte A)

    for (int i = 0; i < 6; i++)
    {
      brutC = REG_PIOC_PDSR >> 1;                                 // On lit les 32 bits du port_C dont 16 contiennent les datas décalé une fois a droite le bit 1 devient 0
      *(v++) = (uint16_t) ((brutC  & mask1) >> 2) | (brutC & mask2);        // On met en forme en isolant 7 bits haut et supprimant 9 et 10 et recollant avec les 9 bits du bas
      REG_PIOC_ODSR = brutC | maskHigh28;                        // Niveau haut sur Read (Carte A)
      REG_PIOC_ODSR = brutC & maskLow28;                         // Niveau bas sur Read (Carte A)
    }
    REG_PIOB_ODSR = brutB | maskHighCS;                          // niveau haut sur Chip select (Carte A)
    REG_PIOC_ODSR = brutC | maskHigh28;                          // Niveau haut sur Read (Carte A) (pour remettre au niveau passif)
  }
}
//===========================function definitions========================
void Start_Acquisition()     // start Software triggered convertion for testing interrupt on a DC signal. The normal way is via BUSY interrupt
{
  // Start PWM
  PWMC_EnableChannel (PWM, 2) ;   // enable
  // RAZ software events counter
  eventSoftCounter = 0;
  //  Start and reset hardware events counter
  TC_Start(TC0, 0);
  // Start and reset time measuring counter
  TC_Start(TC0, 1);
  // Enable ISR
  attachInterrupt(digitalPinToInterrupt(T_MCP), MCP_EVENT_ISR, RISING); // go to MCP_Event_ISR() when rising edge on T_MCP
  attachInterrupt(digitalPinToInterrupt(BUSY_1), ADC_READOUT_ISR, FALLING); // go to ADC_READOUT_ISR() when falling edge on BUSY
}
//=======================================================================
void Stop_Acquisition()     // start Software triggered convertion for testing interrupt on a DC signal. The normal way is via BUSY interrupt
{
  // Stop PWM
  PWMC_DisableChannel (PWM, 2) ;   // enable
  // Read and stop software events counter
  CycleAcquisitionData[0][cycleNumber] = eventSoftCounter;
  TC_Stop(TC0, 0);
  // Read hardware events counter
  CycleAcquisitionData[1][cycleNumber] = eventHardCounter;
  // Stop time measuring counter
  TC_Stop(TC0, 1);
  // Disable ISR
  detachInterrupt(digitalPinToInterrupt(T_MCP)); // go to MCP_Event_ISR() when rising edge on T_MCP
  detachInterrupt(digitalPinToInterrupt(BUSY_1)); // go to ADC_READOUT_ISR() when falling edge on BUSY
  // Increase cycle number for next cyccle
  cycleNumber ++;
}
//=======================================================================
void Convert_ADC()     // start Software triggered convertion for testing interrupt on a DC signal. The normal way is via BUSY interrupt
{
  // Start ADC conversion on command
  digitalWrite(CONVST, LOW);  //pulse CONVST low then high again, this triggers the call to fastread(); just below
  digitalWrite(CONVST, HIGH);
}
//=======================================================================
void Send_Buffer()     // envoie des buffers sur la liaison USB native
{
  if (Bufferfull[num_autre])
  {
    //digitalWrite(TEST_PIN, LOW); // pour l'oscillo
    byteswritten += SerialUSB.write(Buffer[num_autre], Buff_char_size); // la procédure fonctionne par octet
    //digitalWrite(TEST_PIN, HIGH);
    Bufferfull[num_autre] = false;
    BUFFERS_FULL = false;

    memset(Buffer[num_autre], 0, Buff_char_size);                     // remise à zéro rapide (cf Web) pour éviter les erreurs lors de transmission de buffers incomplets
  }
  // il faudrait vérifier Serial.availableForWrite() avant d'envoyer, sinon ce sera plus tard. on évite ainsi de bloquer
  if (Bufferfull[num_actuel])
  {
    byteswritten += SerialUSB.write(Buffer[num_actuel], Buff_char_size); // la procédure fonction par octet

    Bufferfull[num_actuel] = false;
    BUFFERS_FULL = false;

    memset(Buffer[num_actuel], 0, Buff_char_size);                      // remise à zéro rapide (cf Web). Discutable mais, à ce stade cela évite de lire deux fois la même info et permet la transmission de buffers incomplets
  }
}
//=======================================================================
// Oled screen init screen
void Output_Display(void)                    // SUBROUTINE: DISPLAYS THE FOUR STRINGS, THEN THE SAME IN REVERSE ORDER
{
  byte r = 0;                        // Row index
  byte c = 0;                        // Column index
  oled.command(0x01);                // Clears display (and cursor home)
  currentMillis = millis();
  previousMillis = currentMillis + 3;
  while (millis() < nextMillis); // We wait to be sure that the boot of the display chip is finished ?
  for (r = 0; r < ROW_N; r++)        // One row at a time,
  {
    oled.command(NewLine[r]);       //  moves the cursor to the first column of that line
    for (c = 0; c < COLUMN_N; c++)   // One character at a time,
    {
      oled.data(TEXT1[r][c]);         //  displays the correspondig string
    }
  }
}
//=======================================================================
void New_Fill_Display(void)
{
  int i, j, taille;
  char DataChar[20];
  char white = 32;

  String dataString = String(callNumber);  // display p the number of BufferWriting (successful reads of batches of 6 voltages)
  taille = dataString.length() + 1;
  dataString.toCharArray(DataChar, taille);
  for (int i = 0; i < taille - 1 ; i++) Tab[0][21 - taille + i] = DataChar[i]; //oled.write(21-taille+i, 0, DataChar[i]);

  dataString = String(byteswritten);
  taille = dataString.length() + 1;
  dataString.toCharArray(DataChar, taille);
  for (int i = 0; i < taille - 1 ; i++) Tab[1][21 - taille + i] = DataChar[i]; // nombre d'octets transférés

  uint16_t dataInt;
  char OneNumber[7];

  for (j = 0; j < 6; j++) // les six valeurs
  {
    dataInt = Buffer[0][2 * j] << 8 + Buffer[0][2 * j + 1];
    dataString = String(dataInt, HEX); //String(dataInt, HEX);
    taille = dataString.length() + 1;
    dataString.toCharArray(DataChar, taille);
    int lineNumber = 2 + j / 3;
    int offset = (1 + j % 3) * 6;
    for (i = 0; i < taille - 1 ; i++) Tab[lineNumber][offset - taille + i] = DataChar[i];
  }
}
//=======================================================================
// write "Counting rate :" on screen
void Counting_Rate()
{
  byte r = 0;                        // Row index
  byte c = 0;                        // Column index
  oled.command(0x01);                // Clears display (and cursor home)
  delay(2);                          // After a clear display, a minimum pause of 1-2 ms is required
  for (r = 0; r < ROW_N; r++)        // One row at a time,
  {
    oled.command(NewLine[r]);       //  moves the cursor to the first column of that line
    for (c = 0; c < COLUMN_N; c++)   // One character at a time,
    {
      oled.data(TEXT2[r][c]);         //  displays the correspondig string
    }
  }
  //delay(2000);                       // Waits, only for visual effect purpose
}
//=======================================================================
void Value_Display()
{
  int i;
  int j;
  uint16_t dataInt;
  char OneNumber[7];
  char DataChar[7];
  char white = 32;
  String dataString;
  int taille;
  int lineNumber;
  int offset;

  for (j = 0; j < 2; j++) // les six valeurs
  {
    dataInt = Buffer[0][2 * j]; //<<8 + Buffer[0][2*j+1];
    //dataInt = j;
    //dataInt = dataInt*20/65536;
    dataString = String(dataInt, HEX); //String(dataInt, HEX);
    taille = dataString.length() + 1;
    dataString.toCharArray(DataChar, taille);
    for (i = 0; i < 6 ; i++) OneNumber[i] = white; //on rempli de whites
    for (i = 0; i < taille - 1 ; i++) OneNumber[7 - taille + i] = DataChar[i];
    lineNumber = 2 + j / 3;
    offset = (j % 3) * 6;
    for (i = 0; i < 6 ; i++) oled.write(offset + i, lineNumber, OneNumber[i]);
    oled.write(offset + 7, lineNumber, white);
    //  for ( i = 0; i < taille - 1 ; i++) oled.write(offset-taille + i, lineNumber,DataChar[i]);
  }
}
//=======================================================================
// display writing rate value
void Writing_Rate_Display()
{
  char DataChar[10];
  unsigned long dataInt;
  String dataString;
  int taille;

  dataInt = callNumber;           // display p the number of BufferWriting (successful reads of batches of 6 voltages)
  dataString = String(dataInt);
  taille = dataString.length() + 1;
  dataString.toCharArray(DataChar, taille);
  for (int i = 0; i < taille - 1 ; i++) oled.write(21 - taille + i, 0, DataChar[i]);

  /*  char DataChar2[10];
    int dataInt2 = byteswritten;                       // display the number of bytes written from the buffer to the USBPort
    String dataString2 = String(dataInt2);
    int taille2 = dataString2.length() + 1;
    dataString2.toCharArray(DataChar2, taille2);
    for (int i = 0; i < taille2 - 1 ; i++)
    {
      oled.write(4 + i, 3, DataChar2[i]);
    }*/

  /*  char DataChar3[10];
    float dataInt3 = exclusions/callNumber;         // display the acceptance rate
    String dataString3 = String(dataInt3);
    int taille3 = dataString3.length() + 1;
    dataString3.toCharArray(DataChar3, taille3);
    for (int i = 0; i < taille3 - 1 ; i++)
    {
      oled.write(14 + i, 2, DataChar3[i]);
    }*/
}
