// Bibliotecas inclusas 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Definições do Display
#define endereco  0x27
#define coluna    16
#define linha     2

LiquidCrystal_I2C lcd(endereco, coluna, linha);

// Pinagem dos 3 sensores
const int pinTCRT1 = 6;
const int pinTCRT2 = 5;
const int pinTCRT3 = 3;

// Pinagem de comunicação com módulo i2c do display
const int sclDP = A4;
const int sdaDP = A5;

// Pinagem dos 3 LEDs
const int pinLED1 = A0;
const int pinLED2 = A1;
const int pinLED3 = A2;

// Pinagem do botão tátil
const int botao = 2;

// Declaração de variáveis 
const float spc = 0.485; // Variação de espaço (DELTA S) entre os sensores 1 e 2 = 2 e 3

float t1 = 0; // Declaração da variável tempo (momento sensor 1)
float t2 = 0; // Declaração da variável tempo (momento sensor 2)
float t3 = 0; // Declaração da variável tempo (momento sensor 3)
float deltat1 = 0; // Declaração da variável tempo entre os sensores 1 e 2
float deltat2 = 0; // Declaração da variável tempo entre os sensores 1 e 3
float v1 = 0; // Declaração da varável volocidade sensor 1
float v2 = 0; // Declaração da varável volocidade sensor 2
float v3 = 0; // Declaração da varável volocidade sensor 3
float grav = 0; // Declaração da variável gravidade
float erro = 0; // Declaração da variável margem de erro

void setup() {
  Serial.begin(9600);
  lcd.init();
  Serial.println("iniciando...");
  lcd.setBacklight(HIGH);
  lcd.print("INICIANDO...");
  delay(3000);
  lcd.setBacklight(LOW);

  // Declaração de modo dos pinos
  pinMode(pinTCRT1, INPUT);
  pinMode(pinTCRT2, INPUT);
  pinMode(pinTCRT3, INPUT);
  pinMode(pinLED1, OUTPUT);
  pinMode(pinLED2, OUTPUT);
  pinMode(pinLED3, OUTPUT);
}

void loop() {

  if(digitalRead(pinTCRT1) == 0){ // Acender LED1 caso sensor1 detecte objeto
    t1 = millis(); // marcar em miliseg o ultimo momento de detecção no sensor 1
    digitalWrite(pinLED1, HIGH);
    delay(10);
    digitalWrite(pinLED1, LOW);
  }

  else if((digitalRead(pinTCRT2) == 0) && (digitalRead(pinTCRT1) == 1)){ // Acender LED2 caso sensor2 detecte objeto
    t2 = millis(); // marcar em miliseg o ultimo momento de detecção no sensor 2
    digitalWrite(pinLED2, HIGH);
    delay(10);
    digitalWrite(pinLED2, LOW);
  }

  else if((digitalRead(pinTCRT3) == 0) && (digitalRead(pinTCRT2) == 1)){ // Acender LED3 caso sensor3 detecte objeto
    t3 = millis(); // marcar em miliseg o ultimo momento de detecção no sensor 3
    digitalWrite(pinLED3, HIGH);
    delay(10);
    digitalWrite(pinLED3, LOW);
  }

  // Cáculos finais
  deltat1 = (t2 - t1)/1000.00; // Variação de tempo entre os sensores 1 e 2
  deltat2 = (t3 - t1)/1000.00; // Variação de tempo entre os sensores 1 e 2
  grav = 2 * spc / pow((deltat1 + 0.01), 2); // Foi somado ao intervalo de tempo, o delay do LED
  v1 = 0; // Velocidade inicia é 0, o objeto precisa ser solto em repouso
  v2 = grav * (deltat1 + 0.01); // Valor da velocidade no sensor 2
  v3 = grav * (deltat2 + 0.02); // Valor da velocidade no sensor 3
  erro = ((grav - 9.8) / 9.8) * 100; // Margem de erro em relação a gravidade conhecida (9,8 m/s2)

  if(digitalRead(botao) == 1){ // Imprimir dados no display e no monitor serial quando apertar o botão
    // Impressão serial de caráter experimental 1
    Serial.print("Tempo sensor 1: ");
    Serial.println(deltat1 + 0.01);
    Serial.print("Tempo sensor 2: ");
    Serial.println(deltat2 + 0.02);
    Serial.println();

    // Impressão serial de caráter experimental 2
    Serial.print("DeltaS * 2 = ");
    Serial.println(spc * 2);
    Serial.print("DELTA T^2 sensor 1: ");
    Serial.println(pow((deltat1 + 0.01), 2));
    Serial.print("DELTA T^2 sensor 2: ");
    Serial.println(pow((deltat2 + 0.02), 2));
    Serial.println("=========================================");
    Serial.println();

    // Impressão final no LCD
    lcd.clear();
    lcd.setBacklight(HIGH);
    lcd.print("Gravidade");
    lcd.setCursor(0, 1);
    lcd.print(grav);
    lcd.print(" m/s^2");
    delay(3000);
    lcd.clear();
    lcd.print("Vinst (sensor 1)");
    lcd.setCursor(0, 1);
    lcd.print(v1);
    lcd.print(" m/s");
    delay(3000);
    lcd.clear();
    lcd.print("Vinst (sensor 2)");
    lcd.setCursor(0, 1);
    lcd.print(v2);
    lcd.print(" m/s");
    delay(3000);
    lcd.clear();
    lcd.print("Vinst (sensor 3)");
    lcd.setCursor(0, 1);
    lcd.print(v3);
    lcd.print(" m/s");
    delay(3000);
    lcd.clear();
    lcd.print("Margem de erro:");
    lcd.setCursor(0, 1);
    lcd.print(erro);
    lcd.print(" %");
    delay(3000);
    lcd.clear();
  }

  else{ // Apagar display quando os dados forem apresentados
    lcd.setBacklight(LOW); 
  }
}
