#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <DHT.h>


// definindo o buzzer
#define DHT_PIN 7
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);


// definindo o lcd
LiquidCrystal_I2C lcd(0x27, 16, 2);

// definindo outputs
#define GREEN_LED 3
#define YELLOW_LED 4
#define RED_LED 5
#define BUZZER A3

// define inputs
#define LIGHT_SENSOR A2

// define values
#define LDR_YELLOW_LED 500

// active buzzer
bool activeBuzzer = false;

// temperature
// converte a tensão lida em Graus Celsius
float temperature = 0;

// Faz a leitura da tensao no Sensor de Umidade
float humidity = 0;  // Lê a umidade relativa do ar em percentual


// definindo icones
byte temp[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b01110
};

byte water[8] = {
  0b00100,
  0b00100,
  0b01010,
  0b01010,
  0b10001,
  0b10001,
  0b10001,
  0b01110
};

byte light[8] = {
  0b00000,
  0b00000,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00000
};

void setup() {
  Serial.begin(9600);
  // definindo os pinos dos leds e da buzina como saídas
  setupOutputs();
  setupDisplayI2C();
  dht.begin();
}

void loop() {
  int ldr = analogRead(LIGHT_SENSOR);  // lendo o valor do LDR

  // converte a tensão lida em Graus Celsius
  float temperature = 10.0; //dht.readTemperature()

  // Faz a leitura da tensao no Sensor de Umidade
  float humidity = dht.readHumidity();  // Lê a umidade relativa do ar em percentual

  //Calcula media de 5 leituras da temperatura
  int tempRead = 0;
  int mediaTemp = 0;
  for (int i = 0; i < 5; i++) {
    tempRead += temperature;
    delay(200);  //delay a cada leitura para maior precisão
  }
  mediaTemp = tempRead / 5;  //resultado média de 5 leituras da temperatura

  //calcula média de 5 leituras da umidade
  int humidityRead = 0;
  int mediaHumidity = 0;
  for (int x = 0; x < 5; x++) {
    humidityRead += humidity;
    delay(200);  //delay a cada leitura para maior precisão
  }
  mediaHumidity = humidityRead / 5;  //resultado média de 5 leituras da umidade

  // Estado da temperatura em graus celcius
  //temperatura BOA
  if (mediaTemp >= 10 and mediaTemp <= 15) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)0);
    lcd.print(" Temperatura OK");
    lcd.setCursor(0, 1);
    lcd.print("Temp. = ");
    lcd.setCursor(8, 1);
    lcd.print(mediaTemp);
    lcd.setCursor(10, 1);
    lcd.print("C");
    delay(3000);
  }
  //Temperatura ALTA acima de 15 graus
  else if (mediaTemp > 15) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(RED_LED, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)0);
    lcd.print(" Temp. ALTA");
    lcd.setCursor(0, 1);
    lcd.print("Temp. = ");
    lcd.setCursor(8, 1);
    lcd.print(mediaTemp);
    lcd.setCursor(10, 1);
    lcd.print("C");

    // faz com que a buzina toque sem parar enquanto estiver acima de 15 graus
    ringBuzzer(15.0, static_cast<float>(temperature), 2);
    temperature = 10.0; //dht.readTemperature()

  }
  //temperatura baixa menos que 10 graus
  else {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)0);
    lcd.print(" Temp. BAIXA");
    lcd.setCursor(0, 1);
    lcd.print("Temp. = ");
    lcd.setCursor(8, 1);
    lcd.print(mediaTemp);
    lcd.setCursor(10, 1);
    lcd.print("C");
    // faz com que a buzina toque sem parar enquanto estiver abaixo de 10 graus
    ringBuzzer(static_cast<float>(temperature), 10.0, 2);
    float temperature = 10.0; //dht.readTemperature()
  }


  //Atualiza estado da umidade
  if (humidity > 50 and humidity < 70) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)1);
    lcd.print(" Umidade OK");
    lcd.setCursor(0, 1);
    lcd.print("Umidade =");
    lcd.setCursor(10, 1);
    lcd.print(mediaHumidity);
    lcd.setCursor(12, 1);
    lcd.print("%");

    delay(3000);

  } else if (humidity >= 70) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)1);
    lcd.print(" Umidade ALTA");
    lcd.setCursor(0, 1);
    lcd.print("Umidade =");
    lcd.setCursor(10, 1);
    lcd.print(mediaHumidity);
    lcd.setCursor(12, 1);
    lcd.print("%");

    delay(3000);
  }

  else {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)1);
    lcd.print(" Umidade BAIXA");
    lcd.setCursor(0, 1);
    lcd.print("Umidade =");
    lcd.setCursor(10, 1);
    lcd.print(mediaHumidity);
    lcd.setCursor(12, 1);
    lcd.print("%");

    ringBuzzer(static_cast<float>(humidity), 50.0, 1);
    float humidity = dht.readHumidity();
  }


  // verificando a luminosidade e acendendo o LED correspondente
  if (ldr > LDR_YELLOW_LED) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)2);
    lcd.print(" Ambiente luz");
    lcd.setCursor(0, 1);
    lcd.print("adequada");
    delay(3000);
  } else if (ldr > 260) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(RED_LED, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)2);
    lcd.print(" Ambiente a");
    lcd.setCursor(0, 1);
    lcd.print("meia luz");

    // ativando a buzina por 3 segundos
    ringBuzzer(260.0, static_cast<float>(ldr), 0);
    int ldr = analogRead(LIGHT_SENSOR);
  } else {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)2);
    lcd.print(" Ambiente muito");
    lcd.setCursor(0, 1);
    lcd.print("CLARO");

    ringBuzzer(static_cast<float>(ldr), 260.0, 0);
    int ldr = analogRead(LIGHT_SENSOR);
  }
}

void ringBuzzer(float value, float limit, int type) {
  if (!activeBuzzer) {
    activeBuzzer = true;
    while (value < limit) {
      digitalWrite(BUZZER, HIGH);
      delay(200);
      digitalWrite(BUZZER, LOW);
      delay(200);
      digitalWrite(BUZZER, HIGH);
      delay(200);
      digitalWrite(BUZZER, LOW);
      delay(200);
      digitalWrite(BUZZER, HIGH);
      delay(200);
      digitalWrite(BUZZER, LOW);
      delay(1000);
      if (type == 0) {
        value = analogRead(LIGHT_SENSOR);
      }
      if (type == 1) {
        value = dht.readHumidity();
      }
      if (type == 2) {
        value = 10.0; //dht.readTemperature()
      }
    }
    digitalWrite(BUZZER, LOW);
    activeBuzzer = false;
  } else {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER, LOW);
    activeBuzzer = false;
  }
}

void setupOutputs() {
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LIGHT_SENSOR, INPUT);
}

void setupDisplayI2C() {
  Wire.begin();     // Inicializa a comunicação I2C
  lcd.init();       // Inicialização do display
  lcd.backlight();  // Acende o backlight
  lcd.print("Agnelo Vinhos");
  lcd.setCursor(0, 1);
  lcd.print("Monitoramento");
  delay(2000);  // aguardando 2 segundos antes de iniciar a leitura dos sensores
  lcd.createChar(0, temp);
  lcd.createChar(1, water);
  lcd.createChar(2, light);
  lcd.home();
}
