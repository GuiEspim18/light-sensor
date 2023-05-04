#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);


#define DHTPIN 7     // Pino de dados do sensor DHT11
#define DHTTYPE DHT11   // Tipo de sensor DHT
DHT dht(DHTPIN, DHTTYPE);

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

void setup() {
  setupDisplay();
  setupDht11();
}


void loop() {
  dht11();
}


void setupDisplay() {
  Wire.begin(); // Inicializa a comunicação I2C
  lcd.init(); // Inicialização do display
  lcd.backlight(); // Acende o backlight
  lcd.createChar(0, temp);
  lcd.createChar(1, water);
  lcd.home();
}

void setupDht11() {
  dht.begin();
}

void dht11() {
  delayMicroseconds(20);
  float temp = dht.readTemperature(); // Lê a temperatura em graus Celsius
  float umid = dht.readHumidity(); // Lê a umidade relativa do ar em percentual

  lcd.setCursor(0, 0);
  lcd.write((uint8_t)0);
  lcd.print(" Temp: ");
  lcd.print(temp);
  lcd.printstr("C ");
  lcd.setCursor(0, 1);
  lcd.write((uint8_t)1);
  lcd.print(" Um: ");
  lcd.print(umid);
  lcd.printstr("% ");
}