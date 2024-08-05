#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>

// Definições do termistor e do divisor de tensão
#define THERMISTORPIN 34 // Pino onde o termistor está conectado (GPIO34)

// Definições da tela OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Definições das portas I2C
#define I2C_SDA 21
#define I2C_SCL 22

// Definições das tarefas
#define TASK_STACK_SIZE 2048
#define TASK_PRIORITY 1

// Definições do cooler
#define COOLER_PIN 25 // Pino PWM para o cooler

# define BIAS 54

// Variáveis para armazenamento das temperaturas
float lastTemperature = 0.0;
float currentTemperature = 0.0;
float temperatureGradient = 0.0;

// Variáveis do PID
double Setpoint, Input, Output;
double Kp = 100.0, Ki = 10.0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE); // Definido para 'DIRECT' permanentemente

// Mutex para proteção de recursos compartilhados
SemaphoreHandle_t xMutex;

// Função para obter a temperatura do termistor
double getTemperature() {
  int RawADC = analogRead(THERMISTORPIN);

  // Verificação da leitura analógica
  if (RawADC == 0) {
    Serial.println("Error: RawADC is 0, cannot calculate resistance.");
    return NAN;
  }

  long Resistance;
  double Temp;

  // Calculando a resistência do termistor
  Resistance = ((40960000 / RawADC) - 10000);

  // Verificação da resistência calculada
  if (Resistance <= 0) {
    Serial.println("Error: Calculated resistance is non-positive.");
    return NAN;
  }

  // Utilizando a Equação de Steinhart-Hart para calcular a temperatura
  Temp = log(Resistance);
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convertendo Kelvin para Celsius

  return (Temp + BIAS);  // Retornando a temperatura em Celsius
}

// Task para ler e calcular o gradiente de temperatura

void readAndCalculateTask(void *pvParameters) {
  for (;;) {
    // Lê a temperatura do termistor
    float newTemperature = getTemperature();

    // Checa se a leitura é válida
    if (isnan(newTemperature)) {
      Serial.println("Falha ao ler do termistor!");
    } else {
      // Protege a variável compartilhada com o mutex
      xSemaphoreTake(xMutex, portMAX_DELAY);
      lastTemperature = currentTemperature;
      currentTemperature = newTemperature;
      temperatureGradient = currentTemperature - lastTemperature;
      xSemaphoreGive(xMutex);

      Serial.print("Temperatura atual: ");
      Serial.println(currentTemperature);
      Serial.print("Gradiente de temperatura: ");
      Serial.println(temperatureGradient);
    }

    // Espera 1 segundo antes de ler novamente
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task para mostrar o gráfico de temperatura e o gradiente na tela OLED de 0.9"

void displayTask(void *pvParameters) {
  // Inicializa o display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Endereço I2C 0x3C
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.display();

  const int graphWidth = SCREEN_WIDTH - 10;
  const int graphHeight = 20;
  const int graphYPos = SCREEN_HEIGHT - graphHeight - 10;

  float lastTemperatures[graphWidth];
  memset(lastTemperatures, 0, sizeof(lastTemperatures)); // Inicializa o array com zeros
  int index = 0;

  const float Y_MAX = 40.0; // Ajuste os limites conforme necessário
  const float Y_MIN = 0.0;
  const float Y_RANGE = Y_MAX - Y_MIN;

  for (;;) {
    // Limpa o display
    display.clearDisplay();

    // Protege a variável compartilhada com o mutex
    xSemaphoreTake(xMutex, portMAX_DELAY);
    float temperature = currentTemperature;
    float gradient = temperatureGradient;
    xSemaphoreGive(xMutex);

    // Exibe o gradiente de temperatura
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Gradiente Temp: ");
    display.println(gradient);

    // Adiciona a temperatura atual ao array de temperaturas
    lastTemperatures[index] = temperature;
    index = (index + 1) % graphWidth;

    // Desenha o gráfico de tendência da temperatura
    int lastY = graphYPos + graphHeight;
    for (int i = 0; i < graphWidth; i++) {
      int x = (index + i) % graphWidth;
      int y = graphYPos + graphHeight - (int)((lastTemperatures[x] - Y_MIN) / Y_RANGE * graphHeight);
      display.drawPixel(i + 5, y, SSD1306_WHITE);
      if (i > 0) {
        display.drawLine(i + 4, lastY, i + 5, y, SSD1306_WHITE);
      }
      lastY = y;
    }

    // Atualiza o display
    display.display();

    // Espera 1 segundo antes de atualizar novamente
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task para controlar o Coller por meio de um PID

void coolerControlTask(void *pvParameters) {
  // Configura o pino do cooler
  pinMode(COOLER_PIN, OUTPUT);

  // Define o setpoint do gradiente de temperatura desejado
  Setpoint = 0.2; // Por exemplo, queremos que o gradiente seja zero (estabilidade térmica)

  // Inicializa o PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(100, 255); // Limita a saída do PID ao range PWM (0-255)

  for (;;) {
    // Protege a variável compartilhada com o mutex
    xSemaphoreTake(xMutex, portMAX_DELAY);
    Input = temperatureGradient; // O PID agora controla o gradiente
    xSemaphoreGive(xMutex);

    // Executa o cálculo PID
    myPID.Compute();

    // Ajusta a velocidade do cooler com base na saída do PID
    analogWrite(COOLER_PIN, Output);

    Serial.print("PID Output: ");
    Serial.println(Output);

    // Espera 1 segundo antes de atualizar novamente
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);

  // Inicializa as portas I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Cria o mutex
  xMutex = xSemaphoreCreateMutex();

  // Cria as tarefas do FreeRTOS, especificando o núcleo
  xTaskCreatePinnedToCore(readAndCalculateTask, "ReadAndCalculateTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL, 0); // Núcleo 0
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL, 1); // Núcleo 1
  xTaskCreatePinnedToCore(coolerControlTask, "CoolerControlTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL, 1); // Núcleo 1
}

void loop() {
  // pass
}
