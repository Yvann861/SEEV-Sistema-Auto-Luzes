/*
Nome ALUNO A- Tiago Moital
Nome ALUNO B- Yvann Pato
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
EAU- Licenciatura em Engenharia Automóvel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos

TP1: Pretende-se  neste  trabalho  prático  a  implementação  de um  algoritmo de um sistema automático de luzes de um automóvel
     juntamnete com um sensor de leitura de Temperatura/Humidade utilizando um sistema operativo de tempo real FreeRTOS.

LINK: (link do código do github gist).
Github:
Youtube: https://youtube.com/shorts/rlv6s_uRjoI
*/

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <DHT.h>

// Pinos do Display TFT
#define TFT_CS   5
#define TFT_DC   25
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18
#define TFT_RST -1 // ligar ao 3V3

// Pinos Botões e Led´s
const uint8_t interruptPin = 33;  // Botão(SW1)
const uint8_t ledAnalogPin = 2;   // LED Minimos (Brancos)
const uint8_t ledDigitalPin = 4;  // LED Maximos (Amarelos)
const uint8_t buttonSW2 = 32;     //Botão SW2
const int pinLDR = 34;            // Sensor LDR

//configurações PWM
#define LED_ANALOG_PWM_CHANNEL 0  // Canal PWM para LED analógico
#define LED_DIGITAL_PWM_CHANNEL 1 // Canal PWM para LED digital
#define LED_PWM_FREQ 5000         // Frequência PWM
#define LED_PWM_RESOLUTION 8      // Resolução do PWM (8 bits)

//Configurações Sensor DHT11
#define DHTPIN 26                 //Pino
#define DHTTYPE DHT11             //Tipo de sensor
DHT dht(DHTPIN, DHTTYPE);

// Constante Luminosidade
#define LUMINOSITY_THRESHOLD 2000 // Limite de luminosidade para o LED analógico

// Protótipos das funções
static void vHandlerTask(void *pvParameters);
static void vTaskHandleSW2(void *pvParameters);
static void vTaskModo(void *pvParameters);
void vTaskLDR(void *pvParameters);
void vTaskSensorTSL(void *pvParameters);
void vTaskDHT(void *pvParameters);
void vTask_TFT(void *pvParameters);
static void IRAM_ATTR vButtonInterruptHandler(void);
static void IRAM_ATTR vButtonInterruptHandler2(void);

// Semáforos e Filas(queues)
SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t xBinarySemaphoreSW2 = xSemaphoreCreateBinary();
SemaphoreHandle_t xMutexDisplay;         // MUTEX para proteger o acesso ao display
SemaphoreHandle_t xMutexQueueState;      // MUTEX para proteger a fila de estados
QueueHandle_t xQueueModo, xQueuePWM, xQueueDisplayMode, xQueueDisplayState, xQueueDisplayData, xQueueSensorState;

// Variáveis globais
bool autoModeActive = false;       // Indica se o modo AUTO está ativo
bool isNight = false;              // Indica se o LDR detectou noite


void setup() {
    // Inicialização
    Serial.begin(115200);

    // Configuração dos pinos
    pinMode(interruptPin, INPUT_PULLUP);
    pinMode(buttonSW2, INPUT_PULLUP);

    // Configuração do PWM
    ledcAttach(ledAnalogPin, LED_PWM_FREQ, LED_PWM_RESOLUTION);
    ledcAttach(ledDigitalPin, LED_PWM_FREQ, LED_PWM_RESOLUTION);

    // Criar semáforo binário e filas
    vSemaphoreCreateBinary(xBinarySemaphore);
    xQueueModo = xQueueCreate(10, sizeof(uint8_t));
    xQueuePWM = xQueueCreate(1, sizeof(int));
    xQueueDisplayMode = xQueueCreate(1, sizeof(String));  // Armazena uma posição para o modo atual
    xQueueDisplayState = xQueueCreate(1, sizeof(String)); // Armazena uma posição para o estado atual
    xQueueDisplayData = xQueueCreate(1, sizeof(float));   // Armazena uma posição do valor (Tem. ou Humi.)
    xQueueSensorState = xQueueCreate(1, sizeof(bool));    // Armazenar o estado do sensor DHT11 (se está a ler Temp. ou Humi.)

    // Inicializa semáforos MUTEX
    xMutexDisplay = xSemaphoreCreateMutex(); // MUTEX para o display
    xMutexQueueState = xSemaphoreCreateMutex(); // MUTEX para acesso à fila de estado

    // Criar tarefas
    xTaskCreatePinnedToCore(vHandlerTask, "Handler", 1024, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskHandleSW2, "HandleSW2", 1024, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskModo, "Modo", 1024, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskLDR, "Leitura LDR", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTaskSensorTSL, "Sensor Digital", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTaskDHT, "Leitura DHT", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTask_TFT, "TFT", 4096, NULL, 1, NULL, 1);

    // Configurar interrupções
    attachInterrupt(digitalPinToInterrupt(interruptPin), &vButtonInterruptHandler, FALLING);
    attachInterrupt(digitalPinToInterrupt(buttonSW2), &vButtonInterruptHandler2, FALLING);
}

void loop() {
    vTaskDelete(NULL);
}

// Tarefa que gere o MODO
static void vHandlerTask(void *pvParameters) {
    uint8_t currentModo = 0; // Estados: 0=OFF, 1=ON, 2=AUTO

    // Inicialmente toma o semáforo para garantir que está indisponível até ser usado
    xSemaphoreTake(xBinarySemaphore, 0);

    for (;;) {
    	 // Aguarda que o semáforo seja libertado por outro evento (como uma interrupção
    	 // Se o semáforo estiver disponível, ele "Take" o semáforo e prossegue.
        xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

        // Após o receber o semáforo, atualiza o modo
        currentModo = (currentModo + 1) % 3; // Incrementa o modo de operação, voltando para 0 após 2.
        xQueueSend(xQueueModo, (const void*)&currentModo, portMAX_DELAY);   // Envia o estado atualizado para a fila
    }
}

// Tarefa que gere Temp/Hum.
static void vTaskHandleSW2(void *pvParameters) {
    bool showTemperature = true; // Inicializa com Temperatura como padrão
    for (;;) {
    	// Aguarda o semáforo binário seja libertado pela interrupção do botão SW2
    	if (xSemaphoreTake(xBinarySemaphoreSW2, portMAX_DELAY)) {
            showTemperature = !showTemperature;  // Alterna o estado de showTemperature entre Temperatura e Humidade
            xQueueOverwrite(xQueueSensorState, &showTemperature); // Atualiza a fila com o novo estado de showTemperature
            Serial.println(showTemperature ? "Temperatura" : "Humidade");
        }
    }
}

// Tarefa que processa os modos
static void vTaskModo(void *pvParameters) {
    uint8_t receivedModo;
    String mode = "OFF";
    String state = "OFF";

    for (;;) {
    	// Aguarda receber um valor da fila xQueueModo
        if (xQueueReceive(xQueueModo, &receivedModo, portMAX_DELAY) == pdPASS) {
            switch (receivedModo) {
            case 0: // OFF
                autoModeActive = false;
                isNight = false;
                mode = "OFF";
                state = "OFF";
                ledcWrite(ledAnalogPin, 0); // Desliga LED analógico
                ledcWrite(ledDigitalPin, 0); // Desliga LED digital
                Serial.println("Modo OFF");
                break;

            case 1: // ON
                autoModeActive = false;
                isNight = false;
                mode = "ON";
                state = "Medios";
                ledcWrite(ledAnalogPin, 128);
                Serial.println("Modo ON");
                break;

            case 2: // AUTO
                autoModeActive = true;
                mode = "AUTO";
                Serial.println("Modo AUTO");
                break;
            }

            // Proteção ao acessar a filas compartilhadas usando MUTEX
            xSemaphoreTake(xMutexQueueState, portMAX_DELAY); //obtém MUTEX
            xQueueOverwrite(xQueueDisplayMode, &mode);       //Atualiza queue
            xQueueOverwrite(xQueueDisplayState, &state);	 //Atualiza queue
            xSemaphoreGive(xMutexQueueState);               //Liberta MUTEX
        }
    }
}

//Tarefa sensor analógico LDR
void vTaskLDR(void *pvParameters) {
    int ldrValue = 0;   //armazenar o valor lido do LDR
    String state;       //armazenar o estado atual do sistema

    for (;;) {
        if (autoModeActive) {
            ldrValue = analogRead(pinLDR);  // Lê o valor do sensor LDR (entrada analógica)
            if (ldrValue < LUMINOSITY_THRESHOLD) {
                isNight = true;			// Define que é noite
                Serial.println("LDR: Maximos.");
                state = "Maximos";
            } else {
            	isNight = false;		// Define que não é noite
                Serial.println("LDR: Medios.");
                ledcWrite(ledDigitalPin, 0);
                state = "Medios";
            }
            ledcWrite(ledAnalogPin, 128); //LED An. 50% de int.

            // Protege o acesso à fila compartilhada com MUTEX
            xSemaphoreTake(xMutexQueueState, portMAX_DELAY);  // Obtém o MUTEX
            if (xQueueOverwrite(xQueueDisplayState, &state) != pdPASS) {
                Serial.println("Erro ao enviar o estado para a fila!");
            }
            xSemaphoreGive(xMutexQueueState);// Libera o MUTEX
        }
        // Aguarda 500 ms antes de fazer a próxima leitura
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//Tarefa sensor digital TSL2561
void vTaskSensorTSL(void *pvParameters) {

	// Inicializa o sensor TSL2561
	Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

	// Verifica se o sensor foi corretamente inicializad
    if (!tsl.begin()) {
        Serial.println("Erro ao inicializar o sensor TSL2561!");
        while (1);
    }
    tsl.enableAutoRange(true); // Habilita o ajuste automático de alcance
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS); // Define o tempo de integração para medições rápidas

    portBASE_TYPE xStatus; // Variável para monitorizar o status ao interagir com a fila
    int displayPWM = 0;	   // Armazena o valor PWM que será aplicado ao LED digital

    for (;;) {
        if (autoModeActive && isNight) {
            sensors_event_t event; // Estrutura para armazenar os dados do sensor
            tsl.getEvent(&event);  // Obtém os dados do sensor
            if (event.light) {
            	// Mapeia o valor da luminosidade lida (0 a 1000 lux) para o intervalo PWM (255 a 0)
                displayPWM = map((int)event.light, 0, 1000, 255, 0);
                displayPWM = constrain(displayPWM, 0, 255);// Garante que o valor esteja no intervalo permitido
                ledcWrite(ledDigitalPin, displayPWM);
                Serial.printf("LED digital PWM = %d\n", displayPWM);
            } else {
                Serial.println("Erro ao ler o sensor TSL2561!");
            }
        } else
        	// Se não for noite ou modo AUTO não estiver ativo, desliga o LED digital
            displayPWM = 0;

        // Atualiza a fila `xQueuePWM` com o valor atual do PWM
        xStatus = xQueueOverwrite(xQueuePWM, &displayPWM);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//Tarefa sensor DHT11 - Temperatura/Humidade
void vTaskDHT(void *pvParameters) {
	// Indica se o valor atual a ser lido é a Temperatura (true) ou Humidade (false)
	bool showTemperature = true;
    float sensorValue;			//Armazenar o valor lido do sensor

    for (;;) {
    	// Lê o estado atual (Temperatura ou Humidade)
        if (xQueuePeek(xQueueSensorState, &showTemperature, 0) == pdPASS) {}

        // Lê o sensor de acordo com o estado
        sensorValue = showTemperature ? dht.readTemperature() : dht.readHumidity();
        if (!isnan(sensorValue)) { // Verifica se a leitura do sensor é válida (não é "NaN")

        	// Envia o valor lido para a fila
        	xQueueOverwrite(xQueueDisplayData, &sensorValue);

        	// Imprime o valor no Serial Monitor
			Serial.print(showTemperature ? "Temperatura: " : "Humidade: ");
			Serial.println(sensorValue);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//Tarefa Display
void vTask_TFT(void *pvParameters) {
	//variaveis
	//PWM
	int displayPWM = 0, displayPWM_tmp = 0; //variaveis PWM
	int lastDisplayPWM = -1; // Armazenar o último valor PWM exibido no display
	//DHT
	float displayValue = 0;
    bool showTemperature = true;//Indica se é Tem. ou Hum.
    static bool lastShowTemperature = true; // Armazena o último estado para evitar limpeza desnecessária

    //queues
    portBASE_TYPE xStatus; //armazenar o status de operações relacionadas à fila
    const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;// Tempo de espera para operações na queue
	//Estado
    String state; // Modo atual do sistema
	String lastDisplayState = ""; // Armazenar o último estado exibido no display

	//Modo
	String mode; // Estado atual do sistema
	String lastDisplayMode = ""; // Armazenar o último modo exibido no display

	//Inicialização do Display
    Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);
    tft.begin();
    tft.fillScreen(ILI9341_BLACK); //cor fundo
    tft.setRotation(1); //Orientação
    tft.setTextSize(2); //Tamanho letra
    tft.setTextColor(ILI9341_WHITE); //cor letra

    // Títulos fixos
    String title1 = "Projeto SEEV - 2024/25";
    int16_t x1, y1;
    uint16_t w1, h1;
    // Centraliza e imprime os títulos no display
    tft.getTextBounds(title1, 0, 0, &x1, &y1, &w1, &h1);
    int16_t centeredX1 = (320 - w1) / 2;
    tft.setCursor(centeredX1, 10);
    tft.println(title1);

    String title2 = "Sistema Luzes AUTO";
    int16_t x2, y2;
    uint16_t w2, h2;
    // Centraliza e imprime os títulos no display
    tft.getTextBounds(title2, 0, 0, &x2, &y2, &w2, &h2);
    int16_t centeredX2 = (320 - w2) / 2;
    tft.setCursor(centeredX2, 40);
    tft.println(title2);

    // Labels fixos
	tft.setCursor(10, 90);
	tft.println("MODO:");
	tft.setCursor(10, 120);
	tft.println("ESTADO:");
	tft.setCursor(10, 150);
	tft.println("PWM:");
	tft.setCursor(10, 180);
	tft.println("Temperatura:");

    for (;;) {
    	//Leitura do PWM da queue
        xStatus = xQueuePeek(xQueuePWM, &displayPWM_tmp, xTicksToWait);
        if (xStatus == pdPASS) {
            displayPWM = displayPWM_tmp;
        }

        	//leitura/Atualização do Modo
        if (xQueuePeek(xQueueDisplayMode, &mode, 0) == pdPASS) {
            if (mode != lastDisplayMode) {
                xSemaphoreTake(xMutexDisplay, portMAX_DELAY);
                tft.fillRect(80, 90, 240, 20, ILI9341_BLACK);
                tft.setCursor(80, 90);
                tft.println(mode);
                lastDisplayMode = mode;
                xSemaphoreGive(xMutexDisplay);
            }

				//Leiruta/Atuslização do Estado
            if (xQueuePeek(xQueueDisplayState, &state, 0) == pdPASS) {
                if (state != lastDisplayState) {
                    xSemaphoreTake(xMutexDisplay, portMAX_DELAY);
                    tft.fillRect(100, 120, 240, 20, ILI9341_BLACK);
                    tft.setCursor(100, 120);
                    tft.println(state);
                    lastDisplayState = state;
                    xSemaphoreGive(xMutexDisplay);
                }
                	//Atualização do PWM
                if (displayPWM != lastDisplayPWM) {
                    xSemaphoreTake(xMutexDisplay, portMAX_DELAY);
                    tft.fillRect(80, 150, 240, 20, ILI9341_BLACK);
                    tft.setCursor(80, 150);
                    tft.println(displayPWM);
                    lastDisplayPWM = displayPWM;
                    xSemaphoreGive(xMutexDisplay);
                }
                	//Leitura/atualização do DHT
                if (xQueuePeek(xQueueSensorState, &showTemperature, 0) == pdPASS) {}
                			//Verifica o estado atual da fila (mostrar Temp./Hum.)
                		if (xQueuePeek(xQueueDisplayData, &displayValue, portMAX_DELAY) == pdPASS) {
							xSemaphoreTake(xMutexDisplay, portMAX_DELAY);//Toma o MUTEX
							//Se o estado de exibição (Tem./hum.) mudou desde a última atualização
						if (showTemperature != lastShowTemperature) {
							tft.fillRect(10, 180, 300, 30, ILI9341_BLACK); // Limpa apenas quando o tipo de dado muda
							tft.setCursor(10, 180);
							tft.printf("%s:", showTemperature ? "Temperatura" : "Humidade");
							// Atualiza o estado anterior para evitar futuras limpezas desnecessárias
							lastShowTemperature = showTemperature;
							}

							tft.fillRect(150, 180, 100, 30, ILI9341_BLACK); // Limpa apenas o valor numérico
							tft.setCursor(150, 180);
							tft.printf("%.1f %s", displayValue, showTemperature ? "C" : "%");

							xSemaphoreGive(xMutexDisplay);//Solta o MUTEX
                                        }
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}

//Interrupção associada ao botão SW1
static void IRAM_ATTR vButtonInterruptHandler(void) {
    static volatile unsigned long lastInterruptTime = 0; //A. tempo do ultimo acionamneto
    unsigned long interruptTime = millis(); //Obter tempo em ms

    if (interruptTime - lastInterruptTime > 200) { // Controlo do debounce
    	//Inicializa uma variável para verificar se uma tarefa de alta prioridade foi desbloqueada
    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);//Solta o semáforo
        portYIELD_FROM_ISR();
    }

    lastInterruptTime = interruptTime; // Atualiza o tempo do último acionamento
}

//Interrupção associada ao botão SW2
static void IRAM_ATTR vButtonInterruptHandler2(void) {
    static volatile unsigned long lastInterruptTime = 0; //A. tempo do ultimo acionamneto
    unsigned long interruptTime = millis(); //Obter tempo em ms

    if (interruptTime - lastInterruptTime > 200) { // Controlo do debounce
    	//Inicializa uma variável para verificar se uma tarefa de alta prioridade foi desbloqueada
    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xBinarySemaphoreSW2, &xHigherPriorityTaskWoken);//Solta o semáforo
        portYIELD_FROM_ISR();
    }
    lastInterruptTime = interruptTime; // Atualiza o tempo do último acionamento
}
