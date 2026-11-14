/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Recepcion por interrupciones: cola FIFO para no perder comandos. */
#define UART_QUEUE_LEN 16u
#define N2_RX_QUEUE_LEN 512u
uint8_t datoESP32;
uint8_t datoNucleo2;
volatile uint8_t comandos[UART_QUEUE_LEN];
volatile uint8_t respuestas[N2_RX_QUEUE_LEN];
volatile uint8_t cmdHead = 0, cmdTail = 0;
volatile uint16_t rspHead = 0, rspTail = 0;
volatile uint8_t cmdOverrun = 0, rspOverrun = 0;

typedef enum { ESPERA, ESPERANDO_MEDICION, ESPERANDO_GUARDADO } EstadoNucleo;
static EstadoNucleo estadoNucleo = ESPERA;
static uint32_t inicioSolicitud = 0;
static uint32_t mantenerColorHasta = 0;
static uint32_t mantenerNaranjaHasta = 0;
static uint8_t resultadoGuardadoPendiente = 0; // 1 rosa, 2 cyan
static uint8_t tramaActiva = 0;
static uint16_t indiceTrama = 0;
static char tramaN2[192];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void MensajePC(const char *texto)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)texto,
                      (uint16_t)strlen(texto), 200);
}

/* La cola se escribe desde la ISR y se lee en el loop principal. */
static uint8_t SacarComando(uint8_t *dato)
{
    uint8_t ok = 0;
    __disable_irq();
    if (cmdTail != cmdHead) {
        *dato = comandos[cmdTail];
        cmdTail = (uint8_t)((cmdTail + 1u) % UART_QUEUE_LEN);
        ok = 1;
    }
    __enable_irq();
    return ok;
}

static uint8_t SacarRespuesta(uint8_t *dato)
{
    uint8_t ok = 0;
    __disable_irq();
    if (rspTail != rspHead) {
        *dato = respuestas[rspTail];
        rspTail = (uint16_t)((rspTail + 1u) % N2_RX_QUEUE_LEN);
        ok = 1;
    }
    __enable_irq();
    return ok;
}

#define NEOPIXEL_BITS 24
#define NEOPIXEL_RESET 240
#define NEOPIXEL_SIZE (NEOPIXEL_BITS + NEOPIXEL_RESET)

uint16_t neoData[NEOPIXEL_SIZE];
volatile uint8_t neoBusy = 0;

void NeoPixel_Color(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t color = ((uint32_t)g << 16) |
                     ((uint32_t)r << 8) | b;

    uint32_t inicio = HAL_GetTick();

    while (neoBusy)
    {
        if (HAL_GetTick() - inicio > 100)
            return;
    }

    for (int i = 0; i < 24; i++)
    {
        neoData[i] = (color & (1UL << (23 - i)))
                     ? 67 : 34;
    }

    for (int i = 24; i < NEOPIXEL_SIZE; i++)
    {
        neoData[i] = 0;
    }

    neoBusy = 1;

    if (HAL_TIM_PWM_Start_DMA(&htim1,
                             TIM_CHANNEL_1,
                             (uint32_t *)neoData,
                             NEOPIXEL_SIZE) != HAL_OK)
    {
        neoBusy = 0;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
        neoBusy = 0;
    }
}

// MORADO: esperando comando
void NeoPixel_Espera(void)       { NeoPixel_Color(100, 0, 100); }
// AZUL: tomando medicion
void NeoPixel_Midiendo(void)     { NeoPixel_Color(0, 0, 150); }
// VERDE: movimiento correcto
void NeoPixel_Correcto(void)     { NeoPixel_Color(0, 150, 0); }
// ROJO: movimiento incorrecto o error
void NeoPixel_Incorrecto(void)   { NeoPixel_Color(150, 0, 0); }
// NARANJA: solicitud de guardado
void NeoPixel_Guardando(void)    { NeoPixel_Color(180, 65, 0); }
// CYAN: confirmacion de escritura REAL en SD
void NeoPixel_Guardado(void)     { NeoPixel_Color(0, 150, 150); }
// ROSADO: no guardado en SD
void NeoPixel_NoGuardado(void)   { NeoPixel_Color(160, 0, 90); }

// Compatibilidad con llamadas anteriores
void NeoPixel_Error(void) { NeoPixel_Incorrecto(); }

/* ========== 7 SONIDOS DAC, SIN BLOQUEAR UART ========== */
/* TIM6 actualiza la salida del DAC a 16 kHz.
   TIM1 y su DMA siguen reservados al NeoPixel. */
#define AUDIO_MUESTRAS_HZ 16000u

typedef struct {
    uint16_t frecuencia;
    uint16_t duracion_ms;
} NotaAudio;

typedef enum {
    SON_LISTO = 1,
    SON_MEDICION,
    SON_CORRECTO,
    SON_INCORRECTO,
    SON_GUARDANDO,
    SON_GUARDADO,
    SON_NO_GUARDADO
} Sonido;

/* Misma secuencia de notas que la prueba individual de Hercules. */
static const NotaAudio musicaListo[] = {
    {523,120}, {659,120}, {784,120}, {1047,300}, {0,0}
};
static const NotaAudio musicaMedicion[] = {
    {800,100}, {0,80}, {1100,180}, {0,0}
};
static const NotaAudio musicaCorrecto[] = {
    {659,120}, {880,120}, {1175,350}, {0,0}
};
static const NotaAudio musicaIncorrecto[] = {
    {500,220}, {0,100}, {250,400}, {0,0}
};
static const NotaAudio musicaGuardando[] = {
    {600,80}, {0,180}, {750,80}, {0,180},
    {600,80}, {0,180}, {750,80}, {0,0}
};
static const NotaAudio musicaGuardado[] = {
    {784,100}, {988,100}, {1175,130}, {1568,400}, {0,0}
};
static const NotaAudio musicaNoGuardado[] = {
    {370,160}, {0,90}, {311,160}, {0,90}, {247,450}, {0,0}
};

static volatile uint32_t audioFase = 0;
static volatile uint32_t audioPaso = 0;
static const NotaAudio *audioMelodia = NULL;
static uint8_t audioIndice = 0;
static uint8_t audioRepetir = 0;
static uint32_t audioNotaInicio = 0;

/* Escribir la onda cuadrada en PA4. No hacer HAL_Delay aqui. */
void TIM6_DAC_IRQHandler(void)
{
    if ((TIM6->SR & TIM_SR_UIF) != 0u)
    {
        TIM6->SR &= ~TIM_SR_UIF;
        uint32_t paso = audioPaso;
        if (paso != 0u)
        {
            audioFase += paso;
            DAC->DHR12R1 = (audioFase & 0x80000000u) ? 4095u : 0u;
        }
        else
        {
            DAC->DHR12R1 = 0u;
        }
    }
}

static void Audio_Nota(uint16_t frecuencia)
{
    /* Primero apagar salida antes de actualizar frecuencia. */
    audioPaso = 0u;
    audioFase = 0u;
    DAC->DHR12R1 = 0u;
    if (frecuencia != 0u)
    {
        audioPaso = (uint32_t)
            (((uint64_t)frecuencia * 4294967296ULL) / AUDIO_MUESTRAS_HZ);
    }
}

static void Audio_Parar(void)
{
    audioMelodia = NULL;
    audioRepetir = 0u;
    Audio_Nota(0u);
}

static void Audio_Iniciar(void)
{
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0u);

    __HAL_RCC_TIM6_CLK_ENABLE();
    /* Con el reloj del proyecto, TIM6 recibe 84 MHz: 84e6 / 5250 = 16 kHz. */
    TIM6->CR1 = 0u;
    TIM6->PSC = 0u;
    TIM6->ARR = 5249u;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0u;
    TIM6->DIER = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5u, 0u);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 = TIM_CR1_CEN;
}

static void Audio_Reproducir(Sonido sonido)
{
    Audio_Parar();
    switch (sonido)
    {
        case SON_LISTO:       audioMelodia = musicaListo; break;
        case SON_MEDICION:    audioMelodia = musicaMedicion; break;
        case SON_CORRECTO:    audioMelodia = musicaCorrecto; break;
        case SON_INCORRECTO:  audioMelodia = musicaIncorrecto; break;
        case SON_GUARDANDO:
            audioMelodia = musicaGuardando;
            audioRepetir = 1u;
            break;
        case SON_GUARDADO:    audioMelodia = musicaGuardado; break;
        case SON_NO_GUARDADO:audioMelodia = musicaNoGuardado; break;
        default: return;
    }
    audioIndice = 0u;
    audioNotaInicio = HAL_GetTick();
    Audio_Nota(audioMelodia[0].frecuencia);
}

/* Se llama muchas veces desde el loop principal sin bloquearlo. */
static void Audio_Tarea(void)
{
    if (audioMelodia == NULL) return;
    uint32_t ahora = HAL_GetTick();
    if ((uint32_t)(ahora - audioNotaInicio) <
        audioMelodia[audioIndice].duracion_ms)
        return;

    audioIndice++;
    if (audioMelodia[audioIndice].duracion_ms == 0u)
    {
        if (audioRepetir) audioIndice = 0u;
        else { Audio_Parar(); return; }
    }
    audioNotaInicio = ahora;
    Audio_Nota(audioMelodia[audioIndice].frecuencia);
}
/* ========== FIN AUDIO ========== */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, &datoESP32, 1);

  char inicio[] = "Nucleo 1 lista para recibir comandos\r\n";

  HAL_UART_Transmit(&huart2, (uint8_t*)inicio,
                    strlen(inicio), HAL_MAX_DELAY);

  HAL_UART_Receive_IT(&huart3, &datoNucleo2, 1);

  // Morado fijo mientras espera un comando
  NeoPixel_Espera();
  // DAC PA4 y sonido de inicio (Nucleo 1 encendida).
  Audio_Iniciar();
  Audio_Reproducir(SON_LISTO);
  

  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint8_t dato;
      uint32_t ahora = HAL_GetTick();

      // Procesar primero el UART3: solo tramas @...\n de Nucleo 2.
      while (SacarRespuesta(&dato))
      {
          if (dato == '@') {
              tramaActiva = 1;
              indiceTrama = 0;
              continue;
          }
          if (!tramaActiva) continue; // Ignorar ruido/caracteres viejos.
          if (dato == '\r') continue;
          if (dato != '\n') {
              if (indiceTrama < sizeof(tramaN2) - 1u)
                  tramaN2[indiceTrama++] = (char)dato;
              else { tramaActiva = 0; indiceTrama = 0; }
              continue;
          }
          tramaN2[indiceTrama] = 0;
          tramaActiva = 0;

          if (strcmp(tramaN2, "READY") == 0) {
              MensajePC("Nucleo 2 conectada y lista\r\n");
          } else if (strcmp(tramaN2, "START") == 0) {
              MensajePC("Nucleo 2: midiendo gesto...\r\n");
              NeoPixel_Midiendo();
          } else if (strncmp(tramaN2, "RESULT,", 7) == 0) {
              unsigned muestras = 0, correcto = 0;
              long ud = 0, lr = 0;
              unsigned long tiempo = 0;
              char esperado[24] = {0}, realizado[24] = {0};
              if (sscanf(tramaN2 + 7, "%u,%ld,%ld,%lu,%23[^,],%23[^,],%u",
                         &muestras, &ud, &lr, &tiempo,
                         esperado, realizado, &correcto) == 7) {
                  char salida[230];
                  snprintf(salida, sizeof(salida),
                           "\r\n--- RESULTADO DE MEDICION ---\r\n"
                           "Muestras: %u | UD: %ld | LR: %ld\r\n"
                           "Tiempo: %lu ms\r\n"
                           "Solicitado: %s | Detectado: %s\r\n"
                           "Movimiento: %s\r\n-----------------------------\r\n",
                           muestras, ud, lr, tiempo, esperado, realizado,
                           correcto ? "CORRECTO" : "INCORRECTO");
                  MensajePC(salida);
                  estadoNucleo = ESPERA;
                  if (correcto) {
                      NeoPixel_Correcto();
                      Audio_Reproducir(SON_CORRECTO);
                  } else {
                      NeoPixel_Incorrecto();
                      Audio_Reproducir(SON_INCORRECTO);
                  }
                  mantenerColorHasta = HAL_GetTick() + 2200u;
              } else {
                  MensajePC("ERROR: resultado mal formado de Nucleo 2\r\n");
                  estadoNucleo = ESPERA;
                  NeoPixel_Incorrecto();
                  Audio_Reproducir(SON_INCORRECTO);
                  mantenerColorHasta = HAL_GetTick() + 2200u;
              }
          } else if (strncmp(tramaN2, "SAVE,", 5) == 0) {
              estadoNucleo = ESPERA;
              if (strcmp(tramaN2 + 5, "OK") == 0) {
                  MensajePC("SD: guardado REAL confirmado\r\n");
                  resultadoGuardadoPendiente = 2;
              } else if (strcmp(tramaN2 + 5, "NO_DATA") == 0) {
                  MensajePC("SD: NO GUARDADO; falta una medicion\r\n");
                  resultadoGuardadoPendiente = 1;
              } else {
                  MensajePC("SD: NO GUARDADO; escritura SD no implementada\r\n");
                  resultadoGuardadoPendiente = 1;
              }
              // No apagar naranja de inmediato. Mantener >= 1000 ms.
              if ((int32_t)(mantenerNaranjaHasta - HAL_GetTick()) <= 0)
                  mantenerNaranjaHasta = HAL_GetTick();
          } else if (strncmp(tramaN2, "ERROR,", 6) == 0) {
              MensajePC("Nucleo 2 ERROR: ");
              MensajePC(tramaN2 + 6);
              MensajePC("\r\n");
              estadoNucleo = ESPERA;
              resultadoGuardadoPendiente = 0;
              NeoPixel_Incorrecto();
              Audio_Reproducir(SON_INCORRECTO);
              mantenerColorHasta = HAL_GetTick() + 2200u;
          } else if (strcmp(tramaN2, "BUSY") == 0) {
              MensajePC("Nucleo 2 esta ocupada\r\n");
              // No crear otro resultado de gesto.
          }
      }

      // Bluetooth -> Nucleo 1 USART1 -> Nucleo 2 USART3.
      while (SacarComando(&dato))
      {
          if (dato == '\r' || dato == '\n' || dato == ' ') continue;
          if (dato >= 'a' && dato <= 'z') dato -= 32;
          if (dato != 'L' && dato != 'G') {
              MensajePC("Comando desconocido: usa L o G\r\n");
              continue;
          }
          if (estadoNucleo != ESPERA) {
              MensajePC("Nucleo 2 ocupada: espera el resultado\r\n");
              continue;
          }

          mantenerColorHasta = 0;
          resultadoGuardadoPendiente = 0;
          if (dato == 'L') {
              MensajePC("Solicitando UNA lectura a Nucleo 2\r\n");
              estadoNucleo = ESPERANDO_MEDICION;
              NeoPixel_Midiendo();
              Audio_Reproducir(SON_MEDICION);
          } else {
              MensajePC("Solicitando guardar en SD\r\n");
              estadoNucleo = ESPERANDO_GUARDADO;
              NeoPixel_Guardando();
              Audio_Reproducir(SON_GUARDANDO);
              mantenerNaranjaHasta = HAL_GetTick() + 1200u;
          }

          inicioSolicitud = HAL_GetTick();
          if (HAL_UART_Transmit(&huart3, &dato, 1, 100) != HAL_OK) {
              MensajePC("ERROR TX UART3 hacia Nucleo 2\r\n");
              estadoNucleo = ESPERA;
              resultadoGuardadoPendiente = 0;
              NeoPixel_Incorrecto();
              Audio_Reproducir(SON_INCORRECTO);
              mantenerColorHasta = HAL_GetTick() + 2200u;
          }
      }

      ahora = HAL_GetTick();
      // Mostrar rosa/cyan DESPUES de mantener naranja al menos 1.2s.
      if (resultadoGuardadoPendiente &&
          (int32_t)(ahora - mantenerNaranjaHasta) >= 0) {
          if (resultadoGuardadoPendiente == 2) {
              NeoPixel_Guardado();
              Audio_Reproducir(SON_GUARDADO);
          } else {
              NeoPixel_NoGuardado();
              Audio_Reproducir(SON_NO_GUARDADO);
          }
          resultadoGuardadoPendiente = 0;
          mantenerColorHasta = ahora + 2200u;
      }

      // Vuelve a morado solo despues de mostrar el resultado.
      if (estadoNucleo == ESPERA && mantenerColorHasta &&
          (int32_t)(ahora - mantenerColorHasta) >= 0) {
          mantenerColorHasta = 0;
          NeoPixel_Espera();
      }

      if (cmdOverrun || rspOverrun) {
          __disable_irq();
          uint8_t desborde = cmdOverrun | rspOverrun;
          cmdOverrun = rspOverrun = 0;
          __enable_irq();
          if (desborde) MensajePC("AVISO: desborde UART; datos perdidos\r\n");
      }

      if (estadoNucleo != ESPERA &&
          (uint32_t)(ahora - inicioSolicitud) > 8000u) {
          MensajePC("ERROR: Nucleo 2 no respondio en 8 segundos\r\n");
          estadoNucleo = ESPERA;
          resultadoGuardadoPendiente = 0;
          NeoPixel_Incorrecto();
          mantenerColorHasta = HAL_GetTick() + 2200u;
      }
      Audio_Tarea();
      HAL_Delay(2);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 104;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint8_t siguiente = (uint8_t)((cmdHead + 1u) % UART_QUEUE_LEN);
        if (siguiente != cmdTail) {
            comandos[cmdHead] = datoESP32;
            cmdHead = siguiente;
        } else cmdOverrun = 1;
        HAL_UART_Receive_IT(&huart1, &datoESP32, 1);
    } else if (huart->Instance == USART3) {
        uint16_t siguiente = (uint16_t)((rspHead + 1u) % N2_RX_QUEUE_LEN);
        if (siguiente != rspTail) {
            respuestas[rspHead] = datoNucleo2;
            rspHead = siguiente;
        } else rspOverrun = 1;
        HAL_UART_Receive_IT(&huart3, &datoNucleo2, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &datoESP32, 1);
    } else if (huart->Instance == USART3) {
        HAL_UART_Receive_IT(&huart3, &datoNucleo2, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
