/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Nucleo 2 independiente: APDS-9960, TFT y USART2
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "ili9341.h"
#include "lcd_pins.h"
#include "diskio.h"
#include "zelda.h"
#include "kirby.h"
#include "link.h"
#include "shadow.h"
#include "sonic.h"
#include "flechas.h"
#include "chimuelo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    G_NONE,
    G_UP,
    G_DOWN,
    G_LEFT,
    G_RIGHT
} Gesture;
typedef enum {
    APDS_IDLE,
    APDS_WAIT,
    APDS_DONE,
    APDS_TIMEOUT,
    APDS_UNCLEAR,
    APDS_OVERFLOW,
    APDS_IO_ERROR
} ApdsResult;
typedef struct {
    I2C_HandleTypeDef *bus;
    int32_t ud_delta;
    int32_t lr_delta;
    int16_t first_ud, first_lr;
    int16_t last_ud, last_lr;
    int32_t peak_ud, peak_lr;
    int32_t peak_strength;
    uint16_t saturated_count;
    uint8_t confirmations;
    Gesture candidate;
    uint16_t valid_count;
    uint32_t started;
    uint32_t last_data;
    uint32_t last_service;
    uint8_t batch[32][4];
    uint8_t batch_count;
    uint8_t active;
    uint8_t ready;
    uint8_t saw_data;
    Gesture gesture;
} Apds9960;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N1_RX_TAM 32u
#define APDS_ADDR               ((uint16_t)(0x39u << 1))
#define APDS_TIMEOUT_MS         3000u
#define APDS_FIFO_PERIOD_MS     30u
#define APDS_THRESHOLD_OUT      10u
#define APDS_SENSITIVITY        35
#define APDS_FLIP_UD            0
#define APDS_FLIP_LR            0
#define APDS_REG_ENABLE         0x80u
#define APDS_REG_GCONF4         0xABu
#define APDS_REG_GFLVL          0xAEu
#define APDS_REG_GSTATUS        0xAFu
#define APDS_REG_FIFO           0xFCu
#define DIR_NONE                G_NONE
#define DIR_UP                  G_UP
#define DIR_DOWN                G_DOWN
#define DIR_LEFT                G_LEFT
#define DIR_RIGHT               G_RIGHT
#define FONDO  0x0842u
#define PANEL  0x10A4u
#define BLANCO 0xFFFFu
#define CELESTE 0x07FFu
#define VERDE  0x07E0u
#define ROJO   0xF800u
#define BORDE  0x21CEu

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static uint8_t n1Byte;
static volatile uint8_t n1Cola[N1_RX_TAM];
static volatile uint8_t n1Entrada, n1Salida, n1Error;
static volatile uint32_t n1BytesRX;
static uint32_t n1Ordenes, n1TramasTX;
static FATFS sistemaSD;
static FIL archivoSD;

static Apds9960 sensor;
static uint8_t midiendo;
static Gesture direccionEsperada;
static uint32_t inicioEstimulo;
static uint32_t semilla = 0xA341316Cu;
static struct {
    Gesture esperado, realizado;
    uint32_t tiempo;
    int32_t ud, lr;
    uint16_t muestras, saturadas;
    uint8_t correcto, valido, guardado;
} ultimoResultado;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Imprimir(const char *formato, ...);
static uint8_t DiagnosticarSensor(void);
static void EscanearBus(void);
static void IniciarPrueba(void);
static void TareaSensor(void);
static void RecibirHercules(void);
static Gesture SortearDireccion(void);
static void PantallaInicio(void);
static void PantallaEstimulo(Gesture direccion);
static void PantallaResultado(Gesture esperado, Gesture realizado, uint32_t tiempo);
static void PantallaError(const char *detalle);
static void N1_Enviar(const char *formato, ...);
static void N1_Tarea(void);
static void N1_Resultado(void);
static void DiagnosticarEnlace(void);
static FRESULT SD_Montar(void);
static void SD_MostrarError(FRESULT r);
static void SD_Probar(void);
static void SD_Verificar(void);
static FRESULT GuardarSD(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef read_bytes(
    Apds9960 *d,
    uint8_t reg,
    uint8_t *datos,
    uint16_t cantidad)
{
    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(
        d->bus,
        APDS_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        datos,
        cantidad,
        40
    );
    if (r != HAL_OK)
        Imprimir("ERROR READ reg=0x%02X HAL=%d ERR=0x%08lX\r\n",
                 reg, (int)r, (unsigned long)HAL_I2C_GetError(d->bus));
    return r;
}
static HAL_StatusTypeDef write_reg(
    Apds9960 *d,
    uint8_t reg,
    uint8_t valor)
{
    HAL_StatusTypeDef r = HAL_I2C_Mem_Write(
        d->bus,
        APDS_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &valor,
        1,
        40
    );
    if (r != HAL_OK)
        Imprimir("ERROR WRITE reg=0x%02X valor=0x%02X HAL=%d ERR=0x%08lX\r\n",
                 reg, valor, (int)r, (unsigned long)HAL_I2C_GetError(d->bus));
    return r;
}
HAL_StatusTypeDef APDS_ReadID(Apds9960 *d, uint8_t *id)
{
    return read_bytes(d, 0x92u, id, 1);
}
HAL_StatusTypeDef APDS_Stop(Apds9960 *d)
{
    d->active = 0;
    HAL_StatusTypeDef a =
        write_reg(d, APDS_REG_GCONF4, 0);
    HAL_StatusTypeDef b =
        write_reg(d, APDS_REG_ENABLE, 0x01u);
    if (a != HAL_OK || b != HAL_OK) {
        d->ready = 0;
        return HAL_ERROR;
    }
    return HAL_OK;
}
HAL_StatusTypeDef APDS_Init(
    Apds9960 *d,
    I2C_HandleTypeDef *bus)
{
    memset(d, 0, sizeof(*d));
    d->bus = bus;
    uint8_t id;
    if (APDS_ReadID(d, &id) != HAL_OK)
        return HAL_ERROR;
    if (id != 0xABu && id != 0x9Cu)
        return HAL_ERROR;
    static const uint8_t ajustes[][2] = {
        {APDS_REG_ENABLE, 0},
        {0x81, 219},
        {0x83, 0xFF},
        {0x8E, 0x89},
        {0x9D, 0},
        {0x9E, 0},
        {0x8D, 0x60},
        {0x8F, 0x09},
        {0x90, 0x31},
        {0x9F, 0},
        {0xA0, 40},
        {0xA1, 30},
        {0xA2, 0x40},
        {0xA3, 0x41},
        {0xA4, 0},
        {0xA5, 0},
        {0xA7, 0},
        {0xA9, 0},
        {0xA6, 0xC9},
        {0xAA, 0},
        {APDS_REG_GCONF4, 0},
        {APDS_REG_ENABLE, 1}
    };
    for (unsigned i = 0;
         i < sizeof(ajustes) / sizeof(ajustes[0]);
         i++) {
        if (write_reg(
                d,
                ajustes[i][0],
                ajustes[i][1]) != HAL_OK) {
            (void)APDS_Stop(d);
            return HAL_ERROR;
        }
    }
    HAL_Delay(3);
    d->ready = 1;
    return HAL_OK;
}
HAL_StatusTypeDef APDS_Start(Apds9960 *d)
{
    if (!d->ready || d->active)
        return HAL_ERROR;
    if (APDS_Stop(d) != HAL_OK ||
        write_reg(d, APDS_REG_ENABLE, 0) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(3);
    /* Limpiar FIFO y datos anteriores. */
    if (write_reg(d, APDS_REG_GCONF4, 0x04u) != HAL_OK ||
        write_reg(d, APDS_REG_GCONF4, 0) != HAL_OK)
        return HAL_ERROR;
    d->peak_ud = d->peak_lr = 0;
    d->peak_strength = 0;
    d->saturated_count = 0;
    d->confirmations = 0;
    d->candidate = DIR_NONE;
    d->ud_delta = 0;
    d->lr_delta = 0;
    d->first_ud = d->first_lr = 0;
    d->last_ud = d->last_lr = 0;
    d->valid_count = 0;
    d->batch_count = 0;
    d->gesture = DIR_NONE;
    d->saw_data = 0;
    if (write_reg(d, APDS_REG_ENABLE, 1) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(3);
    /*
     * GIEN = 0: no requiere interrupcion INT.
     * GMODE = 1.
     * ENABLE: PON + WEN + PEN + GEN.
     */
    if (write_reg(d, APDS_REG_GCONF4, 0x01u) != HAL_OK ||
        write_reg(d, APDS_REG_ENABLE, 0x4Du) != HAL_OK) {
        (void)APDS_Stop(d);
        return HAL_ERROR;
    }
    d->started = HAL_GetTick();
    d->last_data = d->started;
    d->last_service =
        d->started - APDS_FIFO_PERIOD_MS;
    d->active = 1;
    return HAL_OK;
}
static int valid_sample(const uint8_t muestra[4])
{
    return muestra[0] > APDS_THRESHOLD_OUT &&
           muestra[1] > APDS_THRESHOLD_OUT &&
           muestra[2] > APDS_THRESHOLD_OUT &&
           muestra[3] > APDS_THRESHOLD_OUT;
}
/* La direccion requiere un eje dominante y dos muestras consecutivas. */
static Gesture classify_delta(int32_t ud, int32_t lr)
{
    if (APDS_FLIP_UD) ud = -ud;
    if (APDS_FLIP_LR) lr = -lr;

    int32_t au = labs(ud);
    int32_t al = labs(lr);

    if (au >= APDS_SENSITIVITY && au * 4 > al * 5)
        return ud > 0 ? DIR_DOWN : DIR_UP;

    if (al >= APDS_SENSITIVITY && al * 4 > au * 5)
        return lr > 0 ? DIR_RIGHT : DIR_LEFT;

    return DIR_NONE;
}

static void process_batch(Apds9960 *d)
{
    static uint32_t ultimoReporte;
    for (unsigned i = 0; i < d->batch_count; i++)
    {
        const uint8_t *m = d->batch[i];
        if (!valid_sample(m))
        {
            d->candidate = DIR_NONE;
            d->confirmations = 0;
            continue;
        }

        /* Una muestra recortada no permite comparar bien los canales. */
        if (m[0] == 255u || m[1] == 255u ||
            m[2] == 255u || m[3] == 255u)
        {
            if (d->saturated_count < 65535u) d->saturated_count++;
            d->confirmations = 0;
            d->candidate = DIR_NONE;
            continue;
        }

        int ud = 100 * ((int)m[0] - (int)m[1]) /
                       ((int)m[0] + (int)m[1]);
        int lr = 100 * ((int)m[2] - (int)m[3]) /
                       ((int)m[2] + (int)m[3]);
        if (d->valid_count == 0)
        {
            d->first_ud = (int16_t)ud;
            d->first_lr = (int16_t)lr;
        }
        d->last_ud = (int16_t)ud;
        d->last_lr = (int16_t)lr;
        if (d->valid_count < 65535u) d->valid_count++;
        d->ud_delta = (int32_t)d->last_ud - d->first_ud;
        d->lr_delta = (int32_t)d->last_lr - d->first_lr;

        Gesture g = classify_delta(d->ud_delta, d->lr_delta);
        if (g == DIR_NONE)
        {
            d->candidate = DIR_NONE;
            d->confirmations = 0;
            continue;
        }
        if (g == d->candidate)
        {
            if (d->confirmations < 255u) d->confirmations++;
        }
        else
        {
            d->candidate = g;
            d->confirmations = 1;
        }

        int32_t strength = labs(d->ud_delta);
        if (labs(d->lr_delta) > strength) strength = labs(d->lr_delta);
        if (d->confirmations >= 2u && strength > d->peak_strength)
        {
            d->peak_strength = strength;
            d->peak_ud = d->ud_delta;
            d->peak_lr = d->lr_delta;
        }
    }

    uint32_t ahora = HAL_GetTick();
    if (d->batch_count && (uint32_t)(ahora - ultimoReporte) >= 150u)
    {
        const uint8_t *m = d->batch[d->batch_count - 1u];
        Imprimir("RAW U=%u D=%u L=%u R=%u | UD=%ld LR=%ld N=%u SAT=%u\r\n",
                 (unsigned)m[0], (unsigned)m[1],
                 (unsigned)m[2], (unsigned)m[3],
                 (long)d->ud_delta, (long)d->lr_delta,
                 (unsigned)d->valid_count, (unsigned)d->saturated_count);
        ultimoReporte = ahora;
    }
    d->batch_count = 0;
}

static Gesture decode_gesture(const Apds9960 *d)
{
    if (d->valid_count < 5u || !d->peak_strength) return DIR_NONE;
    return classify_delta(d->peak_ud, d->peak_lr);
}

static ApdsResult finish(
    Apds9960 *d,
    ApdsResult resultado)
{
    return APDS_Stop(d) == HAL_OK
           ? resultado
           : APDS_IO_ERROR;
}
ApdsResult APDS_Poll(Apds9960 *d)
{
    if (!d->active)
        return APDS_IDLE;
    uint32_t ahora = HAL_GetTick();
    if ((uint32_t)(ahora - d->started) >= APDS_TIMEOUT_MS)
    {
        d->ud_delta = (int32_t)d->last_ud - d->first_ud;
        d->lr_delta = (int32_t)d->last_lr - d->first_lr;

        Imprimir("CIERRE POR TIEMPO: UD %d->%d, LR %d->%d\r\n",
                 (int)d->first_ud, (int)d->last_ud,
                 (int)d->first_lr, (int)d->last_lr);

        if (d->valid_count < 5u)
            return finish(d, APDS_TIMEOUT);

        /* Aunque no haya salido del modo de gestos, evaluar lo recibido. */
        d->gesture = decode_gesture(d);
        return finish(d, d->gesture == DIR_NONE
                         ? APDS_UNCLEAR : APDS_DONE);
    }
    if ((uint32_t)(ahora - d->last_service) <
        APDS_FIFO_PERIOD_MS)
        return APDS_WAIT;
    d->last_service = ahora;
    uint8_t estado;
    uint8_t nivel;
    uint8_t modo;
    uint8_t datos[128];
    if (read_bytes(
            d, APDS_REG_GSTATUS, &estado, 1) != HAL_OK ||
        read_bytes(
            d, APDS_REG_GFLVL, &nivel, 1) != HAL_OK) {
        return finish(d, APDS_IO_ERROR);
    }
    if ((estado & 0x02u) || nivel > 32u)
        return finish(d, APDS_OVERFLOW);
    if (nivel) {
        if (read_bytes(
                d,
                APDS_REG_FIFO,
                datos,
                (uint16_t)(nivel * 4u)) != HAL_OK) {
            return finish(d, APDS_IO_ERROR);
        }
        for (unsigned i = 0; i < nivel; i++) {
            memcpy(
                d->batch[d->batch_count++],
                &datos[i * 4u],
                4
            );
            if (d->batch_count == 32u)
                process_batch(d);
        }
        process_batch(d);
        d->saw_data = 1;
        d->last_data = HAL_GetTick();

        /* Terminar en cuanto el algoritmo confirme una direccion. */
        Gesture detectado = decode_gesture(d);

        if (detectado != DIR_NONE)
        {
            d->gesture = detectado;
            return finish(d, APDS_DONE);
        }
    }
    if (read_bytes(
            d, APDS_REG_GCONF4, &modo, 1) != HAL_OK)
        return finish(d, APDS_IO_ERROR);
    ahora = HAL_GetTick();
    if (!nivel &&
        decode_gesture(d) != DIR_NONE &&
        d->saw_data &&
        !(estado & 1u) &&
        !(modo & 1u) &&
        (uint32_t)(ahora - d->last_data) >=
            APDS_FIFO_PERIOD_MS) {
        d->ud_delta = (int32_t)d->last_ud - d->first_ud;
        d->lr_delta = (int32_t)d->last_lr - d->first_lr;
        d->gesture = decode_gesture(d);
        return finish(
            d,
            d->gesture == DIR_NONE
                ? APDS_UNCLEAR
                : APDS_DONE
        );
    }
    return APDS_WAIT;
}
const char *Gesture_Name(Gesture gesto)
{
    switch (gesto) {
    case DIR_UP:
        return "ARRIBA";
    case DIR_DOWN:
        return "ABAJO";
    case DIR_LEFT:
        return "IZQUIERDA";
    case DIR_RIGHT:
        return "DERECHA";
    default:
        return "SIN DATOS";
    }
}


/* Graficos de la TFT. Las imagenes permanecen en sus archivos .h. */
static void Centrar(const char *texto, int y, int fuente,
                    uint16_t color, uint16_t fondo)
{
    int ancho = fuente == 1 ? 8 : 16;
    int x = (320 - (int)strlen(texto) * ancho) / 2;
    if (x < 0) x = 0;
    LCD_Print((char *)texto, x, y, fuente, color, fondo);
}

static void Cabecera(const char *texto, uint16_t color)
{
    LCD_Clear(FONDO);
    FillRect(0, 0, 320, 34, color);
    Centrar(texto, 10, 2, BLANCO, color);
}

static void Pie(const char *texto)
{
    FillRect(0, 220, 320, 20, FONDO);
    Centrar(texto, 224, 1, BLANCO, FONDO);
}

static void DibujarPersonaje(Gesture g, uint16_t centro, uint16_t y)
{
    const uint16_t *imagen;
    uint16_t w, h, transparente;
    switch (g)
    {
    case G_UP:
        imagen = link; w = link_width; h = link_height;
        transparente = LINK_TRANSPARENTE;
        break;
    case G_DOWN:
        imagen = sonic; w = sonic_width; h = sonic_height;
        transparente = SONIC_TRANSPARENTE;
        break;
    case G_LEFT:
        imagen = shadow; w = shadow_width; h = shadow_height;
        transparente = SHADOW_TRANSPARENTE;
        break;
    case G_RIGHT:
        imagen = kirby; w = kirby_width; h = kirby_height;
        transparente = KIRBY_TRANSPARENTE;
        break;
    default:
        return;
    }
    LCD_BitmapTransparent(centro - w / 2u, y, w, h, imagen, transparente);
}

static void DibujarFlecha(Gesture g, uint16_t x, uint16_t y,
                          uint16_t color)
{
    const uint16_t *imagen;
    switch (g)
    {
    case G_UP:    imagen = imagen_ARRIBA; break;
    case G_DOWN:  imagen = imagen_ABAJO; break;
    case G_LEFT:  imagen = imagen_IZQUIERDA; break;
    case G_RIGHT: imagen = imagen_DERECHA; break;
    default: return;
    }

    /* Ampliacion de 48x48 a 96x96 con transparencia. */
    uint16_t fila[96];
    for (unsigned fy = 0; fy < 48u; fy++)
    {
        for (unsigned fx = 0; fx < 48u; fx++)
        {
            uint16_t pixel = imagen[fy * 48u + fx];
            if (pixel != FLECHA_TRANSPARENTE) pixel = color;
            fila[fx * 2u] = pixel;
            fila[fx * 2u + 1u] = pixel;
        }
        for (unsigned k = 0; k < 2u; k++)
        {
            LCD_BitmapTransparent(x, y + fy * 2u + k, 96, 1,
                                  fila, FLECHA_TRANSPARENTE);
        }
    }
}

static void PantallaInicio(void)
{
    LCD_Bitmap(0, 0, zelda_width, zelda_height, zelda);
    Centrar("ESPERANDO DATOS", 10, 2, BLANCO, FONDO);
    Pie("ESPERANDO ORDEN DE NUCLEO 1");
}

static void PantallaEstimulo(Gesture direccion)
{
    Cabecera("SIGUE LA DIRECCION", 0x0193u);
    FillRect(12, 42, 296, 165, PANEL);
    Rect(12, 42, 296, 165, BORDE);

    if (direccion == G_RIGHT)
    {
        /* Kirby ya contiene su flecha. */
        DibujarPersonaje(direccion, 160, 51);
    }
    else
    {
        DibujarPersonaje(direccion, 84, 51);
        DibujarFlecha(direccion, 188, 63, CELESTE);
    }
    Centrar(Gesture_Name(direccion), 186, 1, CELESTE, PANEL);
    Pie("HAZ UN SOLO MOVIMIENTO");
}

static void PantallaResultado(Gesture esperado, Gesture realizado,
                              uint32_t tiempo)
{
    char texto[32];
    snprintf(texto, sizeof(texto), "%lu.%03lu S",
             (unsigned long)(tiempo / 1000u),
             (unsigned long)(tiempo % 1000u));

    if (realizado == G_NONE)
    {
        const uint16_t claro = 0xDFFFu;
        Cabecera("SIN GESTO", 0xA800u);
        FillRect(0, 40, 320, 177, claro);
        Centrar("MOVIMIENTO NO RECONOCIDO", 43, 1, 0x0000u, claro);
        LCD_BitmapTransparent(40, 63, chimuelo_width, chimuelo_height,
                              chimuelo, CHIMUELO_TRANSPARENTE);
        Centrar(texto, 201, 1, 0x0000u, claro);
    }
    else
    {
        uint8_t correcto = esperado == realizado;
        uint16_t color = correcto ? VERDE : ROJO;
        Cabecera(correcto ? "CORRECTO" : "INCORRECTO",
                  correcto ? 0x03A0u : 0xA800u);
        FillRect(12, 42, 136, 174, PANEL);
        Rect(12, 42, 136, 174, BORDE);
        DibujarPersonaje(esperado, 80, 51);
        LCD_Print((char *)Gesture_Name(esperado),
                  80 - (int)strlen(Gesture_Name(esperado)) * 4,
                  190, 1, CELESTE, PANEL);
        LCD_Print("TIEMPO DE RESPUESTA", 160, 46, 1, BLANCO, FONDO);
        LCD_Print(texto, 166, 66, 2, color, FONDO);
        LCD_Print("SOLICITADO", 166, 106, 1, BLANCO, FONDO);
        LCD_Print((char *)Gesture_Name(esperado), 166, 124, 1, CELESTE, FONDO);
        LCD_Print("REALIZADO", 166, 156, 1, BLANCO, FONDO);
        LCD_Print((char *)Gesture_Name(realizado), 166, 174, 1, color, FONDO);
    }
    Pie("ESPERANDO ORDEN DE NUCLEO 1");
}

static void PantallaError(const char *detalle)
{
    Cabecera("ERROR SENSOR", 0xA800u);
    Centrar(detalle, 90, 1, BLANCO, FONDO);
    Centrar("REVISA SENSOR Y CONEXIONES", 120, 1, CELESTE, FONDO);
    Pie("REINTENTA DESDE NUCLEO 1");
}

static Gesture SortearDireccion(void)
{
    semilla ^= HAL_GetTick();
    if (!semilla) semilla = 0xA341316Cu;
    semilla ^= semilla << 13;
    semilla ^= semilla >> 17;
    semilla ^= semilla << 5;
    return (Gesture)(G_UP + (semilla & 3u));
}


static void N1_Enviar(const char *formato, ...)
{
    char trama[160];
    va_list args;
    va_start(args, formato);
    int n = vsnprintf(trama, sizeof(trama), formato, args);
    va_end(args);
    if (n <= 0 || n >= (int)sizeof(trama)) {
        Imprimir("ERROR: trama UART3 demasiado larga.\r\n");
        return;
    }
    if (HAL_UART_Transmit(&huart3, (uint8_t *)trama,
                          (uint16_t)n, 250) != HAL_OK) {
        Imprimir("ERROR TX UART3 hacia Nucleo 1.\r\n");
        return;
    }
    n1TramasTX++;
    if (trama[n - 1] == '\n') trama[n - 1] = 0;
    Imprimir("N2 -> N1: %s\r\n", trama);
}

static void N1_Resultado(void)
{
    N1_Enviar("@RESULT,%u,%ld,%ld,%lu,%s,%s,%u\n",
              (unsigned)ultimoResultado.muestras,
              (long)ultimoResultado.ud,
              (long)ultimoResultado.lr,
              (unsigned long)ultimoResultado.tiempo,
              Gesture_Name(ultimoResultado.esperado),
              ultimoResultado.realizado == G_NONE ? "NO_RECONOCIDO"
                  : Gesture_Name(ultimoResultado.realizado),
              (unsigned)ultimoResultado.correcto);
}

static void N1_Tarea(void)
{
    if (n1Error)
    {
        (void)HAL_UART_AbortReceive(&huart3);
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        n1Entrada = n1Salida = 0;
        n1Error = 0;
        if (HAL_UART_Receive_IT(&huart3, &n1Byte, 1) != HAL_OK)
            n1Error = 1;
        Imprimir("ERROR RX USART3: reenvia la orden desde Nucleo 1.\r\n");
        N1_Enviar("@ERROR,UART_RX\n");
        return;
    }

    if (n1Salida == n1Entrada) return;
    uint8_t dato = n1Cola[n1Salida];
    n1Salida = (uint8_t)((n1Salida + 1u) % N1_RX_TAM);
    if (dato == '\r' || dato == '\n' || dato == ' ' || dato == '\t') return;

    Imprimir("N1 -> N2: %c recibido.\r\n", dato);
    if (dato == 'L' || dato == 'l' || dato == 'G' || dato == 'g') {
        n1Ordenes++;
        Imprimir("ENLACE RX CONFIRMADO: orden recibida desde Nucleo 1.\r\n");
    }
    if (midiendo)
    {
        N1_Enviar("@BUSY\n");
        return;
    }

    switch (dato) {
    case 'L':
    case 'l':
        IniciarPrueba();
        break;
    case 'G':
    case 'g':
        if (!ultimoResultado.valido) {
            Pie("SIN MEDICION PARA GUARDAR");
            N1_Enviar("@SAVE,NO_DATA\n");
        } else {
            FRESULT r = GuardarSD();
            if (r == FR_OK) N1_Enviar("@SAVE,OK\n");
            else N1_Enviar("@ERROR,SD_FATFS_%u\n", (unsigned)r);
        }
        break;
    default:
        N1_Enviar("@ERROR,COMANDO\n");
        break;
    }
}

static void DiagnosticarEnlace(void)
{
    uint32_t modoTX = (GPIOB->MODER >> 20u) & 3u;
    uint32_t afTX = (GPIOB->AFR[1] >> 8u) & 15u;
    uint32_t modoRX = (GPIOC->MODER >> 10u) & 3u;
    uint32_t afRX = (GPIOC->AFR[0] >> 20u) & 15u;
    uint32_t irq = NVIC_GetEnableIRQ(USART3_IRQn);
    Imprimir("\r\n--- DIAGNOSTICO ENLACE NUCLEO 1 ---\r\n");
    Imprimir("USART3: PB10=TX, PC5=RX, 115200 8N1.\r\n");
    Imprimir("GPIO TX PB10: modo=%lu AF=%lu | RX PC5: modo=%lu AF=%lu\r\n",
             (unsigned long)modoTX, (unsigned long)afTX,
             (unsigned long)modoRX, (unsigned long)afRX);
    Imprimir("NVIC USART3=%lu | RX armada=%u\r\n",
             (unsigned long)irq,
             (unsigned)(huart3.RxState == HAL_UART_STATE_BUSY_RX));
    Imprimir("RX bytes=%lu | Ordenes L/G=%lu | TX tramas=%lu\r\n",
             (unsigned long)n1BytesRX, (unsigned long)n1Ordenes,
             (unsigned long)n1TramasTX);
    if (modoTX != 2u || afTX != 7u || modoRX != 2u || afRX != 7u || !irq)
        Imprimir("ERROR CONFIG: genera MX con USART3 PB10/PC5 AF7 e interrupcion habilitada.\r\n");
    else
        Imprimir("CONFIG USART3 OK: pines AF7 e interrupcion habilitada.\r\n");
    if (!n1Ordenes)
        Imprimir("Sin ordenes recibidas aun. Envia L desde Bluetooth hacia Nucleo 1.\r\n");
    else
        Imprimir("RX CONFIRMADO: Nucleo 2 recibio ordenes de Nucleo 1.\r\n");
    Imprimir("TX OK confirma salida del STM32; el programa de Nucleo 1 no devuelve ACK.\r\n");
}

static void Imprimir(const char *formato, ...)
{
    char texto[192];
    va_list argumentos;
    va_start(argumentos, formato);
    int n = vsnprintf(texto, sizeof(texto), formato, argumentos);
    va_end(argumentos);
    if (n <= 0) return;
    if (n >= (int)sizeof(texto)) n = sizeof(texto) - 1;
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)texto, (uint16_t)n, 250);
}

static uint8_t DiagnosticarSensor(void)
{
    Imprimir("\r\n--- DIAGNOSTICO APDS-9960 ---\r\n");
    Imprimir("SCL(PB8)=%u SDA(PB9)=%u\r\n",
             (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
             (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));

    HAL_StatusTypeDef r = HAL_I2C_IsDeviceReady(&hi2c1, APDS_ADDR, 3, 100);
    uint32_t error = HAL_I2C_GetError(&hi2c1);
    Imprimir("READY 0x39: HAL=%d ERR=0x%08lX\r\n", (int)r, (unsigned long)error);
    if (r != HAL_OK) {
        Imprimir("FALLO: no se confirmo respuesta en 0x39.\r\n");
        return 0;
    }

    sensor.bus = &hi2c1;
    uint8_t id = 0;
    r = APDS_ReadID(&sensor, &id);
    error = HAL_I2C_GetError(&hi2c1);
    Imprimir("ID registro 0x92: HAL=%d ID=0x%02X ERR=0x%08lX\r\n",
             (int)r, (unsigned)id, (unsigned long)error);
    if (r != HAL_OK) return 0;
    if (id != 0xABu && id != 0x9Cu) {
        Imprimir("Hay respuesta I2C, pero el ID no es uno de los aceptados.\r\n");
        return 0;
    }
    Imprimir("OK: comunicacion I2C e identificador reconocidos.\r\n");
    return 1;
}

static void EscanearBus(void)
{
    if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) ||
        !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) {
        Imprimir("No escaneo: SCL o SDA esta baja en reposo. Revisa el bus.\r\n");
        return;
    }
    Imprimir("\r\nEscaneando direcciones I2C de 0x08 a 0x77...\r\n");
    unsigned encontrados = 0;
    for (uint16_t direccion = 0x08; direccion <= 0x77; direccion++) {
        HAL_StatusTypeDef r = HAL_I2C_IsDeviceReady(&hi2c1, direccion << 1, 1, 10);
        uint32_t error = HAL_I2C_GetError(&hi2c1);
        if (r == HAL_OK) {
            Imprimir("Responde: 0x%02X%s\r\n", (unsigned)direccion,
                     direccion == 0x39 ? " (direccion del APDS-9960)" : "");
            encontrados++;
        } else if (r == HAL_BUSY || r == HAL_TIMEOUT ||
                   (error & HAL_I2C_ERROR_TIMEOUT)) {
            Imprimir("Escaneo detenido: HAL=%d ERR=0x%08lX\r\n",
                     (int)r, (unsigned long)error);
            return;
        }
    }
    Imprimir("Fin del escaneo. Dispositivos encontrados: %u\r\n", encontrados);
}

/* SD: acceso exclusivo desde el ciclo principal, nunca desde una ISR. */
static FRESULT SD_Montar(void)
{
    (void)f_mount(NULL, "0:", 0);
    DSTATUS antes = disk_status(0);
    DSTATUS inicio = disk_initialize(0);
    DSTATUS despues = disk_status(0);
    Imprimir("SD ESTADO: antes=0x%02X init=0x%02X despues=0x%02X\r\n",
             (unsigned)antes, (unsigned)inicio, (unsigned)despues);
    if (inicio & STA_NOINIT) return FR_NOT_READY;
    return f_mount(&sistemaSD, "0:", 1);
}

static void SD_MostrarError(FRESULT r)
{
    Pie("ERROR SD: REVISA HERCULES");
    Imprimir("ERROR SD FatFs=%u\r\n", (unsigned)r);
    if (r == FR_NOT_READY)
        Imprimir("La inicializacion SPI de la SD no termino. Revisa SDDBG CMD0/CMD8.\r\n");
    else if (r == FR_NO_FILESYSTEM)
        Imprimir("La tarjeta responde, pero no se encontro un sistema FAT valido.\r\n");
    else if (r == FR_DISK_ERR)
        Imprimir("Fallo de lectura, escritura o verificacion de la tarjeta.\r\n");
    Imprimir("No se confirmo guardado. El enlace y el sensor siguen disponibles.\r\n");
}

static void SD_Probar(void)
{
    Imprimir("\r\n--- PRUEBA SD: INICIO Y MONTAJE, SIN ESCRIBIR ---\r\n");
    FRESULT r = SD_Montar();
    if (r != FR_OK) { SD_MostrarError(r); return; }
    DWORD sectores = 0;
    if (disk_ioctl(0, GET_SECTOR_COUNT, &sectores) != RES_OK)
    {
        SD_MostrarError(FR_DISK_ERR);
        return;
    }
    Imprimir("SD OK: montaje FAT correcto; dispositivo de %lu MiB (%lu sectores).\r\n",
             (unsigned long)(sectores / 2048u), (unsigned long)sectores);
    Imprimir("No se escribio ningun archivo. V prueba escritura y lectura.\r\n");
    Pie("SD MONTADA - V PARA VERIFICAR");
}

static FRESULT SD_Escribir(const char *texto)
{
    UINT escritos = 0;
    UINT longitud = (UINT)strlen(texto);
    FRESULT r = f_write(&archivoSD, texto, longitud, &escritos);
    return r == FR_OK && escritos != longitud ? FR_DISK_ERR : r;
}

/* Solo intenta retirar los bytes agregados durante una operacion fallida. */
static void SD_RetirarParcial(DWORD tamAnterior)
{
    FRESULT r = f_lseek(&archivoSD, tamAnterior);
    if (r == FR_OK) r = f_truncate(&archivoSD);
    if (r != FR_OK)
        Imprimir("AVISO: no se pudo retirar una escritura parcial.\r\n");
}

static FRESULT GuardarSD(void)
{
    if (ultimoResultado.guardado)
    {
        Pie("DATOS YA GUARDADOS");
        Imprimir("SD: esta medicion ya se guardo; no se repite la fila.\r\n");
        return FR_OK;
    }

    Imprimir("SD: guardando medicion en 0:/GESTOS.CSV...\r\n");
    FRESULT r = SD_Montar();
    if (r != FR_OK) { SD_MostrarError(r); return r; }
    r = f_open(&archivoSD, "0:/GESTOS.CSV", FA_OPEN_ALWAYS | FA_WRITE);
    if (r != FR_OK) { SD_MostrarError(r); return r; }

    DWORD tamAnterior = (DWORD)f_size(&archivoSD);
    uint8_t intentoEscritura = 0;
    r = f_lseek(&archivoSD, tamAnterior);
    if (r == FR_OK && tamAnterior == 0u)
    {
        intentoEscritura = 1;
        r = SD_Escribir("solicitado,realizado,tiempo_ms,correcto,muestras,ud_pico,lr_pico,saturadas\r\n");
    }
    if (r == FR_OK)
    {
        char fila[160];
        int n = snprintf(fila, sizeof(fila), "%s,%s,%lu,%u,%u,%ld,%ld,%u\r\n",
                         Gesture_Name(ultimoResultado.esperado),
                         ultimoResultado.realizado == G_NONE ? "NO_RECONOCIDO"
                             : Gesture_Name(ultimoResultado.realizado),
                         (unsigned long)ultimoResultado.tiempo,
                         (unsigned)ultimoResultado.correcto,
                         (unsigned)ultimoResultado.muestras,
                         (long)ultimoResultado.ud, (long)ultimoResultado.lr,
                         (unsigned)ultimoResultado.saturadas);
        if (n < 0 || n >= (int)sizeof(fila)) r = FR_INVALID_PARAMETER;
        else { intentoEscritura = 1; r = SD_Escribir(fila); }
    }
    if (r != FR_OK && intentoEscritura) SD_RetirarParcial(tamAnterior);
    FRESULT cierre = f_close(&archivoSD);
    if (r == FR_OK) r = cierre;
    if (r != FR_OK)
    {
        if (cierre != FR_OK)
            Imprimir("Fallo al cerrar: el contenido escrito queda sin confirmar.\r\n");
        SD_MostrarError(r);
        return r;
    }

    ultimoResultado.guardado = 1;
    Pie("DATOS GUARDADOS");
    Imprimir("SD OK: DATOS GUARDADOS en GESTOS.CSV; cierre correcto.\r\n");
    return FR_OK;
}

static void SD_Verificar(void)
{
    Imprimir("\r\n--- PRUEBA SD: AGREGA Y RELEE UNA LINEA EN N2TEST.LOG ---\r\n");
    FRESULT r = SD_Montar();
    if (r != FR_OK) { SD_MostrarError(r); return; }

    char linea[64], leido[64];
    int n = snprintf(linea, sizeof(linea), "NUCLEO2,PRUEBA_SD,%lu\r\n",
                     (unsigned long)HAL_GetTick());
    if (n <= 0 || n >= (int)sizeof(linea))
    { SD_MostrarError(FR_INVALID_PARAMETER); return; }

    r = f_open(&archivoSD, "0:/N2TEST.LOG", FA_OPEN_ALWAYS | FA_WRITE);
    if (r != FR_OK) { SD_MostrarError(r); return; }
    DWORD posicion = (DWORD)f_size(&archivoSD);
    uint8_t intento = 0;
    r = f_lseek(&archivoSD, posicion);
    if (r == FR_OK) { intento = 1; r = SD_Escribir(linea); }
    if (r != FR_OK && intento) SD_RetirarParcial(posicion);
    FRESULT cierre = f_close(&archivoSD);
    if (r == FR_OK) r = cierre;
    if (r != FR_OK) { SD_MostrarError(r); return; }

    /* Remontar invalida la ventana de cache del sistema antes de releer. */
    r = SD_Montar();
    if (r != FR_OK) { SD_MostrarError(r); return; }
    r = f_open(&archivoSD, "0:/N2TEST.LOG", FA_READ);
    if (r != FR_OK) { SD_MostrarError(r); return; }
    r = f_lseek(&archivoSD, posicion);
    UINT cantidad = 0;
    if (r == FR_OK) r = f_read(&archivoSD, leido, (UINT)n, &cantidad);
    cierre = f_close(&archivoSD);
    if (r == FR_OK) r = cierre;
    if (r == FR_OK && (cantidad != (UINT)n || memcmp(linea, leido, (size_t)n)))
        r = FR_DISK_ERR;
    if (r != FR_OK) { SD_MostrarError(r); return; }

    Pie("SD ESCRITURA Y LECTURA OK");
    Imprimir("SD VERIFICADA: linea escrita, archivo cerrado y lectura identica.\r\n");
    Imprimir("Archivo: 0:/N2TEST.LOG. Esta prueba no guarda una medicion.\r\n");
}

static void IniciarPrueba(void)
{
    ultimoResultado.valido = 0;
    ultimoResultado.guardado = 0;
    if (!DiagnosticarSensor())
    {
        PantallaError("SIN RESPUESTA I2C");
        N1_Enviar("@ERROR,SENSOR\n");
        return;
    }
    if (APDS_Init(&sensor, &hi2c1) != HAL_OK)
    {
        Imprimir("FALLO: no se pudo configurar APDS-9960.\r\n");
        PantallaError("NO SE PUDO CONFIGURAR");
        N1_Enviar("@ERROR,SENSOR\n");
        return;
    }

    direccionEsperada = SortearDireccion();
    PantallaEstimulo(direccionEsperada);
    inicioEstimulo = HAL_GetTick();

    if (APDS_Start(&sensor) != HAL_OK)
    {
        Imprimir("FALLO: no se pudo armar la deteccion de gestos.\r\n");
        PantallaError("NO SE PUDO ARMAR");
        N1_Enviar("@ERROR,SENSOR\n");
        return;
    }
    midiendo = 1;
    N1_Enviar("@START\n");
    Imprimir("SOLICITADO: %s\r\n", Gesture_Name(direccionEsperada));
    Imprimir("SENSOR ACTIVO: haz un gesto ahora; tienes 3 segundos.\r\n");
}

static void TareaSensor(void)
{
    if (!midiendo) return;
    ApdsResult r = APDS_Poll(&sensor);
    if (r == APDS_WAIT) return;

    uint32_t tiempo = HAL_GetTick() - inicioEstimulo;
    midiendo = 0;
    Imprimir("Muestras=%u UD=%ld LR=%ld\r\n", (unsigned)sensor.valid_count,
             (long)sensor.ud_delta, (long)sensor.lr_delta);
    Imprimir("PICO UD=%ld LR=%ld SAT=%u\r\n",
             (long)sensor.peak_ud, (long)sensor.peak_lr,
             (unsigned)sensor.saturated_count);

    ultimoResultado.valido = 0;
    if (r == APDS_DONE || r == APDS_TIMEOUT || r == APDS_UNCLEAR)
    {
        ultimoResultado.esperado = direccionEsperada;
        ultimoResultado.realizado = r == APDS_DONE ? sensor.gesture : G_NONE;
        ultimoResultado.tiempo = tiempo;
        ultimoResultado.correcto = r == APDS_DONE && sensor.gesture == direccionEsperada;
        ultimoResultado.muestras = sensor.valid_count;
        ultimoResultado.saturadas = sensor.saturated_count;
        ultimoResultado.ud = sensor.peak_ud;
        ultimoResultado.lr = sensor.peak_lr;
        ultimoResultado.valido = 1;
        ultimoResultado.guardado = 0;
    }
    if (ultimoResultado.valido)
        N1_Resultado();
    else
        N1_Enviar(r == APDS_OVERFLOW ? "@ERROR,FIFO_LLENO\n"
                                   : "@ERROR,LECTURA_I2C\n");

    if (r == APDS_DONE)
    {
        Imprimir("GESTO DETECTADO: %s\r\n", Gesture_Name(sensor.gesture));
        Imprimir("%s | Tiempo de respuesta: %lu ms\r\n",
                 sensor.gesture == direccionEsperada ? "CORRECTO" : "INCORRECTO",
                 (unsigned long)tiempo);
        PantallaResultado(direccionEsperada, sensor.gesture, tiempo);
    }
    else if (r == APDS_TIMEOUT || r == APDS_UNCLEAR)
    {
        Imprimir(r == APDS_TIMEOUT
                 ? "SIN GESTO: faltan muestras utiles al terminar el tiempo.\r\n"
                 : "MOVIMIENTO NO RECONOCIDO: direccion ambigua.\r\n");
        PantallaResultado(direccionEsperada, G_NONE, tiempo);
    }
    else
    {
        Imprimir("FALLO: %s\r\n", r == APDS_OVERFLOW ? "FIFO LLENO" : "ERROR I2C");
        PantallaError(r == APDS_OVERFLOW ? "FIFO LLENO" : "ERROR I2C");
    }
    Imprimir("Medicion terminada. Esperando orden de Nucleo 1.\r\n");
}

static void RecibirHercules(void)
{
    uint8_t dato;
    HAL_StatusTypeDef r = HAL_UART_Receive(&huart2, &dato, 1, 1);
    if (r == HAL_TIMEOUT) return;
    if (r != HAL_OK) { __HAL_UART_CLEAR_OREFLAG(&huart2); return; }
    if (dato == '\r' || dato == '\n' || dato == ' ' || dato == '\t') return;
    if (midiendo) { Imprimir("Medicion en curso; espera el resultado.\r\n"); return; }
    switch (dato) {
    case 'C': case 'c': DiagnosticarEnlace(); break;
    case 'H': case 'h': N1_Enviar("@READY\n"); break;
    case 'T': case 't': SD_Probar(); break;
    case 'V': case 'v': SD_Verificar(); break;
    case 'D': case 'd': (void)DiagnosticarSensor(); break;
    case 'S': case 's': EscanearBus(); break;
    default:
        Imprimir("C=enlace H=READY T=SD sin escribir V=prueba escritura SD D=sensor S=I2C.\r\n");
        Imprimir("L y G se solicitan desde Nucleo 1, no desde Hercules local.\r\n");
        break;
    }
}

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  Imprimir("\r\nN2: INICIO DEL PROGRAMA\r\n");

  if (HAL_UART_Receive_IT(&huart3, &n1Byte, 1) != HAL_OK)
  {
      Imprimir("ERROR INICIANDO UART3\r\n");
      Error_Handler();
  }

  Imprimir("N2: UART3 LISTO\r\n");
  N1_Enviar("@READY\n");

  Imprimir("N2: INICIANDO PANTALLA\r\n");
  LCD_Init();

  if (!LCD_IsReady())
      Imprimir("ERROR SPI PANTALLA\r\n");

  PantallaInicio();

  Imprimir("N2: PANTALLA TERMINADA\r\n");
  DiagnosticarEnlace();
  (void)DiagnosticarSensor();

  Imprimir("N2: SISTEMA LISTO\r\n");
  Imprimir("T=prueba SD, V=verificar SD, D=sensor, S=I2C\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        N1_Tarea();
        TareaSensor();
        RecibirHercules();
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
  RCC_OscInitStruct.PLL.PLLN = 320;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

    __HAL_RCC_I2C1_CLK_ENABLE();

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

    __HAL_RCC_SPI1_CLK_ENABLE();

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

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

    __HAL_RCC_USART2_CLK_ENABLE();

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

    __HAL_RCC_USART3_CLK_ENABLE();

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SDCS_Pin|CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DC_Pin */
  GPIO_InitStruct.Pin = DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDCS_Pin CS_Pin */
  GPIO_InitStruct.Pin = SDCS_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* Configurar PB0 como Chip Select de la microSD */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Mantener la microSD deseleccionada al iniciar */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart3) return;
    n1BytesRX++;
    uint8_t siguiente = (uint8_t)((n1Entrada + 1u) % N1_RX_TAM);
    if (siguiente == n1Salida)
        n1Error = 1;
    else {
        n1Cola[n1Entrada] = n1Byte;
        n1Entrada = siguiente;
    }
    if (HAL_UART_Receive_IT(&huart3, &n1Byte, 1) != HAL_OK)
        n1Error = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) n1Error = 1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

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

    (void)file;
    (void)line;

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
