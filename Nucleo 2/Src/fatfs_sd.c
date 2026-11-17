#include "main.h"
#include "ff.h"
#include "diskio.h"
#include "fatfs_sd.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

/* Diagnostico por Hercules. La API del controlador no cambia. */
static void sd_log(const char *formato, ...)
{
    char texto[192];
    va_list args;
    va_start(args, formato);
    int n = vsnprintf(texto, sizeof(texto), formato, args);
    va_end(args);
    if (n <= 0) return;
    if (n >= (int)sizeof(texto)) n = (int)sizeof(texto) - 1;
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)texto, (uint16_t)n, 100);
}

static HAL_StatusTypeDef ultimoHAL;
static uint32_t ultimoErrorSPI;
static uint8_t comandoEnviado;

static DSTATUS estado = STA_NOINIT;
static uint8_t bloques, fallo;
static uint32_t prescalerAnterior;
/* Prueba conservadora: prescaler /128 en SPI1 para acceso a sectores. */
#define SD_SPI_OPERACION SPI_BAUDRATEPRESCALER_128

static uint8_t transferir(uint8_t valor)
{
    uint8_t recibido = 0xFFu;
    HAL_StatusTypeDef r = HAL_SPI_TransmitReceive(&hspi1, &valor, &recibido, 1, 50);
    if (r != HAL_OK)
    {
        fallo = 1u;
        ultimoHAL = r;
        ultimoErrorSPI = hspi1.ErrorCode;
    }
    return recibido;
}

static void deseleccionar(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
    /* Reloj adicional con CS alto: libera la tarjeta. */
    (void)transferir(0xFFu);
}

static int busIniciar(uint32_t prescaler)
{
    prescalerAnterior = hspi1.Init.BaudRatePrescaler;
    fallo = 0u;
    ultimoHAL = HAL_OK;
    ultimoErrorSPI = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); /* CS TFT desactivado */
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
    hspi1.Init.BaudRatePrescaler = prescaler;
    HAL_StatusTypeDef r = HAL_SPI_Init(&hspi1);
    if (r != HAL_OK)
    {
        fallo = 1u;
        ultimoHAL = r;
        ultimoErrorSPI = hspi1.ErrorCode;
    }
    return !fallo;
}

static int busTerminar(void)
{
    deseleccionar();
    hspi1.Init.BaudRatePrescaler = prescalerAnterior;
    HAL_StatusTypeDef r = HAL_SPI_Init(&hspi1);
    if (r != HAL_OK)
    {
        fallo = 1u;
        ultimoHAL = r;
        ultimoErrorSPI = hspi1.ErrorCode;
    }
    if (fallo) estado = STA_NOINIT;
    return !fallo;
}

static int esperarListo(uint32_t limite)
{
    uint32_t inicio = HAL_GetTick();
    do {
        if (transferir(0xFFu) == 0xFFu && !fallo) return 1;
    } while (!fallo && (uint32_t)(HAL_GetTick() - inicio) < limite);
    return 0;
}

/* CRC16-CCITT SD para bloque de 512 bytes; polinomio 0x1021, inicio 0. */
static uint16_t SD_CRC16(const uint8_t *datos, uint16_t longitud)
{
    uint16_t crc = 0u;
    for (uint16_t i = 0; i < longitud; ++i) {
        crc ^= (uint16_t)datos[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* CRC7 SD: valido para cualquier comando, no solo CMD0/CMD8. */
static uint8_t SD_CRC7(const uint8_t *datos, uint8_t longitud)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < longitud; ++i) {
        uint8_t dato = datos[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc <<= 1;
            if ((dato ^ crc) & 0x80u) crc ^= 0x09u;
            dato <<= 1;
        }
    }
    return (uint8_t)((crc << 1) | 1u);
}

/* MantenerCS=1 mantiene seleccionada la tarjeta tras CMD55. */
static uint8_t SD_Comando(uint8_t numero, uint32_t argumento, uint8_t mantenerCS)
{
    comandoEnviado = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); /* TFT no seleccionada */
    if (!mantenerCS) {
        deseleccionar();
        HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
        (void)transferir(0xFFu);
    }
    if (!esperarListo(500u)) {
        sd_log("SDDBG CMD%u: MISO ocupado, CS=%u MISO=%u\r\n",
               (unsigned)numero,
               (unsigned)HAL_GPIO_ReadPin(SD_CS_PORT, SD_CS_PIN),
               (unsigned)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
        return 0xFFu;
    }

    uint8_t paquete[5] = {
        (uint8_t)(0x40u | numero),
        (uint8_t)(argumento >> 24),
        (uint8_t)(argumento >> 16),
        (uint8_t)(argumento >> 8),
        (uint8_t)argumento
    };
    for (unsigned i = 0; i < 5u; ++i) (void)transferir(paquete[i]);
    (void)transferir(SD_CRC7(paquete, 5u));
    comandoEnviado = !fallo;

    for (unsigned i = 0; i < 10u && !fallo; ++i) {
        uint8_t respuesta = transferir(0xFFu);
        if (!(respuesta & 0x80u)) return respuesta;
    }
    return 0xFFu;
}

static uint8_t comando(uint8_t numero, uint32_t argumento)
{
    return SD_Comando(numero, argumento, 0u);
}

static int recibirBloque(uint8_t *datos, uint16_t cantidad)
{
    uint32_t inicio = HAL_GetTick();
    uint8_t token = 0xFFu;
    do {
        token = transferir(0xFFu);
        if (token != 0xFFu || fallo) break;
    } while ((uint32_t)(HAL_GetTick() - inicio) < 1000u);

    if (token != 0xFEu || fallo) {
        sd_log("SDDBG READ TOKEN=0x%02X espera=%lu ms HAL=%u ERR=0x%08lX CS=%u\r\n",
               (unsigned)token,
               (unsigned long)(HAL_GetTick() - inicio),
               (unsigned)ultimoHAL,
               (unsigned long)ultimoErrorSPI,
               (unsigned)HAL_GPIO_ReadPin(SD_CS_PORT, SD_CS_PIN));
        return 0;
    }

    for (uint16_t i = 0; i < cantidad && !fallo; ++i)
        datos[i] = transferir(0xFFu);
    (void)transferir(0xFFu);
    (void)transferir(0xFFu);
    return !fallo;
}

void SD_Reset(void)
{
    estado = STA_NOINIT;
    bloques = 0u;
}

DSTATUS SD_disk_initialize(BYTE pdrv)
{
    sd_log("\r\nSDDBG INICIO pdrv=%u\r\n", (unsigned)pdrv);
    if (pdrv) return STA_NOINIT;
    SD_Reset();
    if (!busIniciar(SPI_BAUDRATEPRESCALER_256))
    {
        (void)busTerminar();
        sd_log("SDDBG FALLO HAL_SPI_Init HAL=%u ERR=0x%08lX\r\n",
               (unsigned)ultimoHAL, (unsigned long)ultimoErrorSPI);
        return estado;
    }

    sd_log("SDDBG SPI1 inicio=%lu Hz | CS_SD(PB0)=%u | MISO(PA6)=%u\r\n",
           (unsigned long)(HAL_RCC_GetPCLK2Freq() / 256u),
           (unsigned)HAL_GPIO_ReadPin(SD_CS_PORT, SD_CS_PIN),
           (unsigned)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
    HAL_Delay(10);
    for (unsigned i = 0; i < 10u; i++) (void)transferir(0xFFu);

    uint8_t r;
    uint32_t inicio = HAL_GetTick();
    do {
        r = comando(0, 0);
    } while (r != 1u && !fallo && (uint32_t)(HAL_GetTick() - inicio) < 1000u);
    sd_log("SDDBG CMD0=0x%02X ENVIADO=%u HAL=%u ERR=0x%08lX\r\n",
           (unsigned)r, (unsigned)comandoEnviado,
           (unsigned)ultimoHAL, (unsigned long)ultimoErrorSPI);

    if (r == 1u && !fallo)
    {
        r = comando(8, 0x1AAu);
        sd_log(
            "SDDBG CMD8=0x%02X ENVIADO=%u HAL=%u ERR=0x%08lX\r\n",
            (unsigned)r,
            (unsigned)comandoEnviado,
            (unsigned)ultimoHAL,
            (unsigned long)ultimoErrorSPI
        );
        /* CMD8=0 es anomalo tras CMD0. Solo se tolera para diagnostico
           si el patron de retorno R7 coincide. */
        uint8_t v2 = (r == 1u || r == 0u);
        uint8_t compatible = v2 || r == 5u;
        if (v2)
        {
            uint8_t eco[4];
            for (unsigned i = 0; i < 4u; i++) eco[i] = transferir(0xFFu);
            sd_log("SDDBG R7=%02X %02X %02X %02X\r\n",
                   (unsigned)eco[0], (unsigned)eco[1],
                   (unsigned)eco[2], (unsigned)eco[3]);
            compatible = eco[2] == 1u && eco[3] == 0xAAu;
            if (eco[0] != 0u || eco[1] != 0u || r != 1u)
                sd_log("SDDBG AVISO: CMD8/R7 irregular (no valida SD aun)\r\n");
        }
        if (compatible && !fallo)
        {
            uint8_t r55 = 0xFFu;
            uint8_t r41 = 0xFFu;
            inicio = HAL_GetTick();
            do {
                r55 = comando(55, 0);
                r = r55;
                if (r55 <= 1u)
                {
                    /* ACMD41 debe seguir a CMD55 sin deseleccionar SD. */
                    r41 = SD_Comando(41, v2 ? 0x40000000u : 0u, 1u);
                    r = r41;
                }
                if (r == 0u) break;
            } while (!fallo && (uint32_t)(HAL_GetTick() - inicio) < 2000u);
            sd_log("SDDBG CMD55=0x%02X ACMD41=0x%02X\r\n",
                   (unsigned)r55, (unsigned)r41);
            if (r == 0u && !fallo)
            {
                uint8_t correcto = 1u;
                if (v2)
                {
                    r = comando(58, 0);
                    sd_log("SDDBG CMD58=0x%02X\r\n", (unsigned)r);
                    correcto = r == 0u;
                    if (correcto)
                    {
                        uint8_t ocr[4];
                        for (unsigned i = 0; i < 4u; i++) ocr[i] = transferir(0xFFu);
                        sd_log("SDDBG OCR=%02X %02X %02X %02X\r\n",
                               (unsigned)ocr[0], (unsigned)ocr[1],
                               (unsigned)ocr[2], (unsigned)ocr[3]);
                        bloques = (ocr[0] & 0x40u) != 0u;
                        correcto = (ocr[0] & 0x80u) != 0u;
                    }
                }
                if (correcto && !bloques)
                {
                    r = comando(16, 512);
                    sd_log("SDDBG CMD16=0x%02X\r\n", (unsigned)r);
                    correcto = r == 0u;
                }
                if (correcto && !fallo) estado = 0u;
            }
        }
    }
    (void)busTerminar();
    sd_log("SDDBG FIN estado=0x%02X BLOQUES=%u HAL=%u ERR=0x%08lX\r\n",
           (unsigned)estado, (unsigned)bloques,
           (unsigned)ultimoHAL, (unsigned long)ultimoErrorSPI);
    return estado;
}

DSTATUS SD_disk_status(BYTE pdrv)
{
    return pdrv ? STA_NOINIT : estado;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !buff || !count) return RES_PARERR;
    if (estado & STA_NOINIT) return RES_NOTRDY;
    DWORD ultimo = bloques ? 0xFFFFFFFFu : 0x7FFFFFu;
    if (sector > ultimo || (DWORD)(count - 1u) > ultimo - sector)
        return RES_PARERR;
    if (!busIniciar(SD_SPI_OPERACION)) {
        (void)busTerminar();
        return RES_ERROR;
    }
    DRESULT resultado = RES_OK;
    while (count--) {
        uint32_t direccion = bloques ? sector : sector * 512u;
        uint8_t r17 = comando(17, direccion);
        sd_log("SDDBG READ sector=%lu CMD17=0x%02X\r\n",
               (unsigned long)sector, (unsigned)r17);
        if (r17 != 0u || !recibirBloque(buff, 512)) {
            resultado = RES_ERROR;
            break;
        }
        buff += 512;
        sector++;
    }
    if (!busTerminar()) resultado = RES_ERROR;
    return resultado;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !buff || !count) return RES_PARERR;
    if (estado & STA_NOINIT) return RES_NOTRDY;
    DWORD ultimo = bloques ? 0xFFFFFFFFu : 0x7FFFFFu;
    if (sector > ultimo || (DWORD)(count - 1u) > ultimo - sector)
        return RES_PARERR;
    if (!busIniciar(SD_SPI_OPERACION)) {
        (void)busTerminar();
        return RES_ERROR;
    }
    DRESULT resultado = RES_OK;
    while (count--) {
        uint32_t direccion = bloques ? sector : sector * 512u;
        uint8_t r24 = comando(24, direccion);
        sd_log("SDDBG WRITE sector=%lu CMD24=0x%02X\r\n",
               (unsigned long)sector, (unsigned)r24);
        if (r24 != 0u || !esperarListo(500)) {
            resultado = RES_ERROR;
            break;
        }
        (void)transferir(0xFFu); /* byte previo al token */
        (void)transferir(0xFEu);
        HAL_StatusTypeDef tx = HAL_SPI_Transmit(&hspi1, (uint8_t *)buff, 512, 1500);
        if (tx != HAL_OK) {
            fallo = 1u;
            ultimoHAL = tx;
            ultimoErrorSPI = hspi1.ErrorCode;
        }
        /* Algunas tarjetas verifican CRC16 incluso en modo SPI. */
        uint16_t crc16 = SD_CRC16(buff, 512u);
        (void)transferir((uint8_t)(crc16 >> 8));
        (void)transferir((uint8_t)crc16);
        uint8_t respuesta = 0xFFu;
        for (unsigned i = 0; i < 32u && respuesta == 0xFFu && !fallo; i++)
            respuesta = transferir(0xFFu);
        sd_log("SDDBG WRITE RESP=0x%02X HAL=%u ERR=0x%08lX\r\n",
               (unsigned)respuesta, (unsigned)ultimoHAL,
               (unsigned long)ultimoErrorSPI);
        if (fallo || (respuesta & 0x1Fu) != 0x05u || !esperarListo(1500)) {
            resultado = RES_ERROR;
            break;
        }
        buff += 512;
        sector++;
    }
    if (!busTerminar()) resultado = RES_ERROR;
    return resultado;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv) return RES_PARERR;
    if (estado & STA_NOINIT) return RES_NOTRDY;
    if (cmd == GET_SECTOR_SIZE && buff) { *(WORD *)buff = 512u; return RES_OK; }
    if (cmd == GET_BLOCK_SIZE && buff) { *(DWORD *)buff = 1u; return RES_OK; }
    if (cmd != CTRL_SYNC && (cmd != GET_SECTOR_COUNT || !buff)) return RES_PARERR;
    if (!busIniciar(SD_SPI_OPERACION)) {
        (void)busTerminar();
        return RES_ERROR;
    }
    DRESULT resultado = RES_ERROR;
    if (cmd == CTRL_SYNC) {
        HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
        if (esperarListo(1500)) resultado = RES_OK;
    } else {
        uint8_t csd[16];
        if (comando(9, 0) == 0u && recibirBloque(csd, 16)) {
            if ((csd[0] >> 6) == 1u) {
                uint32_t tam = ((uint32_t)(csd[7] & 0x3Fu) << 16)
                             | ((uint32_t)csd[8] << 8) | csd[9];
                if (tam < 0x3FFFFFu) { *(DWORD *)buff = (tam + 1u) << 10; resultado = RES_OK; }
            } else if ((csd[0] >> 6) == 0u) {
                uint32_t tam = ((uint32_t)(csd[6] & 3u) << 10)
                             | ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
                int n = (csd[5] & 15) + ((csd[10] & 0x80u) >> 7)
                      + ((csd[9] & 3u) << 1) + 2 - 9;
                if (n >= 0 && n < 21) { *(DWORD *)buff = (tam + 1u) << n; resultado = RES_OK; }
            }
        }
    }
    if (!busTerminar()) resultado = RES_ERROR;
    return resultado;
}
