#include <Arduino.h>
#include <SPI.h>
#include <LittleFS.h>
#include "esp_camera.h"

// =====================================================
// AI-Thinker ESP32-CAM: камера OV2640
// =====================================================

#define CAM_PIN_PWDN     32
#define CAM_PIN_RESET    -1
#define CAM_PIN_XCLK      0
#define CAM_PIN_SIOD     26
#define CAM_PIN_SIOC     27

#define CAM_PIN_Y9       35
#define CAM_PIN_Y8       34
#define CAM_PIN_Y7       39
#define CAM_PIN_Y6       36
#define CAM_PIN_Y5       21
#define CAM_PIN_Y4       19
#define CAM_PIN_Y3       18
#define CAM_PIN_Y2        5

#define CAM_PIN_VSYNC    25
#define CAM_PIN_HREF     23
#define CAM_PIN_PCLK     22

// =====================================================
// CC1101: подключение
// =====================================================

constexpr uint8_t CC1101_PIN_SCK  = 14;
constexpr uint8_t CC1101_PIN_MISO = 12;
constexpr uint8_t CC1101_PIN_MOSI = 13;
constexpr uint8_t CC1101_PIN_CSN  = 15;

// GDO0 и GDO2 в этом варианте не используются.

constexpr uint32_t CC1101_SPI_FREQUENCY = 2000000;

SPIClass cc1101Spi(HSPI);

SPISettings cc1101SpiSettings(
    CC1101_SPI_FREQUENCY,
    MSBFIRST,
    SPI_MODE0
);

// =====================================================
// Файлы
// =====================================================

const char *PHOTO_PATH = "/photo.jpg";
const char *TEMP_PATH  = "/photo.tmp";

// =====================================================
// Настройки камеры
// =====================================================

constexpr framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_QVGA;
constexpr uint16_t CAMERA_WIDTH  = 320;
constexpr uint16_t CAMERA_HEIGHT = 240;
constexpr uint8_t CAMERA_JPEG_QUALITY = 18;

// =====================================================
// Команды CC1101
// =====================================================

constexpr uint8_t CC1101_SRES  = 0x30;
constexpr uint8_t CC1101_SFSTXON = 0x31;
constexpr uint8_t CC1101_SXOFF = 0x32;
constexpr uint8_t CC1101_SCAL  = 0x33;
constexpr uint8_t CC1101_SRX   = 0x34;
constexpr uint8_t CC1101_STX   = 0x35;
constexpr uint8_t CC1101_SIDLE = 0x36;
constexpr uint8_t CC1101_SWOR  = 0x38;
constexpr uint8_t CC1101_SPWD  = 0x39;
constexpr uint8_t CC1101_SFRX  = 0x3A;
constexpr uint8_t CC1101_SFTX  = 0x3B;
constexpr uint8_t CC1101_SWORRST = 0x3C;
constexpr uint8_t CC1101_SNOP  = 0x3D;

// =====================================================
// Адреса FIFO, PATABLE и status-регистров
// =====================================================

constexpr uint8_t CC1101_PATABLE = 0x3E;
constexpr uint8_t CC1101_TXFIFO  = 0x3F;

constexpr uint8_t CC1101_PARTNUM   = 0x30;
constexpr uint8_t CC1101_VERSION   = 0x31;
constexpr uint8_t CC1101_MARCSTATE = 0x35;
constexpr uint8_t CC1101_PKTSTATUS = 0x38;
constexpr uint8_t CC1101_TXBYTES   = 0x3A;

// =====================================================
// Биты SPI-заголовка
// =====================================================

constexpr uint8_t CC1101_WRITE_BURST = 0x40;
constexpr uint8_t CC1101_READ_SINGLE = 0x80;
constexpr uint8_t CC1101_READ_BURST  = 0xC0;

// =====================================================
// Состояния MARCSTATE
// =====================================================

constexpr uint8_t CC1101_STATE_IDLE = 0x01;
constexpr uint8_t CC1101_STATE_RX = 0x0D;
constexpr uint8_t CC1101_STATE_RXFIFO_OVERFLOW = 0x11;
constexpr uint8_t CC1101_STATE_FSTXON = 0x12;
constexpr uint8_t CC1101_STATE_TX = 0x13;
constexpr uint8_t CC1101_STATE_TX_END = 0x14;
constexpr uint8_t CC1101_STATE_TXFIFO_UNDERFLOW = 0x16;

// =====================================================
// Мощность CC1101
// =====================================================

/*
 * Пользовательские регистры не содержат PATABLE.
 *
 * При FREND0 = 0x10 используется PATABLE[0].
 * Для 433 МГц 0xC0 соответствует примерно +10 dBm.
 *
 * При необходимости замените значением из SmartRF Studio.
 */
constexpr uint8_t CC1101_PATABLE0 = 0xC0;

// =====================================================
// Пользовательские регистры CC1101
// =====================================================

struct RegisterSetting {
    uint8_t address;
    uint8_t value;
    const char *name;

    /*
     * FSCAL-регистры могут измениться после автоматической
     * калибровки синтезатора.
     */
    bool runtimeMutable;
};

const RegisterSetting CC1101_SETTINGS[] = {
    {0x00, 0x29, "IOCFG2",   false},
    {0x01, 0x2E, "IOCFG1",   false},
    {0x02, 0x06, "IOCFG0",   false},
    {0x03, 0x07, "FIFOTHR",  false},
    {0x04, 0xD3, "SYNC1",    false},
    {0x05, 0x91, "SYNC0",    false},
    {0x06, 0xFF, "PKTLEN",   false},
    {0x07, 0x04, "PKTCTRL1", false},
    {0x08, 0x01, "PKTCTRL0", false},
    {0x09, 0x00, "ADDR",     false},
    {0x0A, 0x00, "CHANNR",   false},
    {0x0B, 0x06, "FSCTRL1",  false},
    {0x0C, 0x00, "FSCTRL0",  false},
    {0x0D, 0x10, "FREQ2",    false},
    {0x0E, 0xAA, "FREQ1",    false},
    {0x0F, 0x56, "FREQ0",    false},
    {0x10, 0xFB, "MDMCFG4",  false},
    {0x11, 0xF8, "MDMCFG3",  false},
    {0x12, 0x03, "MDMCFG2",  false},
    {0x13, 0x22, "MDMCFG1",  false},
    {0x14, 0xF8, "MDMCFG0",  false},
    {0x15, 0x77, "DEVIATN",  false},
    {0x16, 0x07, "MCSM2",    false},
    {0x17, 0x30, "MCSM1",    false},
    {0x18, 0x18, "MCSM0",    false},
    {0x19, 0x16, "FOCCFG",   false},
    {0x1A, 0x6C, "BSCFG",    false},
    {0x1B, 0x03, "AGCCTRL2", false},
    {0x1C, 0x40, "AGCCTRL1", false},
    {0x1D, 0x91, "AGCCTRL0", false},
    {0x1E, 0x87, "WOREVT1",  false},
    {0x1F, 0x6B, "WOREVT0",  false},
    {0x20, 0xF8, "WORCTRL",  false},
    {0x21, 0x56, "FREND1",   false},
    {0x22, 0x10, "FREND0",   false},
    {0x23, 0xEA, "FSCAL3",   true},
    {0x24, 0x2A, "FSCAL2",   true},
    {0x25, 0x00, "FSCAL1",   true},
    {0x26, 0x1F, "FSCAL0",   true},
    {0x27, 0x41, "RCCTRL1",  false},
    {0x28, 0x00, "RCCTRL0",  false},
    {0x29, 0x59, "FSTEST",   false},
    {0x2A, 0x7F, "PTEST",    false},
    {0x2B, 0x3F, "AGCTEST",  false},
    {0x2C, 0x81, "TEST2",    false},
    {0x2D, 0x35, "TEST1",    false},
    {0x2E, 0x09, "TEST0",    false}
};

constexpr size_t CC1101_SETTINGS_COUNT =
    sizeof(CC1101_SETTINGS) /
    sizeof(CC1101_SETTINGS[0]);

// =====================================================
// Радиопротокол изображения
// =====================================================

/*
 * CC1101 имеет FIFO 64 байта.
 *
 * В RX FIFO будут находиться:
 *   1 байт длины
 *   до 60 байт пакета
 *   2 байта статуса, поскольку PKTCTRL1 = 0x04
 *
 * Всего максимум 63 байта.
 */
constexpr uint8_t RADIO_PACKET_MAX_SIZE = 60;

constexpr uint8_t PROTOCOL_MAGIC_0 = 'I';
constexpr uint8_t PROTOCOL_MAGIC_1 = 'M';
constexpr uint8_t PROTOCOL_VERSION = 1;

enum ImagePacketType : uint8_t {
    PACKET_START = 1,
    PACKET_DATA  = 2,
    PACKET_END   = 3
};

/*
 * Заголовок:
 *
 * 0      'I'
 * 1      'M'
 * 2      версия
 * 3      тип
 * 4..5   ID изображения
 * 6..7   номер DATA-пакета
 * 8..9   общее количество DATA-пакетов
 * 10     размер payload
 * 11     зарезервировано
 */
constexpr uint8_t PROTOCOL_HEADER_SIZE = 12;
constexpr uint8_t PROTOCOL_CRC_SIZE = 4;

constexpr uint8_t IMAGE_BYTES_PER_PACKET =
    RADIO_PACKET_MAX_SIZE -
    PROTOCOL_HEADER_SIZE -
    PROTOCOL_CRC_SIZE;

static_assert(
    IMAGE_BYTES_PER_PACKET == 44,
    "Некорректный размер payload"
);

constexpr uint8_t CC1101_SEND_ATTEMPTS = 3;
constexpr uint32_t CC1101_TX_TIMEOUT_MS = 3000;
constexpr uint16_t INTER_PACKET_DELAY_MS = 3;

uint16_t nextImageId = 1;

// =====================================================
// Информация о фотографии
// =====================================================

struct PhotoInfo {
    uint32_t size;
    uint16_t width;
    uint16_t height;
};

// =====================================================
// Little-endian
// =====================================================

void writeUint16LE(uint8_t *destination, uint16_t value)
{
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8);
}

void writeUint32LE(uint8_t *destination, uint32_t value)
{
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8);
    destination[2] = static_cast<uint8_t>(value >> 16);
    destination[3] = static_cast<uint8_t>(value >> 24);
}

// =====================================================
// CRC32 — совместим с zlib.crc32()
// =====================================================

uint32_t crc32Update(
    uint32_t crc,
    const uint8_t *data,
    size_t length
) {
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1UL) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

uint32_t calculateBufferCrc32(
    const uint8_t *data,
    size_t length
) {
    uint32_t crc = 0xFFFFFFFFUL;
    crc = crc32Update(crc, data, length);
    return crc ^ 0xFFFFFFFFUL;
}

// =====================================================
// Низкоуровневый SPI CC1101
// =====================================================

bool CC1101_wait_ready(uint32_t timeoutUs = 20000)
{
    uint32_t startedAt = micros();

    while (digitalRead(CC1101_PIN_MISO) == HIGH) {
        if (
            static_cast<uint32_t>(micros() - startedAt) >=
            timeoutUs
        ) {
            return false;
        }

        delayMicroseconds(1);
    }

    return true;
}

bool CC1101_begin_access()
{
    cc1101Spi.beginTransaction(cc1101SpiSettings);

    digitalWrite(CC1101_PIN_CSN, LOW);

    if (!CC1101_wait_ready()) {
        digitalWrite(CC1101_PIN_CSN, HIGH);
        cc1101Spi.endTransaction();
        return false;
    }

    return true;
}

void CC1101_end_access()
{
    digitalWrite(CC1101_PIN_CSN, HIGH);
    cc1101Spi.endTransaction();
}

bool CC1101_write_reg(uint16_t address, uint8_t value)
{
    uint8_t registerAddress =
        static_cast<uint8_t>(address) & 0x3F;

    if (!CC1101_begin_access()) {
        return false;
    }

    cc1101Spi.transfer(registerAddress);
    cc1101Spi.transfer(value);

    CC1101_end_access();
    return true;
}

bool CC1101_read_reg(uint8_t address, uint8_t &value)
{
    if (!CC1101_begin_access()) {
        return false;
    }

    cc1101Spi.transfer(
        (address & 0x3F) |
        CC1101_READ_SINGLE
    );

    value = cc1101Spi.transfer(0x00);

    CC1101_end_access();
    return true;
}

bool CC1101_read_status_once(
    uint8_t address,
    uint8_t &value
) {
    if (!CC1101_begin_access()) {
        return false;
    }

    /*
     * Для status-регистров 0x30...0x3D
     * одновременно устанавливаются READ и BURST.
     */
    cc1101Spi.transfer(
        (address & 0x3F) |
        CC1101_READ_BURST
    );

    value = cc1101Spi.transfer(0x00);

    CC1101_end_access();
    return true;
}

bool CC1101_read_status_stable(
    uint8_t address,
    uint8_t &value
) {
    uint8_t previous = 0;

    if (!CC1101_read_status_once(address, previous)) {
        return false;
    }

    for (uint8_t attempt = 0; attempt < 10; attempt++) {
        uint8_t current = 0;

        if (!CC1101_read_status_once(address, current)) {
            return false;
        }

        if (current == previous) {
            value = current;
            return true;
        }

        previous = current;
    }

    value = previous;
    return true;
}

bool CC1101_strobe(
    uint8_t command,
    uint8_t *statusByte = nullptr
) {
    if (!CC1101_begin_access()) {
        return false;
    }

    uint8_t status =
        cc1101Spi.transfer(command);

    CC1101_end_access();

    if (statusByte != nullptr) {
        *statusByte = status;
    }

    return true;
}

bool CC1101_write_patable0(uint8_t value)
{
    if (!CC1101_begin_access()) {
        return false;
    }

    cc1101Spi.transfer(CC1101_PATABLE);
    cc1101Spi.transfer(value);

    CC1101_end_access();
    return true;
}

bool CC1101_read_patable0(uint8_t &value)
{
    if (!CC1101_begin_access()) {
        return false;
    }

    cc1101Spi.transfer(
        CC1101_PATABLE |
        CC1101_READ_SINGLE
    );

    value = cc1101Spi.transfer(0x00);

    CC1101_end_access();
    return true;
}

// =====================================================
// Сброс CC1101
// =====================================================

bool CC1101_reset()
{
    Serial.println("[CC1101] Ручной сброс...");

    /*
     * Начальная часть ручной последовательности:
     * SCLK=1, SI=0, переключение CSN.
     */
    pinMode(CC1101_PIN_SCK, OUTPUT);
    pinMode(CC1101_PIN_MOSI, OUTPUT);
    pinMode(CC1101_PIN_CSN, OUTPUT);
    pinMode(CC1101_PIN_MISO, INPUT);

    digitalWrite(CC1101_PIN_SCK, HIGH);
    digitalWrite(CC1101_PIN_MOSI, LOW);
    digitalWrite(CC1101_PIN_CSN, HIGH);

    delayMicroseconds(5);

    digitalWrite(CC1101_PIN_CSN, LOW);
    delayMicroseconds(10);

    digitalWrite(CC1101_PIN_CSN, HIGH);
    delayMicroseconds(50);

    /*
     * Запускаем аппаратный HSPI.
     */
    cc1101Spi.begin(
        CC1101_PIN_SCK,
        CC1101_PIN_MISO,
        CC1101_PIN_MOSI,
        CC1101_PIN_CSN
    );

    cc1101Spi.beginTransaction(cc1101SpiSettings);

    digitalWrite(CC1101_PIN_CSN, LOW);

    if (!CC1101_wait_ready()) {
        digitalWrite(CC1101_PIN_CSN, HIGH);
        cc1101Spi.endTransaction();

        Serial.println(
            "[CC1101] CHIP_RDYn не перешёл в LOW перед SRES"
        );

        return false;
    }

    cc1101Spi.transfer(CC1101_SRES);

    if (!CC1101_wait_ready()) {
        digitalWrite(CC1101_PIN_CSN, HIGH);
        cc1101Spi.endTransaction();

        Serial.println(
            "[CC1101] CHIP_RDYn не перешёл в LOW после SRES"
        );

        return false;
    }

    digitalWrite(CC1101_PIN_CSN, HIGH);
    cc1101Spi.endTransaction();

    delay(1);

    Serial.println("[CC1101] Сброс завершён");
    return true;
}

// =====================================================
// Состояние CC1101
// =====================================================

bool CC1101_get_marcstate(uint8_t &state)
{
    if (!CC1101_read_status_stable(
            CC1101_MARCSTATE,
            state)) {
        return false;
    }

    state &= 0x1F;
    return true;
}

bool CC1101_get_txbytes(uint8_t &txBytes)
{
    return CC1101_read_status_stable(
        CC1101_TXBYTES,
        txBytes
    );
}

bool CC1101_enter_idle(uint32_t timeoutMs = 500)
{
    if (!CC1101_strobe(CC1101_SIDLE)) {
        return false;
    }

    uint32_t startedAt = millis();

    while (
        static_cast<uint32_t>(millis() - startedAt) <
        timeoutMs
    ) {
        uint8_t state = 0;

        if (!CC1101_get_marcstate(state)) {
            delay(1);
            continue;
        }

        if (state == CC1101_STATE_IDLE) {
            return true;
        }

        if (state == CC1101_STATE_TXFIFO_UNDERFLOW) {
            CC1101_strobe(CC1101_SFTX);
        }

        delay(1);
    }

    return false;
}

bool CC1101_flush_tx()
{
    if (!CC1101_enter_idle()) {
        return false;
    }

    if (!CC1101_strobe(CC1101_SFTX)) {
        return false;
    }

    delayMicroseconds(100);
    return true;
}

// =====================================================
// Запись радиопакета в TX FIFO
// =====================================================

bool CC1101_write_tx_packet(
    const uint8_t *data,
    uint8_t length
) {
    if (
        data == nullptr ||
        length == 0 ||
        length > RADIO_PACKET_MAX_SIZE
    ) {
        return false;
    }

    if (!CC1101_begin_access()) {
        return false;
    }

    cc1101Spi.transfer(
        CC1101_TXFIFO |
        CC1101_WRITE_BURST
    );

    /*
     * PKTCTRL0 = 0x01:
     * переменная длина пакета.
     *
     * Первый байт FIFO — длина payload.
     */
    cc1101Spi.transfer(length);

    for (uint8_t i = 0; i < length; i++) {
        cc1101Spi.transfer(data[i]);
    }

    CC1101_end_access();
    return true;
}

// =====================================================
// Передача одного физического радиопакета
// =====================================================

bool CC1101_send_packet(
    const uint8_t *data,
    uint8_t length
) {
    if (
        data == nullptr ||
        length == 0 ||
        length > RADIO_PACKET_MAX_SIZE
    ) {
        Serial.printf(
            "[CC1101] Неверный размер пакета: %u\n",
            length
        );

        return false;
    }

    for (
        uint8_t attempt = 1;
        attempt <= CC1101_SEND_ATTEMPTS;
        attempt++
    ) {
        if (!CC1101_flush_tx()) {
            Serial.printf(
                "[CC1101] Не удалось очистить TX FIFO, попытка %u\n",
                attempt
            );

            delay(10);
            continue;
        }

        if (!CC1101_write_tx_packet(data, length)) {
            Serial.printf(
                "[CC1101] Ошибка записи TX FIFO, попытка %u\n",
                attempt
            );

            delay(10);
            continue;
        }

        if (!CC1101_strobe(CC1101_STX)) {
            Serial.printf(
                "[CC1101] Ошибка команды STX, попытка %u\n",
                attempt
            );

            delay(10);
            continue;
        }

        uint32_t startedAt = millis();
        bool underflow = false;

        while (
            static_cast<uint32_t>(millis() - startedAt) <
            CC1101_TX_TIMEOUT_MS
        ) {
            uint8_t state = 0;
            uint8_t txBytes = 0;

            bool stateOk =
                CC1101_get_marcstate(state);

            bool txBytesOk =
                CC1101_get_txbytes(txBytes);

            if (!stateOk || !txBytesOk) {
                delay(1);
                continue;
            }

            if (
                state == CC1101_STATE_TXFIFO_UNDERFLOW ||
                (txBytes & 0x80)
            ) {
                underflow = true;
                break;
            }

            uint8_t fifoBytes = txBytes & 0x7F;

            /*
             * MCSM1 = 0x30:
             * после TX модуль переходит в IDLE.
             *
             * IDLE + пустой FIFO означает завершение.
             */
            if (
                state == CC1101_STATE_IDLE &&
                fifoBytes == 0
            ) {
                return true;
            }

            delay(1);
        }

        if (underflow) {
            Serial.printf(
                "[CC1101] TX FIFO underflow, попытка %u\n",
                attempt
            );
        } else {
            Serial.printf(
                "[CC1101] Тайм-аут TX, попытка %u\n",
                attempt
            );
        }

        CC1101_enter_idle();
        CC1101_strobe(CC1101_SFTX);

        delay(20U * attempt);
    }

    return false;
}

// =====================================================
// Применение ручной конфигурации
// =====================================================

bool CC1101_apply_configuration()
{
    Serial.println(
        "[CC1101] Запись пользовательских регистров..."
    );

    if (!CC1101_enter_idle()) {
        Serial.println(
            "[CC1101] Не удалось перейти в IDLE"
        );
        return false;
    }

    for (
        size_t i = 0;
        i < CC1101_SETTINGS_COUNT;
        i++
    ) {
        const RegisterSetting &setting =
            CC1101_SETTINGS[i];

        if (!CC1101_write_reg(
                setting.address,
                setting.value)) {

            Serial.printf(
                "[CC1101] Ошибка записи %s "
                "(0x%02X = 0x%02X)\n",
                setting.name,
                setting.address,
                setting.value
            );

            return false;
        }
    }

    if (!CC1101_write_patable0(CC1101_PATABLE0)) {
        Serial.println(
            "[CC1101] Ошибка записи PATABLE[0]"
        );
        return false;
    }

    Serial.printf(
        "[CC1101] Записано %u регистров, "
        "PATABLE[0]=0x%02X\n",
        static_cast<unsigned int>(
            CC1101_SETTINGS_COUNT
        ),
        CC1101_PATABLE0
    );

    return true;
}

bool CC1101_verify_configuration(
    bool verbose,
    bool strictCalibration
) {
    Serial.println(
        "[CC1101] Проверка конфигурации..."
    );

    size_t errorCount = 0;

    for (
        size_t i = 0;
        i < CC1101_SETTINGS_COUNT;
        i++
    ) {
        const RegisterSetting &setting =
            CC1101_SETTINGS[i];

        uint8_t actual = 0;

        if (!CC1101_read_reg(
                setting.address,
                actual)) {

            Serial.printf(
                "[CC1101] %s: ошибка чтения\n",
                setting.name
            );

            errorCount++;
            continue;
        }

        bool matches =
            actual == setting.value;

        bool ignoredCalibrationChange =
            !matches &&
            setting.runtimeMutable &&
            !strictCalibration;

        if (
            verbose ||
            (!matches && !ignoredCalibrationChange)
        ) {
            Serial.printf(
                "[CC1101] 0x%02X %-9s "
                "ожидалось 0x%02X, получено 0x%02X %s\n",
                setting.address,
                setting.name,
                setting.value,
                actual,
                matches
                    ? "OK"
                    : (
                        ignoredCalibrationChange
                            ? "AUTO-CAL"
                            : "ERROR"
                    )
            );
        }

        if (!matches && !ignoredCalibrationChange) {
            errorCount++;
        }
    }

    uint8_t actualPa = 0;

    if (!CC1101_read_patable0(actualPa)) {
        Serial.println(
            "[CC1101] Ошибка чтения PATABLE[0]"
        );

        errorCount++;
    } else if (actualPa != CC1101_PATABLE0) {
        Serial.printf(
            "[CC1101] PATABLE[0]: ожидалось 0x%02X, "
            "получено 0x%02X\n",
            CC1101_PATABLE0,
            actualPa
        );

        errorCount++;
    } else if (verbose) {
        Serial.printf(
            "[CC1101] PATABLE[0] = 0x%02X OK\n",
            actualPa
        );
    }

    Serial.printf(
        "[CC1101] Ошибок конфигурации: %u\n",
        static_cast<unsigned int>(errorCount)
    );

    return errorCount == 0;
}

void CC1101_print_status()
{
    uint8_t partNumber = 0;
    uint8_t version = 0;
    uint8_t marcState = 0;
    uint8_t txBytes = 0;
    uint8_t packetStatus = 0;

    bool ok =
        CC1101_read_status_stable(
            CC1101_PARTNUM,
            partNumber
        ) &&
        CC1101_read_status_stable(
            CC1101_VERSION,
            version
        ) &&
        CC1101_get_marcstate(marcState) &&
        CC1101_get_txbytes(txBytes) &&
        CC1101_read_status_stable(
            CC1101_PKTSTATUS,
            packetStatus
        );

    if (!ok) {
        Serial.println(
            "[CC1101] Ошибка чтения состояния"
        );
        return;
    }

    Serial.printf(
        "[CC1101] PARTNUM:   0x%02X\n",
        partNumber
    );

    Serial.printf(
        "[CC1101] VERSION:   0x%02X\n",
        version
    );

    Serial.printf(
        "[CC1101] MARCSTATE: 0x%02X\n",
        marcState
    );

    Serial.printf(
        "[CC1101] TXBYTES:   0x%02X, FIFO=%u\n",
        txBytes,
        txBytes & 0x7F
    );

    Serial.printf(
        "[CC1101] PKTSTATUS: 0x%02X\n",
        packetStatus
    );
}

bool initCC1101()
{
    if (!CC1101_reset()) {
        return false;
    }

    uint8_t partNumber = 0;
    uint8_t version = 0;

    if (
        !CC1101_read_status_stable(
            CC1101_PARTNUM,
            partNumber
        ) ||
        !CC1101_read_status_stable(
            CC1101_VERSION,
            version
        )
    ) {
        Serial.println(
            "[CC1101] Модуль не отвечает по SPI"
        );

        return false;
    }

    Serial.printf(
        "[CC1101] PARTNUM=0x%02X, VERSION=0x%02X\n",
        partNumber,
        version
    );

    if (
        partNumber == 0xFF &&
        version == 0xFF
    ) {
        Serial.println(
            "[CC1101] Проверьте питание и SPI"
        );

        return false;
    }

    if (!CC1101_apply_configuration()) {
        return false;
    }

    /*
     * Проверяем сразу после записи.
     * До первого TX FSCAL ещё не изменялись.
     */
    if (!CC1101_verify_configuration(
            false,
            true)) {
        return false;
    }

    /*
     * MCSM0 = 0x18 включает автоматическую
     * калибровку при переходе IDLE -> TX/RX.
     */
    if (!CC1101_enter_idle()) {
        return false;
    }

    Serial.println("[CC1101] Модуль готов");
    return true;
}

// =====================================================
// LittleFS
// =====================================================

bool initLittleFS()
{
    Serial.println("[FS] Монтирование LittleFS...");

    if (LittleFS.begin(false)) {
        Serial.println("[FS] LittleFS смонтирован");
        return true;
    }

    Serial.println(
        "[FS] Монтирование не удалось, форматирование..."
    );

    if (!LittleFS.format()) {
        Serial.println(
            "[FS] Ошибка форматирования"
        );
        return false;
    }

    if (!LittleFS.begin(false)) {
        Serial.println(
            "[FS] Ошибка монтирования после форматирования"
        );
        return false;
    }

    Serial.println(
        "[FS] LittleFS отформатирован и смонтирован"
    );

    return true;
}

void printFileSystemInfo()
{
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();

    Serial.printf(
        "[FS] Всего: %u, занято: %u, свободно: %u байт\n",
        static_cast<unsigned int>(total),
        static_cast<unsigned int>(used),
        static_cast<unsigned int>(total - used)
    );
}

// =====================================================
// Камера
// =====================================================

bool initCamera()
{
    Serial.println("[CAM] Инициализация камеры...");

    camera_config_t config = {};

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = CAM_PIN_Y2;
    config.pin_d1 = CAM_PIN_Y3;
    config.pin_d2 = CAM_PIN_Y4;
    config.pin_d3 = CAM_PIN_Y5;
    config.pin_d4 = CAM_PIN_Y6;
    config.pin_d5 = CAM_PIN_Y7;
    config.pin_d6 = CAM_PIN_Y8;
    config.pin_d7 = CAM_PIN_Y9;

    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;

    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;

    config.pin_pwdn = CAM_PIN_PWDN;
    config.pin_reset = CAM_PIN_RESET;

    config.xclk_freq_hz = 10000000;

    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = CAMERA_FRAME_SIZE;
    config.jpeg_quality = CAMERA_JPEG_QUALITY;

    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    if (psramFound()) {
        Serial.println("[CAM] PSRAM обнаружена");
        config.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        Serial.println("[CAM] PSRAM отсутствует");
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    esp_err_t result =
        esp_camera_init(&config);

    if (result != ESP_OK) {
        Serial.printf(
            "[CAM] Ошибка инициализации: 0x%X\n",
            static_cast<unsigned int>(result)
        );

        return false;
    }

    sensor_t *sensor =
        esp_camera_sensor_get();

    if (sensor != nullptr) {
        sensor->set_framesize(
            sensor,
            CAMERA_FRAME_SIZE
        );

        sensor->set_quality(
            sensor,
            CAMERA_JPEG_QUALITY
        );

        // При необходимости:
        // sensor->set_vflip(sensor, 1);
        // sensor->set_hmirror(sensor, 1);
    }

    Serial.println("[CAM] Камера готова");
    return true;
}

bool isValidJpeg(
    const uint8_t *data,
    size_t length
) {
    if (data == nullptr || length < 4) {
        return false;
    }

    return
        data[0] == 0xFF &&
        data[1] == 0xD8 &&
        data[length - 2] == 0xFF &&
        data[length - 1] == 0xD9;
}

bool captureAndSavePhoto(PhotoInfo &photoInfo)
{
    Serial.println("[CAM] Получение кадра...");

    camera_fb_t *frame =
        esp_camera_fb_get();

    if (frame == nullptr) {
        Serial.println(
            "[CAM] Камера не вернула кадр"
        );

        return false;
    }

    if (
        frame->format != PIXFORMAT_JPEG ||
        !isValidJpeg(frame->buf, frame->len)
    ) {
        Serial.println(
            "[CAM] Получен некорректный JPEG"
        );

        esp_camera_fb_return(frame);
        return false;
    }

    photoInfo.size =
        static_cast<uint32_t>(frame->len);

    photoInfo.width =
        static_cast<uint16_t>(frame->width);

    photoInfo.height =
        static_cast<uint16_t>(frame->height);

    Serial.printf(
        "[CAM] Кадр: %ux%u, %u байт\n",
        photoInfo.width,
        photoInfo.height,
        static_cast<unsigned int>(photoInfo.size)
    );

    LittleFS.remove(TEMP_PATH);

    File file =
        LittleFS.open(TEMP_PATH, FILE_WRITE);

    if (!file) {
        Serial.println(
            "[FS] Не удалось открыть временный файл"
        );

        esp_camera_fb_return(frame);
        return false;
    }

    size_t written =
        file.write(frame->buf, frame->len);

    file.flush();
    file.close();

    esp_camera_fb_return(frame);
    frame = nullptr;

    if (written != photoInfo.size) {
        Serial.printf(
            "[FS] Записано %u из %u байт\n",
            static_cast<unsigned int>(written),
            static_cast<unsigned int>(photoInfo.size)
        );

        LittleFS.remove(TEMP_PATH);
        return false;
    }

    File checkFile =
        LittleFS.open(TEMP_PATH, FILE_READ);

    if (!checkFile) {
        LittleFS.remove(TEMP_PATH);
        return false;
    }

    uint32_t savedSize =
        static_cast<uint32_t>(checkFile.size());

    checkFile.close();

    if (savedSize != photoInfo.size) {
        Serial.println(
            "[FS] Размер сохранённого файла не совпадает"
        );

        LittleFS.remove(TEMP_PATH);
        return false;
    }

    LittleFS.remove(PHOTO_PATH);

    if (!LittleFS.rename(
            TEMP_PATH,
            PHOTO_PATH)) {

        Serial.println(
            "[FS] Не удалось переименовать файл"
        );

        return false;
    }

    Serial.printf(
        "[FS] Сохранено: %s\n",
        PHOTO_PATH
    );

    return true;
}

bool getStoredPhotoInfo(PhotoInfo &photoInfo)
{
    File file =
        LittleFS.open(PHOTO_PATH, FILE_READ);

    if (!file) {
        Serial.println(
            "[FS] Изображение отсутствует"
        );

        return false;
    }

    photoInfo.size =
        static_cast<uint32_t>(file.size());

    photoInfo.width = CAMERA_WIDTH;
    photoInfo.height = CAMERA_HEIGHT;

    file.close();
    return true;
}

bool calculateFileCrc32(
    const char *path,
    uint32_t &result
) {
    File file =
        LittleFS.open(path, FILE_READ);

    if (!file) {
        return false;
    }

    uint8_t buffer[512];
    uint32_t crc = 0xFFFFFFFFUL;

    while (file.available()) {
        size_t bytesRead =
            file.read(buffer, sizeof(buffer));

        if (bytesRead == 0) {
            file.close();
            return false;
        }

        crc = crc32Update(
            crc,
            buffer,
            bytesRead
        );

        delay(0);
    }

    file.close();

    result = crc ^ 0xFFFFFFFFUL;
    return true;
}

// =====================================================
// Формирование радиопакета протокола
// =====================================================

bool sendImageProtocolPacket(
    ImagePacketType packetType,
    uint16_t imageId,
    uint16_t sequence,
    uint16_t totalDataPackets,
    const uint8_t *payload,
    uint8_t payloadLength
) {
    if (payloadLength > IMAGE_BYTES_PER_PACKET) {
        return false;
    }

    if (
        payloadLength > 0 &&
        payload == nullptr
    ) {
        return false;
    }

    uint8_t packet[RADIO_PACKET_MAX_SIZE] = {};

    packet[0] = PROTOCOL_MAGIC_0;
    packet[1] = PROTOCOL_MAGIC_1;
    packet[2] = PROTOCOL_VERSION;
    packet[3] = static_cast<uint8_t>(packetType);

    writeUint16LE(&packet[4], imageId);
    writeUint16LE(&packet[6], sequence);
    writeUint16LE(
        &packet[8],
        totalDataPackets
    );

    packet[10] = payloadLength;
    packet[11] = 0;

    if (payloadLength > 0) {
        memcpy(
            &packet[PROTOCOL_HEADER_SIZE],
            payload,
            payloadLength
        );
    }

    uint8_t lengthBeforeCrc =
        PROTOCOL_HEADER_SIZE +
        payloadLength;

    uint32_t packetCrc =
        calculateBufferCrc32(
            packet,
            lengthBeforeCrc
        );

    writeUint32LE(
        &packet[lengthBeforeCrc],
        packetCrc
    );

    uint8_t completeLength =
        lengthBeforeCrc +
        PROTOCOL_CRC_SIZE;

    return CC1101_send_packet(
        packet,
        completeLength
    );
}

// =====================================================
// Отправка изображения
// =====================================================

bool sendPhotoCc1101(
    const char *path,
    const PhotoInfo &photoInfo
) {
    uint32_t imageCrc32 = 0;

    Serial.println(
        "[TX] Вычисление CRC32 изображения..."
    );

    if (!calculateFileCrc32(
            path,
            imageCrc32)) {

        Serial.println(
            "[TX] Ошибка подсчёта CRC32"
        );

        return false;
    }

    File file =
        LittleFS.open(path, FILE_READ);

    if (!file) {
        Serial.println(
            "[TX] Не удалось открыть изображение"
        );

        return false;
    }

    uint32_t fileSize =
        static_cast<uint32_t>(file.size());

    if (fileSize != photoInfo.size) {
        Serial.println(
            "[TX] Размер файла не совпадает"
        );

        file.close();
        return false;
    }

    uint32_t packetCount32 =
        (
            photoInfo.size +
            IMAGE_BYTES_PER_PACKET - 1
        ) /
        IMAGE_BYTES_PER_PACKET;

    if (
        packetCount32 == 0 ||
        packetCount32 > 65535UL
    ) {
        Serial.println(
            "[TX] Слишком большое изображение"
        );

        file.close();
        return false;
    }

    uint16_t packetCount =
        static_cast<uint16_t>(packetCount32);

    uint16_t imageId = nextImageId++;

    if (nextImageId == 0) {
        nextImageId = 1;
    }

    Serial.println();
    Serial.println("======================================");
    Serial.println("[TX] Передача изображения");
    Serial.printf("[TX] Image ID:        %u\n", imageId);
    Serial.printf(
        "[TX] Размер:          %u байт\n",
        static_cast<unsigned int>(photoInfo.size)
    );
    Serial.printf(
        "[TX] Разрешение:      %ux%u\n",
        photoInfo.width,
        photoInfo.height
    );
    Serial.printf(
        "[TX] DATA-пакетов:    %u\n",
        packetCount
    );
    Serial.printf(
        "[TX] Полезных байт:   %u\n",
        IMAGE_BYTES_PER_PACKET
    );
    Serial.printf(
        "[TX] CRC32:           %08X\n",
        static_cast<unsigned int>(imageCrc32)
    );
    Serial.println("======================================");

    uint32_t startedAt = millis();

    // -------------------------------------------------
    // START
    // -------------------------------------------------

    /*
     * START payload:
     *
     * 0..3   размер JPEG
     * 4..7   CRC32 JPEG
     * 8..9   ширина
     * 10..11 высота
     * 12     размер одного DATA payload
     * 13..14 количество DATA-пакетов
     */

    uint8_t startPayload[15] = {};

    writeUint32LE(
        &startPayload[0],
        photoInfo.size
    );

    writeUint32LE(
        &startPayload[4],
        imageCrc32
    );

    writeUint16LE(
        &startPayload[8],
        photoInfo.width
    );

    writeUint16LE(
        &startPayload[10],
        photoInfo.height
    );

    startPayload[12] =
        IMAGE_BYTES_PER_PACKET;

    writeUint16LE(
        &startPayload[13],
        packetCount
    );

    Serial.println("[TX] START");

    if (!sendImageProtocolPacket(
            PACKET_START,
            imageId,
            0,
            packetCount,
            startPayload,
            sizeof(startPayload))) {

        Serial.println(
            "[TX] Ошибка START-пакета"
        );

        file.close();
        return false;
    }

    delay(INTER_PACKET_DELAY_MS);

    // -------------------------------------------------
    // DATA
    // -------------------------------------------------

    uint8_t dataBuffer[IMAGE_BYTES_PER_PACKET];

    for (
        uint32_t packetIndex = 0;
        packetIndex < packetCount;
        packetIndex++
    ) {
        uint32_t offset =
            packetIndex *
            IMAGE_BYTES_PER_PACKET;

        uint32_t remaining =
            photoInfo.size - offset;

        uint8_t requested =
            remaining > IMAGE_BYTES_PER_PACKET
                ? IMAGE_BYTES_PER_PACKET
                : static_cast<uint8_t>(remaining);

        size_t bytesRead =
            file.read(dataBuffer, requested);

        if (bytesRead != requested) {
            Serial.printf(
                "[TX] Ошибка чтения DATA %u\n",
                static_cast<unsigned int>(packetIndex)
            );

            file.close();
            return false;
        }

        if (!sendImageProtocolPacket(
                PACKET_DATA,
                imageId,
                static_cast<uint16_t>(packetIndex),
                packetCount,
                dataBuffer,
                requested)) {

            Serial.printf(
                "[TX] Ошибка DATA %u\n",
                static_cast<unsigned int>(packetIndex)
            );

            file.close();
            return false;
        }

        uint32_t sentPackets =
            packetIndex + 1;

        if (
            sentPackets % 10 == 0 ||
            sentPackets == packetCount
        ) {
            float percent =
                static_cast<float>(sentPackets) *
                100.0f /
                static_cast<float>(packetCount);

            Serial.printf(
                "[TX] %u/%u, %5.1f%%\n",
                static_cast<unsigned int>(sentPackets),
                packetCount,
                percent
            );
        }

        delay(INTER_PACKET_DELAY_MS);
        delay(0);
    }

    file.close();

    // -------------------------------------------------
    // END
    // -------------------------------------------------

    /*
     * END payload:
     *
     * 0..3 размер JPEG
     * 4..7 CRC32 JPEG
     * 8..9 количество DATA-пакетов
     */

    uint8_t endPayload[10] = {};

    writeUint32LE(
        &endPayload[0],
        photoInfo.size
    );

    writeUint32LE(
        &endPayload[4],
        imageCrc32
    );

    writeUint16LE(
        &endPayload[8],
        packetCount
    );

    Serial.println("[TX] END");

    if (!sendImageProtocolPacket(
            PACKET_END,
            imageId,
            packetCount,
            packetCount,
            endPayload,
            sizeof(endPayload))) {

        Serial.println(
            "[TX] Ошибка END-пакета"
        );

        return false;
    }

    uint32_t elapsed =
        millis() - startedAt;

    Serial.println("======================================");
    Serial.println("[TX] Передача завершена");
    Serial.printf(
        "[TX] Время: %u.%03u секунд\n",
        static_cast<unsigned int>(elapsed / 1000),
        static_cast<unsigned int>(elapsed % 1000)
    );
    Serial.println("======================================");

    return true;
}

// =====================================================
// Информация о сохранённом файле
// =====================================================

void printPhotoInfo()
{
    PhotoInfo photoInfo = {};

    if (!getStoredPhotoInfo(photoInfo)) {
        return;
    }

    uint32_t crc32 = 0;

    Serial.printf(
        "[FS] Файл: %s\n",
        PHOTO_PATH
    );

    Serial.printf(
        "[FS] Размер: %u байт\n",
        static_cast<unsigned int>(photoInfo.size)
    );

    if (calculateFileCrc32(
            PHOTO_PATH,
            crc32)) {

        Serial.printf(
            "[FS] CRC32: %08X\n",
            static_cast<unsigned int>(crc32)
        );
    }
}

// =====================================================
// UART-команды
// =====================================================

void printCommands()
{
    Serial.println();
    Serial.println("Команды:");
    Serial.println(
        "  SEND    - снять, сохранить и передать"
    );
    Serial.println(
        "  CAPTURE - только снять и сохранить"
    );
    Serial.println(
        "  TX      - передать сохранённый JPEG"
    );
    Serial.println(
        "  INFO    - информация о файле"
    );
    Serial.println(
        "  RADIO   - состояние CC1101"
    );
    Serial.println(
        "  REGS    - проверить регистры"
    );
    Serial.println(
        "  CONFIG  - повторно записать конфигурацию"
    );
    Serial.println(
        "  DELETE  - удалить JPEG"
    );
    Serial.println(
        "  HELP    - список команд"
    );
    Serial.println();
}

void processCommand(String command)
{
    command.trim();
    command.toUpperCase();

    if (command.length() == 0) {
        return;
    }

    if (command == "SEND") {
        PhotoInfo photoInfo = {};

        if (!captureAndSavePhoto(photoInfo)) {
            Serial.println(
                "[MAIN] Ошибка получения изображения"
            );

            return;
        }

        if (!sendPhotoCc1101(
                PHOTO_PATH,
                photoInfo)) {

            Serial.println(
                "[MAIN] Ошибка радиопередачи"
            );
        }

        return;
    }

    if (command == "CAPTURE") {
        PhotoInfo photoInfo = {};

        if (captureAndSavePhoto(photoInfo)) {
            Serial.println(
                "[MAIN] Изображение сохранено"
            );
        }

        return;
    }

    if (command == "TX") {
        PhotoInfo photoInfo = {};

        if (!getStoredPhotoInfo(photoInfo)) {
            return;
        }

        if (!sendPhotoCc1101(
                PHOTO_PATH,
                photoInfo)) {

            Serial.println(
                "[MAIN] Ошибка радиопередачи"
            );
        }

        return;
    }

    if (command == "INFO") {
        printFileSystemInfo();
        printPhotoInfo();
        return;
    }

    if (command == "RADIO") {
        CC1101_print_status();
        return;
    }

    if (command == "REGS") {
        /*
         * FSCAL-регистры могут измениться после TX,
         * поэтому их изменение не считается ошибкой.
         */
        CC1101_verify_configuration(
            true,
            false
        );

        return;
    }

    if (command == "CONFIG") {
        if (
            CC1101_apply_configuration() &&
            CC1101_verify_configuration(
                false,
                true
            )
        ) {
            Serial.println(
                "[CC1101] Конфигурация восстановлена"
            );
        }

        return;
    }

    if (command == "DELETE") {
        if (LittleFS.remove(PHOTO_PATH)) {
            Serial.println(
                "[FS] Изображение удалено"
            );
        } else {
            Serial.println(
                "[FS] Изображение отсутствует"
            );
        }

        return;
    }

    if (command == "HELP") {
        printCommands();
        return;
    }

    Serial.printf(
        "[UART] Неизвестная команда: %s\n",
        command.c_str()
    );

    printCommands();
}

// =====================================================
// Arduino
// =====================================================

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(250);

    delay(1000);

    Serial.println();
    Serial.println("======================================");
    Serial.println("ESP32-CAM + CC1101");
    Serial.println("Прямое управление CC1101 через SPI");
    Serial.println("UART используется только для отладки");
    Serial.println("GDO0/GDO2 не используются");
    Serial.println("======================================");

    if (!initLittleFS()) {
        Serial.println(
            "[MAIN] Критическая ошибка LittleFS"
        );

        while (true) {
            delay(1000);
        }
    }

    printFileSystemInfo();

    if (!initCamera()) {
        Serial.println(
            "[MAIN] Критическая ошибка камеры"
        );

        while (true) {
            delay(1000);
        }
    }

    if (!initCC1101()) {
        Serial.println(
            "[MAIN] Критическая ошибка CC1101"
        );

        while (true) {
            delay(1000);
        }
    }

    Serial.println();
    Serial.println("[MAIN] READY");

    printCommands();
}

void loop()
{
    if (!Serial.available()) {
        delay(10);
        return;
    }

    String command =
        Serial.readStringUntil('\n');

    processCommand(command);
}