#include <Arduino.h>
#include "esp_camera.h"
#include <LittleFS.h>

// =====================================================
// AI-Thinker ESP32-CAM: выводы камеры
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

#define FLASH_LED_PIN     4

// =====================================================
// Настройки
// =====================================================

constexpr uint32_t SERIAL_BAUD = 115200;

// 220 байт выбрано с учётом будущего перехода на CC1101.
// Вместе с заголовком пакета размер будет меньше 255 байт.
constexpr size_t UART_CHUNK_SIZE = 220;

const char *PHOTO_PATH = "/photo.jpg";
const char *TEMP_PATH  = "/photo.tmp";

uint32_t nextImageId = 1;

// =====================================================
// Структуры протокола
// Все числа передаются в little-endian.
// ESP32 использует little-endian.
// =====================================================

struct __attribute__((packed)) ImageHeader {
    char magic[4];             // "IMG1"
    uint16_t protocolVersion;  // 1
    uint16_t headerSize;       // sizeof(ImageHeader)
    uint32_t imageId;
    uint32_t imageSize;
    uint32_t imageCrc32;
    uint16_t width;
    uint16_t height;
    uint16_t chunkSize;
    uint16_t chunkCount;
};

struct __attribute__((packed)) ChunkHeader {
    char magic[4];             // "DAT1"
    uint32_t imageId;
    uint16_t chunkIndex;
    uint16_t payloadSize;
    uint32_t payloadCrc32;
};

struct __attribute__((packed)) EndPacket {
    char magic[4];             // "END1"
    uint32_t imageId;
    uint32_t imageCrc32;
};

static_assert(sizeof(ImageHeader) == 28, "Некорректный размер ImageHeader");
static_assert(sizeof(ChunkHeader) == 16, "Некорректный размер ChunkHeader");
static_assert(sizeof(EndPacket) == 12, "Некорректный размер EndPacket");

struct PhotoInfo {
    uint32_t size;
    uint16_t width;
    uint16_t height;
};

// =====================================================
// CRC32
// Полином совместим с Python zlib.crc32()
// =====================================================

uint32_t crc32Update(
    uint32_t crc,
    const uint8_t *data,
    size_t length
) {
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1) {
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
// Надёжная запись в UART
// =====================================================

bool uartWriteAll(const uint8_t *data, size_t length)
{
    size_t totalWritten = 0;
    unsigned long lastProgress = millis();

    while (totalWritten < length) {
        size_t written = Serial.write(
            data + totalWritten,
            length - totalWritten
        );

        if (written > 0) {
            totalWritten += written;
            lastProgress = millis();
        } else {
            delay(1);

            if (millis() - lastProgress > 5000) {
                return false;
            }
        }
    }

    return true;
}

template <typename T>
bool uartWriteStruct(const T &value)
{
    return uartWriteAll(
        reinterpret_cast<const uint8_t *>(&value),
        sizeof(T)
    );
}

// =====================================================
// LittleFS
// =====================================================

bool initLittleFS()
{
    if (LittleFS.begin(false)) {
        Serial.println("LittleFS смонтирован");
        return true;
    }

    Serial.println("LittleFS не смонтирован");
    Serial.println("Форматирование LittleFS...");

    if (!LittleFS.format()) {
        Serial.println("Ошибка форматирования LittleFS");
        return false;
    }

    if (!LittleFS.begin(false)) {
        Serial.println("Ошибка монтирования LittleFS после форматирования");
        return false;
    }

    Serial.println("LittleFS отформатирован и смонтирован");
    return true;
}

// =====================================================
// Камера
// =====================================================

bool initCamera()
{
    camera_config_t config = {};

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    config.pin_d0 = CAM_PIN_Y2;
    config.pin_d1 = CAM_PIN_Y3;
    config.pin_d2 = CAM_PIN_Y4;
    config.pin_d3 = CAM_PIN_Y5;
    config.pin_d4 = CAM_PIN_Y6;
    config.pin_d5 = CAM_PIN_Y7;
    config.pin_d6 = CAM_PIN_Y8;
    config.pin_d7 = CAM_PIN_Y9;

    config.pin_xclk  = CAM_PIN_XCLK;
    config.pin_pclk  = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href  = CAM_PIN_HREF;

    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;

    config.pin_pwdn  = CAM_PIN_PWDN;
    config.pin_reset = CAM_PIN_RESET;

    // Сниженная частота для более стабильного запуска.
    config.xclk_freq_hz = 10000000;

    config.pixel_format = PIXFORMAT_JPEG;

    // Для CC1101 разумно начинать с QVGA.
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 18;

    // Один framebuffer: камера не работает постоянно.
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    if (psramFound()) {
        Serial.println("PSRAM обнаружена");
        config.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        Serial.println("PSRAM не обнаружена");
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    esp_err_t result = esp_camera_init(&config);

    if (result != ESP_OK) {
        Serial.printf(
            "Ошибка инициализации камеры: 0x%X\n",
            static_cast<unsigned int>(result)
        );
        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();

    if (sensor != nullptr) {
        sensor->set_framesize(sensor, FRAMESIZE_QVGA);
        sensor->set_quality(sensor, 18);

        // При необходимости:
        // sensor->set_vflip(sensor, 1);
        // sensor->set_hmirror(sensor, 1);
    }

    Serial.println("Камера инициализирована");
    return true;
}

bool isValidJpeg(const uint8_t *data, size_t length)
{
    if (data == nullptr || length < 4) {
        return false;
    }

    return data[0] == 0xFF &&
           data[1] == 0xD8 &&
           data[length - 2] == 0xFF &&
           data[length - 1] == 0xD9;
}

// =====================================================
// Захват и сохранение изображения
// =====================================================

bool captureAndSavePhoto(PhotoInfo &photoInfo)
{
    camera_fb_t *frame = esp_camera_fb_get();

    if (frame == nullptr) {
        Serial.println("ERR: камера не вернула кадр");
        return false;
    }

    if (frame->format != PIXFORMAT_JPEG) {
        Serial.println("ERR: кадр не является JPEG");
        esp_camera_fb_return(frame);
        return false;
    }

    if (!isValidJpeg(frame->buf, frame->len)) {
        Serial.println("ERR: некорректные маркеры JPEG");
        esp_camera_fb_return(frame);
        return false;
    }

    photoInfo.size = static_cast<uint32_t>(frame->len);
    photoInfo.width = static_cast<uint16_t>(frame->width);
    photoInfo.height = static_cast<uint16_t>(frame->height);

    LittleFS.remove(TEMP_PATH);

    File file = LittleFS.open(TEMP_PATH, FILE_WRITE);

    if (!file) {
        Serial.println("ERR: невозможно открыть временный файл");
        esp_camera_fb_return(frame);
        return false;
    }

    size_t written = file.write(frame->buf, frame->len);

    file.flush();
    file.close();

    esp_camera_fb_return(frame);
    frame = nullptr;

    if (written != photoInfo.size) {
        Serial.printf(
            "ERR: записано %u из %u байт\n",
            static_cast<unsigned int>(written),
            static_cast<unsigned int>(photoInfo.size)
        );

        LittleFS.remove(TEMP_PATH);
        return false;
    }

    File checkFile = LittleFS.open(TEMP_PATH, FILE_READ);

    if (!checkFile) {
        Serial.println("ERR: невозможно проверить временный файл");
        LittleFS.remove(TEMP_PATH);
        return false;
    }

    uint32_t savedSize = static_cast<uint32_t>(checkFile.size());
    checkFile.close();

    if (savedSize != photoInfo.size) {
        Serial.println("ERR: размер сохранённого файла не совпадает");
        LittleFS.remove(TEMP_PATH);
        return false;
    }

    LittleFS.remove(PHOTO_PATH);

    if (!LittleFS.rename(TEMP_PATH, PHOTO_PATH)) {
        Serial.println("ERR: невозможно переименовать временный файл");
        return false;
    }

    return true;
}

// =====================================================
// Подсчёт CRC файла
// =====================================================

bool calculateFileCrc32(
    const char *path,
    uint32_t &resultCrc
) {
    File file = LittleFS.open(path, FILE_READ);

    if (!file) {
        return false;
    }

    uint8_t buffer[512];
    uint32_t crc = 0xFFFFFFFFUL;

    while (file.available()) {
        size_t bytesRead = file.read(buffer, sizeof(buffer));

        if (bytesRead == 0) {
            file.close();
            return false;
        }

        crc = crc32Update(crc, buffer, bytesRead);
        delay(0);
    }

    file.close();

    resultCrc = crc ^ 0xFFFFFFFFUL;
    return true;
}

// =====================================================
// Отправка изображения по UART
// =====================================================

bool sendPhotoUart(
    const char *path,
    const PhotoInfo &photoInfo
) {
    uint32_t imageCrc32 = 0;

    if (!calculateFileCrc32(path, imageCrc32)) {
        Serial.println("ERR: невозможно вычислить CRC изображения");
        return false;
    }

    File file = LittleFS.open(path, FILE_READ);

    if (!file) {
        Serial.println("ERR: невозможно открыть изображение");
        return false;
    }

    uint32_t imageId = nextImageId++;

    uint32_t calculatedChunks =
        (photoInfo.size + UART_CHUNK_SIZE - 1) /
        UART_CHUNK_SIZE;

    if (calculatedChunks > 65535) {
        Serial.println("ERR: слишком много блоков");
        file.close();
        return false;
    }

    uint16_t chunkCount =
        static_cast<uint16_t>(calculatedChunks);

    ImageHeader header = {};

    memcpy(header.magic, "IMG1", 4);
    header.protocolVersion = 1;
    header.headerSize = sizeof(ImageHeader);
    header.imageId = imageId;
    header.imageSize = photoInfo.size;
    header.imageCrc32 = imageCrc32;
    header.width = photoInfo.width;
    header.height = photoInfo.height;
    header.chunkSize = UART_CHUNK_SIZE;
    header.chunkCount = chunkCount;

    // С этого места до END1 нельзя выводить текст через Serial.
    if (!uartWriteStruct(header)) {
        file.close();
        return false;
    }

    uint8_t buffer[UART_CHUNK_SIZE];
    uint32_t remaining = photoInfo.size;

    for (uint16_t index = 0; index < chunkCount; index++) {
        size_t requested = UART_CHUNK_SIZE;

        if (remaining < requested) {
            requested = remaining;
        }

        size_t bytesRead = file.read(buffer, requested);

        if (bytesRead != requested) {
            file.close();
            return false;
        }

        ChunkHeader chunkHeader = {};

        memcpy(chunkHeader.magic, "DAT1", 4);
        chunkHeader.imageId = imageId;
        chunkHeader.chunkIndex = index;
        chunkHeader.payloadSize =
            static_cast<uint16_t>(bytesRead);
        chunkHeader.payloadCrc32 =
            calculateBufferCrc32(buffer, bytesRead);

        if (!uartWriteStruct(chunkHeader)) {
            file.close();
            return false;
        }

        if (!uartWriteAll(buffer, bytesRead)) {
            file.close();
            return false;
        }

        remaining -= bytesRead;

        // Позволяем системным задачам ESP32 выполняться.
        delay(0);
    }

    file.close();

    EndPacket endPacket = {};

    memcpy(endPacket.magic, "END1", 4);
    endPacket.imageId = imageId;
    endPacket.imageCrc32 = imageCrc32;

    if (!uartWriteStruct(endPacket)) {
        return false;
    }

    Serial.flush();

    // После END1 текст снова разрешён.
    Serial.println();
    Serial.printf(
        "TX завершена: imageId=%u, size=%u, chunks=%u, CRC32=%08X\n",
        static_cast<unsigned int>(imageId),
        static_cast<unsigned int>(photoInfo.size),
        static_cast<unsigned int>(chunkCount),
        static_cast<unsigned int>(imageCrc32)
    );

    return true;
}

// =====================================================
// Обработка команд
// =====================================================

void processCommand(String command)
{
    command.trim();
    command.toUpperCase();

    if (command == "GET") {
        Serial.println("Подготовка изображения...");

        PhotoInfo photoInfo = {};

        if (!captureAndSavePhoto(photoInfo)) {
            return;
        }

        Serial.printf(
            "Снимок сохранён: %ux%u, %u байт\n",
            photoInfo.width,
            photoInfo.height,
            static_cast<unsigned int>(photoInfo.size)
        );

        Serial.println("Начало бинарной передачи...");

        sendPhotoUart(PHOTO_PATH, photoInfo);
        return;
    }

    if (command == "INFO") {
        if (!LittleFS.exists(PHOTO_PATH)) {
            Serial.println("Изображение отсутствует");
            return;
        }

        File file = LittleFS.open(PHOTO_PATH, FILE_READ);

        if (!file) {
            Serial.println("Ошибка открытия изображения");
            return;
        }

        Serial.printf(
            "%s: %u байт\n",
            PHOTO_PATH,
            static_cast<unsigned int>(file.size())
        );

        file.close();
        return;
    }

    if (command == "DELETE") {
        if (LittleFS.remove(PHOTO_PATH)) {
            Serial.println("Изображение удалено");
        } else {
            Serial.println("Изображение не найдено");
        }

        return;
    }

    Serial.println("Доступные команды:");
    Serial.println("  GET");
    Serial.println("  INFO");
    Serial.println("  DELETE");
}

// =====================================================
// Arduino
// =====================================================

void setup()
{
    Serial.begin(SERIAL_BAUD);
    Serial.setTimeout(200);

    delay(1000);

    Serial.println();
    Serial.println("ESP32-CAM UART image transmitter");

    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(FLASH_LED_PIN, LOW);

    if (!initLittleFS()) {
        while (true) {
            delay(1000);
        }
    }

    if (!initCamera()) {
        while (true) {
            delay(1000);
        }
    }

    Serial.println("READY");
    Serial.println("Ожидание команды GET");
}

void loop()
{
    if (!Serial.available()) {
        delay(10);
        return;
    }

    String command = Serial.readStringUntil('\n');
    processCommand(command);
}