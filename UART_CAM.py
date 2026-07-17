from __future__ import annotations

import io
import struct
import time
import zlib
from pathlib import Path

import serial
from PIL import Image


# =====================================================
# Настройки
# =====================================================

PORT = "/dev/tty.usbmodemFX2348N1"
BAUD_RATE = 115200

IMAGE_MAGIC = b"IMG1"
CHUNK_MAGIC = b"DAT1"
END_MAGIC = b"END1"

IMAGE_HEADER_FORMAT = "<4sHHIIIHHHH"
CHUNK_HEADER_FORMAT = "<4sIHHI"
END_PACKET_FORMAT = "<4sII"

IMAGE_HEADER_SIZE = struct.calcsize(IMAGE_HEADER_FORMAT)
CHUNK_HEADER_SIZE = struct.calcsize(CHUNK_HEADER_FORMAT)
END_PACKET_SIZE = struct.calcsize(END_PACKET_FORMAT)

SCRIPT_DIRECTORY = Path(__file__).resolve().parent
OUTPUT_DIRECTORY = SCRIPT_DIRECTORY / "received_images"


class ProtocolError(RuntimeError):
    pass


def read_exact(
    port: serial.Serial,
    count: int,
    timeout_seconds: float = 10.0,
) -> bytes:
    """Прочитать ровно указанное количество байт."""

    result = bytearray()
    deadline = time.monotonic() + timeout_seconds

    while len(result) < count:
        block = port.read(count - len(result))

        if block:
            result.extend(block)

            # После каждого принятого блока обновляем тайм-аут.
            deadline = time.monotonic() + timeout_seconds
            continue

        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"Тайм-аут: принято {len(result)} из {count} байт"
            )

    return bytes(result)


def wait_for_esp32(
    port: serial.Serial,
    timeout_seconds: float = 12.0,
) -> bool:
    """
    Ждём строку READY от ESP32 и одновременно показываем
    диагностический вывод платы.
    """

    print("Ожидание загрузки ESP32-CAM...")

    deadline = time.monotonic() + timeout_seconds
    line_buffer = bytearray()

    while time.monotonic() < deadline:
        byte = port.read(1)

        if not byte:
            continue

        line_buffer.extend(byte)

        if byte == b"\n":
            text = line_buffer.decode(
                "utf-8",
                errors="replace",
            ).strip()

            line_buffer.clear()

            if text:
                print(f"[ESP32] {text}")

            if "READY" in text:
                print("ESP32-CAM готова")
                return True

    print("Предупреждение: строка READY не получена")
    print("Попытка отправить GET всё равно будет выполнена")

    return False


def find_image_header(
    port: serial.Serial,
    timeout_seconds: float = 30.0,
) -> None:
    """
    Ищем IMG1 в потоке.

    До IMG1 ESP32 может отправлять обычные текстовые сообщения.
    """

    print("Ожидание начала бинарного изображения IMG1...")

    deadline = time.monotonic() + timeout_seconds

    magic_window = bytearray()
    text_buffer = bytearray()

    while time.monotonic() < deadline:
        byte = port.read(1)

        if not byte:
            continue

        magic_window.extend(byte)

        if len(magic_window) > len(IMAGE_MAGIC):
            del magic_window[0]

        if bytes(magic_window) == IMAGE_MAGIC:
            print("Получена сигнатура IMG1")
            return

        text_buffer.extend(byte)

        if byte == b"\n":
            text = text_buffer.decode(
                "utf-8",
                errors="replace",
            ).strip()

            text_buffer.clear()

            if text:
                print(f"[ESP32] {text}")

    raise TimeoutError(
        "ESP32 не передала заголовок IMG1 за 30 секунд"
    )


def receive_image(
    port: serial.Serial,
) -> tuple[bytes, dict[str, int]]:
    find_image_header(port)

    remaining_header = read_exact(
        port,
        IMAGE_HEADER_SIZE - len(IMAGE_MAGIC),
    )

    raw_header = IMAGE_MAGIC + remaining_header

    (
        magic,
        protocol_version,
        header_size,
        image_id,
        image_size,
        expected_image_crc,
        width,
        height,
        chunk_size,
        chunk_count,
    ) = struct.unpack(
        IMAGE_HEADER_FORMAT,
        raw_header,
    )

    if magic != IMAGE_MAGIC:
        raise ProtocolError("Неверная сигнатура изображения")

    if protocol_version != 1:
        raise ProtocolError(
            f"Неподдерживаемая версия: {protocol_version}"
        )

    if header_size != IMAGE_HEADER_SIZE:
        raise ProtocolError(
            f"Неверный размер заголовка: {header_size}, "
            f"ожидалось {IMAGE_HEADER_SIZE}"
        )

    if image_size == 0:
        raise ProtocolError("ESP32 передала нулевой размер изображения")

    if chunk_count == 0:
        raise ProtocolError("ESP32 передала нулевое количество блоков")

    print()
    print("Параметры изображения:")
    print(f"  ID:             {image_id}")
    print(f"  Размер:         {image_size} байт")
    print(f"  Разрешение:     {width}x{height}")
    print(f"  Размер блока:   {chunk_size} байт")
    print(f"  Количество:     {chunk_count}")
    print(f"  Ожидаемый CRC:  {expected_image_crc:08X}")
    print()

    image_data = bytearray()

    for expected_index in range(chunk_count):
        raw_chunk_header = read_exact(
            port,
            CHUNK_HEADER_SIZE,
        )

        (
            chunk_magic,
            chunk_image_id,
            chunk_index,
            payload_size,
            expected_chunk_crc,
        ) = struct.unpack(
            CHUNK_HEADER_FORMAT,
            raw_chunk_header,
        )

        if chunk_magic != CHUNK_MAGIC:
            raise ProtocolError(
                f"Блок {expected_index}: ожидалась сигнатура DAT1, "
                f"получено {chunk_magic!r}"
            )

        if chunk_image_id != image_id:
            raise ProtocolError(
                f"Блок {expected_index}: неверный image ID"
            )

        if chunk_index != expected_index:
            raise ProtocolError(
                f"Ожидался блок {expected_index}, "
                f"получен блок {chunk_index}"
            )

        if payload_size == 0:
            raise ProtocolError(
                f"Блок {chunk_index} имеет нулевой размер"
            )

        if payload_size > chunk_size:
            raise ProtocolError(
                f"Блок {chunk_index} слишком большой: "
                f"{payload_size} > {chunk_size}"
            )

        payload = read_exact(
            port,
            payload_size,
        )

        actual_chunk_crc = zlib.crc32(payload) & 0xFFFFFFFF

        if actual_chunk_crc != expected_chunk_crc:
            raise ProtocolError(
                f"Ошибка CRC блока {chunk_index}: "
                f"{actual_chunk_crc:08X} != "
                f"{expected_chunk_crc:08X}"
            )

        image_data.extend(payload)

        received_percent = (
            (expected_index + 1) * 100.0 / chunk_count
        )

        print(
            f"\rПолучено блоков: "
            f"{expected_index + 1}/{chunk_count}, "
            f"{received_percent:5.1f}%",
            end="",
            flush=True,
        )

    print()

    raw_end_packet = read_exact(
        port,
        END_PACKET_SIZE,
    )

    (
        end_magic,
        end_image_id,
        end_image_crc,
    ) = struct.unpack(
        END_PACKET_FORMAT,
        raw_end_packet,
    )

    if end_magic != END_MAGIC:
        raise ProtocolError(
            f"Ожидалась сигнатура END1, получено {end_magic!r}"
        )

    if end_image_id != image_id:
        raise ProtocolError("В END1 находится неверный image ID")

    if end_image_crc != expected_image_crc:
        raise ProtocolError("CRC в END1 не совпадает с заголовком")

    if len(image_data) != image_size:
        raise ProtocolError(
            f"Размер не совпадает: принято {len(image_data)}, "
            f"ожидалось {image_size}"
        )

    actual_image_crc = zlib.crc32(image_data) & 0xFFFFFFFF

    if actual_image_crc != expected_image_crc:
        raise ProtocolError(
            f"Общий CRC изображения не совпадает: "
            f"{actual_image_crc:08X} != "
            f"{expected_image_crc:08X}"
        )

    metadata = {
        "image_id": image_id,
        "image_size": image_size,
        "width": width,
        "height": height,
        "crc32": actual_image_crc,
    }

    return bytes(image_data), metadata


def save_and_check_image(
    jpeg_data: bytes,
    image_id: int,
) -> Path:
    OUTPUT_DIRECTORY.mkdir(
        parents=True,
        exist_ok=True,
    )

    image_path = (
        OUTPUT_DIRECTORY /
        f"image_{image_id}.jpg"
    )

    image_path.write_bytes(jpeg_data)

    # Проверяем JPEG без изменения файла.
    with Image.open(io.BytesIO(jpeg_data)) as image:
        image.verify()

    # Повторно открываем для получения параметров.
    with Image.open(image_path) as image:
        print(f"Формат Pillow: {image.format}")
        print(f"Размер Pillow: {image.size}")
        print(f"Цветовой режим: {image.mode}")

    return image_path


def main() -> None:
    print("=" * 55)
    print("Приём изображения от ESP32-CAM")
    print(f"Порт: {PORT}")
    print(f"Скорость: {BAUD_RATE}")
    print(f"Папка результата: {OUTPUT_DIRECTORY}")
    print("=" * 55)

    try:
        with serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            timeout=0.2,
            write_timeout=5.0,
        ) as port:
            # Некоторые адаптеры используют эти линии
            # для автоматической перезагрузки.
            port.dtr = False
            port.rts = False

            print(f"Последовательный порт {PORT} открыт")

            # Открытие порта может перезагрузить ESP32.
            wait_for_esp32(port)

            print("Отправка команды GET...")

            port.write(b"GET\n")
            port.flush()

            jpeg_data, metadata = receive_image(port)

            print()
            print("Изображение полностью принято")
            print(f"Получено: {len(jpeg_data)} байт")
            print(f"CRC32: {metadata['crc32']:08X}")

            image_path = save_and_check_image(
                jpeg_data,
                metadata["image_id"],
            )

            print()
            print("УСПЕШНО")
            print(f"Файл сохранён здесь:")
            print(image_path.resolve())

    except serial.SerialException as error:
        print()
        print("ОШИБКА ПОСЛЕДОВАТЕЛЬНОГО ПОРТА")
        print(error)
        print()
        print("Проверьте:")
        print("1. Правильный ли номер COM-порта.")
        print("2. Закрыт ли Serial Monitor Arduino IDE.")
        print("3. Подключён ли USB-UART адаптер.")

    except TimeoutError as error:
        print()
        print("ТАЙМ-АУТ")
        print(error)
        print()
        print("ESP32 не начала передачу.")
        print("Проверьте вывод платы через Serial Monitor.")

    except ProtocolError as error:
        print()
        print("ОШИБКА ПРОТОКОЛА")
        print(error)

    except ModuleNotFoundError as error:
        print()
        print("НЕ УСТАНОВЛЕНА БИБЛИОТЕКА")
        print(error)
        print()
        print("Выполните:")
        print("python -m pip install pyserial pillow")

    except Exception as error:
        print()
        print("НЕОЖИДАННАЯ ОШИБКА")
        print(type(error).__name__)
        print(error)

    finally:
        print()
        input("Нажмите Enter для выхода...")


if __name__ == "__main__":
    main()