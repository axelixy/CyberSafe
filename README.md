# CyberSafe

## Структура

| Каталог | Назначение |
|---|---|
| `Bruteforce_PIN/esp32-bruteforce` | Аппаратный перебор PIN через энкодер + красивое видео|
| `Bruteforce_PIN/esp32-bruteforce-timebased` | Timing атака |
| `esp32-voltage-glitching` | Попытка fault injection / voltage glitching |
| `Decrypt_Disk` | Скрипты для работы с образом диска |
| `Dump_Image` | Дамп памяти устройства |
| `Path_Image` | Запатченные образы |


## Сборка ESP-IDF

Для каждого проекта ESP32:

```bash
cd Bruteforce_PIN/esp32-bruteforce-timebased
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

## Классификация атак

Все атаки преведены в презентации (не успеваю)
