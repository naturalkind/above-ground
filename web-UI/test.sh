#!/bin/bash

# Запускаем скрипты Python
python3 server_2.py &

# Создаем новую вкладку в окне терминала
gnome-terminal --tab -e "bash -c 'python3 client.py'"

# Функция для безопасного завершения скриптов при нажатии Ctrl+C
trap 'kill $(jobs -p)' SIGINT

# Ожидаем завершения выполнения всех скриптов
wait
