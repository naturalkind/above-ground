#!/bin/bash

# Запускаем скрипты Python
python3 genetic_pid.py &

echo "Sleeping for 3 seconds…"
sleep 3
echo "Completed"

# Создаем новую вкладку в окне терминала
gnome-terminal --tab -e "bash -c 'python3 client.py'"

# Функция для безопасного завершения скриптов при нажатии Ctrl+C
trap 'kill $(jobs -p)' SIGINT

# Ожидаем завершения выполнения всех скриптов
wait
