#!/bin/bash
cd /home/orangepi/above-ground
# Активируем виртуальное окружение Python
source venv/bin/activate
export TERM=xterm-256color
# Запускаем программу вариант 1
python auto_aim.py

# вариант 2 
#Xvfb :3 -ac +extension DPMS -screen 0 1200x700x8 &
#export DISPLAY=:3
#sleep 3
#xterm -geometry 1200x1200 -bg black -fg green -fa 'Monospace' -fs 12 -e 'python /home/orangepi/above-ground/auto_aim.py' &
#x11vnc -display :3 -forever -nopw -quiet

# вариант 3
