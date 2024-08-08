#!/bin/bash

cd /home/orangepi/above-ground

# Активируем виртуальное окружение Python
source venv/bin/activate

# Запускаем программу
python auto_aim.py


# Вариант 2 

#Xvfb :3 -ac -screen 0 1200x1200x24 & # -ac +extension DPMS
#export TERM=xterm-256color
#export DISPLAY=:3
#sleep 3
#xterm -geometry 1200x1200 -e 'python /home/orangepi/above-ground/auto_aim.py' &
#x11vnc -display :3 -forever -nopw -quiet

########### 
# screen -S python_curses_app -d -m python /home/orangepi/above-ground/auto_aim.py
# export TERM=xterm-256color
# export TERM=linux
# export TERMINFO=/lib/terminfo

















