### Установить NODE

wget https://nodejs.org/dist/v16.15.0/node-v16.15.0-linux-arm64.tar.xz

tar -xf node-v16.15.0-linux-arm64.tar.xz

cd node-v16.15.0-linux-arm64

sudo cp -R * /usr/local/

### Проверить 

node -v

npm -v

### Установить nwjs ARM

sudo mkdir -p /usr/local/lib/nwjs

wget https://github.com/LeonardLaszlo/nw.js-armv7-binaries/releases/download/nw60-arm64_2022-01-08/nw60-arm64_2022-01-08.tar.gz

mkdir nw60-arm64_2022-01-08

tar -xvzf nw60-arm64_2022-01-08.tar.gz -C nw60-arm64_2022-01-08

cd nw60-arm64_2022-01-08/usr/docker/dist/nwjs-chrome-ffmpeg-branding/

tar -xf nwjs-v0.60.1-linux-arm64.tar.gz

cd nwjs-v0.60.1-linux-arm64

sudo cp -R . /usr/local/lib/nwjs/

nano ~/.bashrc

"""
# add nwjs to the path
export NWJS_HOME=/usr/local/lib/nwjs
export PATH=$NWJS_HOME:$PATH
"""

source ~/.bashrc

тестовый запуск 

nw

### Установка Betaflight-configurator
качаем любым способом betaflight-configurator-10.8.0
можно клонировать репозиторий и переключиться на эту версию
скачать вручную или wget
разархивировать
wget https://github.com/betaflight/betaflight-configurator/archive/refs/tags/10.8.0.tar.gz

tar -xf 10.8.0.tar.gz

cd betaflight-configurator-10.8.0/

и запустить один:
v1:
sudo npm install yarn -g
yarn install
yarn gulp dist

v2:
npm install
npm install --save-dev run-script-os
npm start

Для удобства запуска создаём ярлык
cd ~/.local/share/applications
nano betaflight-configurator.desktop

"""
[Desktop Entry]
Type=Application

Name=BF Configurator

Path=/home/orangepi/betaflight-configurator/debug/betaflight-configurator/linux64

Terminal=false

Exec=/usr/local/lib/nwjs/nw .

Icon=/home/orangepi/betaflight-configurator/debug/betaflight-configurator/linux64/icon/bf_icon_128.png

Categories=Utility

"""

ВАЖНО!!!
/home/orangepi/betaflight-configurator/ - это путь к вашей папке betaflight

Проверка на ошибки!
desktop-file-validate betaflight-configurator.desktop 

В точности по этой инструкции у меня не заработало
https://www.gandytech.co.uk/blog/betaflight-configurator-on-manjaro-linux-arm64/
