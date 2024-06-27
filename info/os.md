#### Установка Ubuntu на orange pi 5
0. скачать образ ubuntu
1. установить на sd образ ubuntu
2. загрузиться с sd
3. запустить orangepi-config
   - system-hardware
   - находим в списке ssd-sata, и ставим галочку 
   - сохраняем, перезагружаем 
4. `sudo dd if=/usr/share/orangepi5/rkspi_loader_sata.img of =/dev/mtdblock0`
   - перезагружаем
5. проверяем видимость накопителя в системе
6. установленный образ ubuntu #0 помещаем в корневой раздел (флешка или качаем по новой)
7. записываем образ #0 в ssd с помощью balenaEtcher
8. выключаем
9. запускаем как только начинает моргать зеленым вытаскиваем cd, и дальше грузимся уже с ssd
10. `overlays=ssd-sata` добавляем в файл `boot/orangepiEnv.txt`
11. выполняем пункт 3, 3.1, 3.2, 3.3
12. выключаем
13. грузимся с ssd без проблем
тестировал `Orangepi5_1.1.6_ubuntu_jammy_desktop_gnome_linux5.10.110`


#### Запустить DeepSORT c++/python
https://github.innominds.com/shaoshengsong/DeepSORT

1. установить opencv `sudo pip install opencv-contrib-python==4.8.1.78` 
   - установить скомпилировать onnxruntime
2. установить opencv arm: 
   - https://www.programmersought.com/article/254710007175/
   - https://github.com/huzz/OpenCV-aarch64


инструкция https://github.com/Qengineering/Rock-5-image
установить `git clone git@github.com:rockchip-linux/rknn-toolkit2.git`

Компелировать opencv для работы с npu
https://github.com/opencv/opencv/wiki/TIM-VX-Backend-For-Running-OpenCV-On-NPU
    ``` 
    git clone -b 4.8.1 https://github.com/opencv/opencv_contrib.git
    git clone -b 4.8.1 https://github.com/opencv/opencv.git
    cmake -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_ENABLE_NONFREE=ON -D CMAKE_BUILD_TYPE=RELEASE -D ENABLE_NEON=ON -D ENABLE_TBB=ON -D ENABLE_IPP=ON -D ENABLE_VFVP3=ON -D WITH_OPENMP=ON -D WITH_CSTRIPES=ON -D WITH_OPENCL=ON -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/home/orange/opencv_contrib/modules/ ..
    make -j8
    sudo make install
```
> [!NOTE]
> удаление: https://stackoverflow.com/questions/13134151/how-to-uninstall-opencv-in-ubuntu 

#### Настройка Hostspot

github: `https://gist.github.com/narate/d3f001c97e1c981a59f94cd76f041140`

    ```
    nmcli con add type wifi ifname wlan0 con-name Hostspot autoconnect yes ssid Hostspot
    nmcli con modify Hostspot 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
    nmcli con modify Hostspot wifi-sec.key-mgmt wpa-psk
    nmcli con modify Hostspot wifi-sec.psk "PASSWORD_CREATE"
    nmcli con up Hostspot

    nmcli con show
    nmcli connection delete id <connection name>
    nmcli connection delete <connection name>
    ```

#### Установка Betaflight ARM

1. Установить NODE
    ```
    wget https://nodejs.org/dist/v16.15.0/node-v16.15.0-linux-arm64.tar.xz

    tar -xf node-v16.15.0-linux-arm64.tar.xz

    cd node-v16.15.0-linux-arm64

    sudo cp -R * /usr/local/
    
    node -v

    npm -v
    ```

3. Установить nwjs ARM
    ```
    sudo mkdir -p /usr/local/lib/nwjs

    wget https://github.com/LeonardLaszlo/nw.js-armv7-binaries/releases/download/nw60-arm64_2022-01-08/nw60-arm64_2022-01-08.tar.gz

    mkdir nw60-arm64_2022-01-08

    tar -xvzf nw60-arm64_2022-01-08.tar.gz -C nw60-arm64_2022-01-08

    cd nw60-arm64_2022-01-08/usr/docker/dist/nwjs-chrome-ffmpeg-branding/

    tar -xf nwjs-v0.60.1-linux-arm64.tar.gz

    cd nwjs-v0.60.1-linux-arm64

    sudo cp -R . /usr/local/lib/nwjs/

    nano ~/.bashrc

    # add nwjs to the path
    export NWJS_HOME=/usr/local/lib/nwjs
    export PATH=$NWJS_HOME:$PATH

    source ~/.bashrc

    #тестовый запуск 
    nw
    ```

4. Установка Betaflight-configurator. Качаем любым способом betaflight-configurator-10.8.0
можно клонировать репозиторий и переключиться на эту версию/скачать вручную/wget.
Разархивировать.
    ```
    wget https://github.com/betaflight/betaflight-configurator/archive/refs/tags/10.8.0.tar.gz

    tar -xf 10.8.0.tar.gz

    cd betaflight-configurator-10.8.0/

    # v1:
    sudo npm install yarn -g
    yarn install
    yarn gulp dist

    # v2:
    npm install
    npm install --save-dev run-script-os
    npm start
    ```

5. Для удобства запуска создаём ярлык
    ```
    cd ~/.local/share/applications
    nano betaflight-configurator.desktop
    ```

    ```
    [Desktop Entry]
    Type=Application

    Name=BF Configurator

    Path=/home/orangepi/betaflight-configurator/debug/betaflight-configurator/linux64

    Terminal=false

    Exec=/usr/local/lib/nwjs/nw .

    Icon=/home/orangepi/betaflight-configurator/debug/betaflight-configurator/linux64/icon/bf_icon_128.png

    Categories=Utility
    ```

> [!CAUTION]
> `/home/orangepi/betaflight-configurator/` - это путь к вашей папке betaflight

Проверка на ошибки!
    ```
    desktop-file-validate betaflight-configurator.desktop 
    ```

> В точности по этой инструкции у меня не заработало `https://www.gandytech.co.uk/blog/betaflight-configurator-on-manjaro-linux-arm64/`

#### Автозапуск 
1. 
    ```
    sudo apt install x11vnc
    sudo apt install xvfb
    sudo apt install xterm
    ```
2. `nano reload_aim.sh`
    ```
    sudo systemctl stop aim.service
    sudo systemctl start aim.service
    sleep 3
    sudo systemctl status aim.service
    ```
3. `sudo nano /etc/systemd/system/aim.service`
    ```
    [Unit]
    Description=Python Curses App
    After=graphical.target

    [Service]
    Type=forking
    User=orangepi
    Environment="DISPLAY:3"
    Environment="TERM=xterm-256color"
    ExecStart=screen -dmS python_curses_app /home/orangepi/above-ground/auto_run.sh
    ExecStop=screen -S python_curses_app -X quit
    Restart=on-failure

    [Install]
    WantedBy=graphical.target
```
4. `nano /home/orangepi/above-ground/auto_run.sh`
    ```
    #!/bin/bash

    cd /home/orangepi/above-ground

    # Активируем виртуальное окружение Python
    source venv/bin/activate
    Xvfb :3 -ac -screen 0 1200x1200x24 &
    export TERM=xterm-256color
    export DISPLAY=:3
    sleep 3
    xterm -geometry 1200x1200 -e "python /home/orangepi/above-ground/auto_aim.py" &
    x11vnc -display :3 -forever -nopw -quiet

    ```
5. 
    ```
    sudo systemctl enable aim.service
    sudo systemctl daemon-reload
    sudo chmod 777 reload_aim.sh
    ./reload_aim.sh
    screen -r python_curses_app
    ```
