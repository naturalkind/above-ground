# above-ground

### Первый эксперимент с airsim
Создаём путь на карте, для каждого отрезка пути у нас есть изображение. Агент получает текущее изображение и сравнивать   
с изображениями заданного отрезка, пытаясь как можно точней приблизить свою траекторию к прочерченной траектории.   
Задачу состоит из цепочки задач:   
- загрузить фотограметрию поверхности в игровую площадку   
- получать снимки с высоты полета по траектории   
- научить дрон летать по траектории из снимков   

## Шпаргалка
### Первый тестовый запуск Blocks https://github.com/microsoft/AirSim/issues/4535
Запуск unreal   
```
UnrealEngine-4.27.0-release/Engine/Binaries/Linux/UE4Editor
```
### Мой путь
1. в blender импортировал .dae   
2. уменьшил полигоны в полученом, для лучшей работы   
3. https://www.youtube.com/watch?v=Rx-aOHCfTOw   

Способ 1, должен работать но не работает!
1 импортируя в Blender .dae формет agisoft photoscan, данные изображения это byte color(RGBA с 8-битными положительными целыми значениями) для вершин углов гряни.
2 дальше следую этой инструкции https://www.youtube.com/watch?v=eq8718t7ZJk&t=184s, экпортирую из Blender в .fbx формат, должно все работать,
но не работает...
3 unreal 4 отображает vert colors, но не отображать как метариал
Изучаю в чем причина, и вот ответы:
https://blenderartists.org/t/how-to-edit-color-attribute-type-from-vertex-color-to-face-corner-byte-color/1408417/8
https://blender.stackexchange.com/questions/265945/i-cant-export-vertex-colors-anymore-in-blender-3-2
Попробовал новую версию, тоже самое...

Способ 2
1 в blender импортировал .dae   
2 уменьшил полигоны, для увеличения быстродействия работы   
3 "запекаю" из вершин углов граней изображение по UV сетке, так как небыло исходного изображения для использования в качестве текстуры
4 из текстуры создаю материал
5 экспортирую в Unreal
не работает, в этом случае пишут что проблема 4 версии unreal
В итоге сохранил изображение из 3 шага и загрузил отдельно



### шпаргалка
https://epicgames.github.io/BlenderTools/send2ue/introduction/quickstart.html - проверить !!!   
https://dtf.ru/gamedev/1044579-besplatnyy-level-dizayn-ili-kak-stroit-landshaft-v-realnom-vremeni-na-ue4 - проверить !!!   
https://github.com/samorr/Computer-Vision-and-Photogrammetry   
https://blender.stackexchange.com/questions/271985/how-to-bake-texture-to-vertex-colors   
https://blender.stackexchange.com/questions/106199/how-to-find-non-manifold-edges   

### найти самые крайние рёбра
```
import bpy
bpy.ops.object.editmode_toggle()
bpy.ops.mesh.select_similar(type='FACE', threshold=0.01)

https://blender.stackexchange.com/questions/106199/how-to-find-non-manifold-edges

```

### Видео уроки   
https://www.youtube.com/watch?v=g7XKX3bm5ak&t=396s   
https://vimeo.com/156786446   
https://www.youtube.com/watch?v=dMUnKmz7u6s   
https://www.youtube.com/watch?v=VtS9jS6X4xQ   
https://www.youtube.com/watch?v=Rx-aOHCfTOw   
https://www.youtube.com/watch?v=Mn8_Yfor4TA   
https://www.youtube.com/watch?v=z5gYPaHSxoA   
https://www.youtube.com/watch?v=UXD97l7ZT0w   
https://www.youtube.com/watch?v=eq8718t7ZJk&t=184s -должно все работать   
https://www.youtube.com/watch?v=00gF5o_aVn0&t=59s - проблема отображения нормалей   

### Для понимания
gui для создания данных, и будущая игровая площадка для бота   
```
python3 main.py
```

склеить изображение   
```
python3 glue_part.py
```
+ [Собранная информацияь по теме](https://github.com/naturalkind/above-ground/blob/main/info/info.md)

![Иллюстрация к проекту](https://github.com/naturalkind/above-ground/blob/main/media-info/%D0%A1%D0%BB%D0%B0%D0%B9%D0%B4%201.png)

### Нужно сделать
- [x] нарезать разного размера изображение, из снимка местности   
- [ ] вносить искажения   
- [x] использовать дискрипторы для воссоздания изображения   
- [ ] обучать нейронную сеть   
- [x] графф для сортировки изображений в нужной последовательности   
- [x] установить gazebo   
- [x] установить airsim   
- [ ] подключить к симуляторам px4/контроллер   
- [ ] получать сигналы контроллера https://github.com/Microsoft/AirSim/issues/1726   
- [x] изучить возможность применения blender   


