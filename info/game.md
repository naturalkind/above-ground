### Игра
1. вид ссверху
2. возможность рисовать
3. по траектории переместить камеру получая снимки
4. выбор высоты

#### шпаргалка
https://epicgames.github.io/BlenderTools/send2ue/introduction/quickstart.html - проверить !!!   
https://dtf.ru/gamedev/1044579-besplatnyy-level-dizayn-ili-kak-stroit-landshaft-v-realnom-vremeni-na-ue4 - проверить !!!   
https://github.com/samorr/Computer-Vision-and-Photogrammetry   
https://blender.stackexchange.com/questions/271985/how-to-bake-texture-to-vertex-colors   


## Мой путь

### Первый тестовый запуск Blocks https://github.com/microsoft/AirSim/issues/4535
```
Engine/Binaries/ThirdParty/Mono/Linux/bin/mono  Engine/Binaries/DotNET/UnrealBuildTool.exe  Development Linux -Project=/home/sadko/Документы/'Unreal Projects'/above/above.uproject  -TargetType=Editor -Progress

```

Запуск unreal   
```
UnrealEngine-4.27.0-release/Engine/Binaries/Linux/UE4Editor
```

1. в blender импортировал .dae   
2. уменьшил полигоны в полученом, для лучшей работы   
3. https://www.youtube.com/watch?v=Rx-aOHCfTOw   

Способ 1, должен работать но не работает!
1. импортируя в Blender .dae формет agisoft photoscan, данные изображения это byte color(RGBA с 8-битными положительными целыми значениями) для вершин углов гряни.   
2. дальше следую этой инструкции https://www.youtube.com/watch?v=eq8718t7ZJk&t=184s, экпортирую из Blender в .fbx формат, должно все работать,
но не работает...   
3. unreal 4 отображает vert colors, но не отображать как метариал    
Изучаю в чем причина, и вот ответы:   
https://blenderartists.org/t/how-to-edit-color-attribute-type-from-vertex-color-to-face-corner-byte-color/1408417/8   
https://blender.stackexchange.com/questions/265945/i-cant-export-vertex-colors-anymore-in-blender-3-2   
Попробовал новую версию, тоже самое...   

Способ 2   
1. в blender импортировал .dae   
2. уменьшил полигоны, для увеличения быстродействия работы   
3. "запекаю" из вершин углов граней изображение по UV сетке, так как небыло исходного изображения для использования в качестве текстуры   
4. из текстуры создаю материал   
5. экспортирую в Unreal   
не работает, в этом случае пишут что проблема 4 версии unreal   
В итоге сохранил изображение из 3 шага и загрузил отдельно   


### Исправить не отображающиеся полигоны Blender (python)  

```
# получить обьект из сцены
bpy.context.active_object.select_set(False)
foto_obj = bpy.context.scene.objects[0]
bpy.context.view_layer.objects.active = foto_obj
foto_obj.select_set(True)

# Выбрать все грани
me = foto_obj.data
bpy.ops.object.mode_set(mode="EDIT")
bpy.ops.mesh.select_mode(type = 'EDGE')
bpy.ops.mesh.select_all(action = 'SELECT')

bpy.ops.mesh.beautify_fill()
bpy.ops.mesh.normals_tools(mode="RESET")
```

### найти самые крайние рёбра
```
import bpy
bpy.ops.object.editmode_toggle()
bpy.ops.mesh.select_similar(type='FACE', threshold=0.01)

https://blender.stackexchange.com/questions/106199/how-to-find-non-manifold-edges

```
