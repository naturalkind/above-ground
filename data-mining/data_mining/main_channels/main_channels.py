from importlib import import_module

from channels.generic.websocket import AsyncJsonWebsocketConsumer

from django.core.cache import cache
from django.conf import settings
from django.utils import dateformat
from asgiref.sync import sync_to_async, async_to_sync
from model_view.models import *

import json
import datetime
import uuid
import base64, io, os, re

session_engine = import_module(settings.SESSION_ENGINE)



class MainHandler(AsyncJsonWebsocketConsumer):
    async def connect(self):
        self.room_group_name = "wall"
        self.sender_id = self.scope['user'].id
        self.sender_name = self.scope['user']
        if str(self.scope['user']) != 'AnonymousUser':
            pass
        print ("CHANNEL_LAYERS", self.channel_name, self.room_group_name, self.scope['user'])
        await self.channel_layer.group_add(
            self.room_group_name,
            self.channel_name
        )
        await self.accept()

    async def disconnect(self, close_code):
        print("Disconnected", close_code)
        # Leave room group
        await self.channel_layer.group_discard(
            self.room_group_name,
            self.channel_name
        )
    async def receive(self, text_data):
        """
        Receive message from WebSocket.
        Get the event and send the appropriate event
        """
        
        response = json.loads(text_data)
        event = response.get("event", None)
        if event == "mainpost":
            if self.scope['user'].is_authenticated:  
#                print (response)
                # сохранение изображения
                self.namefile = f'{str(uuid.uuid4())[:12]}_{response["name_image"]}'
                img_file = open(f"media/data_image/{self.scope['user'].path_data}/{self.namefile}", "wb")
                imgstr = re.search(r'base64,(.*)', response['image']).group(1)
                img_file.write(base64.b64decode(imgstr))
                img_file.close()
                print (self.namefile, response["title"])
                # сохранение в базу данных
                post = Post()
                post.title = response["title"]
                post.text = response["text"]
                post.pure_data = response["arr_coord"]
                post.image = self.namefile
                post.path_data = self.scope['user'].path_data
                post.user_post = self.scope['user']
                post_async = sync_to_async(post.save)
                await post_async()                
#                    _data = {"type": "mainpost"}
#    #                _data={
#    #                        "type": "wallpost",
#    #                        "comment_text": response["comment_text"],
#    #                        "comment_image": comment_image,
#    #                        "comment_user": self.scope['user'].username,
#    #                        "comment_id":str(comment.id),
#    #                        "path_data": self.path_data,
#    #                        "image_user": self.image_user,
#    #                        "post_id": response["post_id"],
#    #                        "user_id": str(self.sender_id),
#    #                        "timecomment":now,
#    #                        "status" : "send_comment"
#    #                    }
#                    await self.channel_layer.group_send(self.room_group_name, _data)
                      

    async def mainpost(self, res):
        """ Receive message from room group """
        # Send message to WebSocket
        print ("WALLPOST", res)
        await self.send(text_data=json.dumps(res))

