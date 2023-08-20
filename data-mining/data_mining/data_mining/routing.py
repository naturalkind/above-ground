from django.urls import path, re_path
from main_channels.main_channels import MainHandler

websocket_urlpatterns = [
    path(r'', MainHandler.as_asgi()),
]
