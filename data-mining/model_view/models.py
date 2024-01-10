#-*- coding: utf-8 -*-
from django.db import models
from django.template.defaultfilters import slugify

from django.db.models.signals import post_save

from django.conf import settings
from django.contrib.auth.models import AbstractUser
from django.db.models import JSONField
from datetime import datetime, timedelta
from django.core.cache import cache 
import base64
import re
import uuid, os

class User(AbstractUser):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False, unique=True)
    path_data = models.TextField(max_length=200, default="", verbose_name='Название каталога', blank=True)
    
    class Meta(AbstractUser.Meta):
        swappable = 'AUTH_USER_MODEL'
        
    def __str__(self) -> str:
        return self.username
            
    def natural_key(self):
        return (self.image_user, self.path_data, self.id, self.username)
    
    def save(self, *args, **kwargs):
        if self.path_data == "":
            self.path_data = str(self.id)[:12]
            if not os.path.exists(f"media/data_image/{self.path_data}"):
                os.makedirs(f"media/data_image/{self.path_data}")
        return super(User, self).save(*args, **kwargs)

class ClassData(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False, unique=True)
    user_class = models.ForeignKey(settings.AUTH_USER_MODEL, related_name='us_class', default="", on_delete=models.CASCADE)
    title = models.CharField(max_length=999999, default="", verbose_name='Загаловок', blank=True)

class Post(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False, unique=True)
    pure_data = JSONField()
    title = models.CharField(max_length=999999, default="", verbose_name='Загаловок', blank=True)
    text = models.TextField(max_length=999999, default="", verbose_name='Текст', blank=True)
    image = models.TextField(max_length=1000, default="", verbose_name='Название картинки', blank=True)
    path_data = models.TextField(max_length=9999, default="", verbose_name='Расположение', blank=True)
    date_post = models.DateTimeField(auto_now_add=True, verbose_name='Дата создания')
    user_post = models.ForeignKey(settings.AUTH_USER_MODEL, related_name='us_post', default="", on_delete=models.CASCADE)
    #class_data = models.ForeignKey(ClassData, default="", on_delete=models.CASCADE)

    
    def __str__(self) -> str:
        return self.text

    def __unicode__(self):
            return u'name: %s , id: %s' % (self.title, self.id)

