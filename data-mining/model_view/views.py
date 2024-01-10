from django.shortcuts import render, redirect, HttpResponseRedirect
from django.http import HttpResponse, Http404, JsonResponse
try:
    from django.utils import simplejson as json
except ImportError:
    import json
from django.template.context_processors import csrf    
from django.contrib import auth

from model_view.models import *

from django.contrib.auth.decorators import login_required
from django.core.paginator import Paginator, PageNotAnInteger, EmptyPage
from django.core import serializers
import requests
import base64
import re
import os
import uuid
from PIL import Image
from datetime import datetime
from django.core.exceptions import ObjectDoesNotExist
from django.contrib.auth.forms import UserCreationForm 

from django.template.loader import render_to_string
from django.template import loader
from django.template import Template, Context
from django.views.decorators.cache import cache_page



def main_page(request):
    if request.user.is_authenticated:
        if request.method == 'GET':
            return index(request)
        if request.method == 'POST':
            pass
    else:
        return HttpResponse(json.dumps({'error':'error authenticated'}), content_type = "application/json")
        
def index(request):
    class_data = ClassData.objects.all()
    return render(request, 'index.html',{'class_data': class_data, 'user': auth.get_user(request)})        
