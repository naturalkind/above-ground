"""
Django settings for data_mining project.

Generated by 'django-admin startproject' using Django 4.2.4.

For more information on this file, see
https://docs.djangoproject.com/en/4.2/topics/settings/

For the full list of settings and their values, see
https://docs.djangoproject.com/en/4.2/ref/settings/
"""

from pathlib import Path

# Build paths inside the project like this: BASE_DIR / 'subdir'.
BASE_DIR = Path(__file__).resolve().parent.parent


# Quick-start development settings - unsuitable for production
# See https://docs.djangoproject.com/en/4.2/howto/deployment/checklist/

# SECURITY WARNING: keep the secret key used in production secret!
SECRET_KEY = 'django-insecure-^!xv@lem4my5bz$#0u=eu-7c*0l6a6_tqa*-1q!!ggp2hvqu0m'

# SECURITY WARNING: don't run with debug turned on in production!
DEBUG = True

ALLOWED_HOSTS =  ['127.0.0.1', 'localhost', '178.158.131.41', '192.168.1.50']


# Application definition

INSTALLED_APPS = [
    'daphne',
    'channels',
    'django.contrib.admin',
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.sessions',
    'django.contrib.messages',
    'django.contrib.staticfiles',
    'main_channels',
    'model_view',
]

AUTH_USER_MODEL = 'model_view.User'

MIDDLEWARE = [
    'django.middleware.security.SecurityMiddleware',
    'django.contrib.sessions.middleware.SessionMiddleware',
    'django.middleware.common.CommonMiddleware',
    'django.middleware.csrf.CsrfViewMiddleware',
    'django.contrib.auth.middleware.AuthenticationMiddleware',
    'django.contrib.messages.middleware.MessageMiddleware',
    'django.middleware.clickjacking.XFrameOptionsMiddleware',
]

ROOT_URLCONF = 'data_mining.urls'

TEMPLATES = [
    {
        'BACKEND': 'django.template.backends.django.DjangoTemplates',
        'DIRS': ['templates/',],
        'APP_DIRS': True,
        'OPTIONS': {
            'context_processors': [
                'django.template.context_processors.debug',
                'django.template.context_processors.request',
                'django.contrib.auth.context_processors.auth',
                'django.contrib.messages.context_processors.messages',
            ],
        },
    },
]

WSGI_APPLICATION = 'data_mining.wsgi.application'
ASGI_APPLICATION = 'data_mining.asgi.application'

#https://github.com/django/channels_redis/issues/332
CHANNEL_LAYERS = {
    "default": {
        "BACKEND": "channels_redis.pubsub.RedisPubSubChannelLayer",
        "CONFIG": {
            "hosts": ["redis://localhost:6379/4"],
        },
    },
}

# Database
# https://docs.djangoproject.com/en/4.2/ref/settings/#databases

DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.sqlite3',
        'NAME': BASE_DIR / 'db.sqlite3',
    }
}


# Password validation
# https://docs.djangoproject.com/en/4.2/ref/settings/#auth-password-validators

AUTH_PASSWORD_VALIDATORS = [
    {
        'NAME': 'django.contrib.auth.password_validation.UserAttributeSimilarityValidator',
    },
    {
        'NAME': 'django.contrib.auth.password_validation.MinimumLengthValidator',
    },
    {
        'NAME': 'django.contrib.auth.password_validation.CommonPasswordValidator',
    },
    {
        'NAME': 'django.contrib.auth.password_validation.NumericPasswordValidator',
    },
]


# Internationalization
# https://docs.djangoproject.com/en/4.2/topics/i18n/

LANGUAGE_CODE = 'en-us'

TIME_ZONE = 'UTC'

USE_I18N = True

USE_TZ = True


# Static files (CSS, JavaScript, Images)
# https://docs.djangoproject.com/en/4.2/howto/static-files/

STATIC_URL = 'static/'
MEDIA_URL = 'media/'

# Default primary key field type
# https://docs.djangoproject.com/en/4.2/ref/settings/#default-auto-field

DEFAULT_AUTO_FIELD = 'django.db.models.BigAutoField'

DATA_UPLOAD_MAX_MEMORY_SIZE = 31457280
DJANGO_ALLOW_ASYNC_UNSAFE = "true"


#база данных:
#Таблица 1
#    id
#    текст 1 (длинный текст) | title
#    текст 2 (длинный текст) | text
#    json поле               | pure_data
#    поле изображения 
#    связь с пользователем

#Таблица 2
#    id class_id
#    текст (длинный текст)   | class_name
#    связь с пользователем

#- желательно работа по websocket
#- функционал отображать только зарегестрированным пользователям
#- отображать две таблицы в admin панеле.

#Клиентская часть одна страница с функционалом:
#    Основноей функционал:
#        1 загрузка изображения
#        2 ограничивающим прямоугольником выделить нужный класса в изобажении
#        3 маркировка класса
#        4 сохранение класса и прямоугольник в сервере
#    Дополнительный:
#        1 отображения всех прямогольников
#        2 возможность редактировать прямоугольник



#Вид данных отправляемых в сервер:
#{
#    pure_data:[{x0:x0, y0:y0, x1:x1, y1:y1, class_name:class_name, class_id:class_id},
#               {x0:x0, y0:y0, x1:x1, y1:y1, class_name:class_name, class_id:class_id},
#               ...],
#    title : "",
#    text: "",
#    name_image: SelectedFile,
#    image: data_image,
#    os_info: window.navigator.userAgent,
#    event: "mainpost",
#}

