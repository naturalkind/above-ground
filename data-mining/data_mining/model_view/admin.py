from django.contrib import admin
from model_view.models import *
from django.contrib.auth.admin import UserAdmin

class PostAdmin(admin.ModelAdmin):
    list_display = ('id', 'title')
    search_fields = ('title', 'id',)
    fields = ('title', 'text', 'pure_data', 'image', 'user_post')  
    
class ClassAdmin(admin.ModelAdmin):
    list_display = ('user_class', 'title')
    fields = ('user_class', 'title')   

class CustomUserAdmin(UserAdmin):
    list_display = ['username', 'is_superuser', 'id', 'last_login', 'path_data']
    fieldsets = UserAdmin.fieldsets + ((None, {'fields': ('path_data',)}),)


admin.site.register(User, CustomUserAdmin)
admin.site.register(Post, PostAdmin)   
admin.site.register(ClassData, ClassAdmin)
