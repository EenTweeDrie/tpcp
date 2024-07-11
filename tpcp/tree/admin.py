from django.apps import apps
from django.contrib import admin
from .models import Tree, Log


# for model in apps.get_app_config('tree').get_models():
#     admin.site.register(model)

# Define the admin class
class TreeAdmin(admin.ModelAdmin):
    list_display = ('name', 'docfile', 'tree_length', 'tree_height', 'tree_diameter','tree_diameter_hyper','tree_crown_height','tree_crown_volume', 'tree_img', 'tree_eps', 'tree_minpts', 'tree_iter', 'tree_onnormalsz')
    fields = ['name', 'docfile', 'tree_length', 'tree_height', 'tree_diameter','tree_diameter_hyper','tree_crown_height','tree_crown_volume', 'tree_img', 'tree_eps', 'tree_minpts', 'tree_iter', 'tree_onnormalsz']

# Register the admin class with the associated model
admin.site.register(Tree, TreeAdmin)

class LogAdmin(admin.ModelAdmin):
    list_display = ('log_name', 'logfile')
    fields = ['log_name', 'logfile']

admin.site.register(Log, LogAdmin)