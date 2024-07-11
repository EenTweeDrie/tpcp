from django.db import models
from django.urls import reverse

# Create your models here.

class Tree(models.Model):
    name = models.CharField(max_length=200, help_text="Название", null=False)
    docfile = models.FileField(upload_to='trees/%Y/%m/%d')
    tree_length = models.FloatField('Длина дерева',null=True, default = 0)
    tree_height = models.FloatField('Высота дерева',null=True, default = 0)
    tree_diameter = models.FloatField('Диаметр дерева',null=True, default = 0)
    tree_diameter_hyper = models.FloatField('Диаметр дерева Hyper',null=True, default = 0)
    tree_crown_height = models.FloatField('Высота кроны',null=True, default = 0)
    tree_crown_volume = models.FloatField('Объем кроны',null=True, default = 0)
    tree_crown_square = models.FloatField('Площадь кроны',null=True, default = 0)
    tree_img = models.ImageField('Изображение',upload_to='images/', blank=True, null=True, help_text="Загрузите изображение")
    tree_plot = models.TextField(help_text="3d",null=True)
    tree_eps = models.FloatField('eps',null=True, default = 0.35)
    tree_minpts = models.IntegerField('minpts',null=True, default = 100)
    tree_iter = models.IntegerField('iter',null=True, default = 1)
    tree_onnormalsz = models.BooleanField('onnormalsz',null=False, default = 1)
    
    def __str__(self):
        return self.name
    
    def get_absolute_url(self):
        return reverse('tree-detail', args=[str(self.id)])
        
    @property
    def tree_img_url(self):
        if self.tree_img and hasattr(self.tree_img, 'url'):
            return self.tree_img.url

    class Meta:
        verbose_name = 'Дерево'
        verbose_name_plural = 'Деревья'

class Log(models.Model):
    log_name = models.CharField(max_length = 200, null=True)
    logfile = models.FileField(upload_to='logs',null=True)
