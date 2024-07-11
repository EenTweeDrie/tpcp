from django.urls import path
from . import views


urlpatterns = [
    path('', views.index, name='index'),
    path('login', views.login_or_reg, name='login'),
    path('registration', views.login_or_reg, name='registration'),
    # path('processing', views.processing, name='processing'),
    path('upload', views.upload, name='upload'),
    path('logout', views.logout_view, name='logout'),
    path('list/', views.TreeListView.as_view(), name='tree_list'),
    path('list/<int:id>',views.treedetail, name='tree-detail'),
    path('list/treeprocessing/<int:id>/', views.treeprocessing, name='tree-processing'),
    path('download', views.createlog, name='download'),
]