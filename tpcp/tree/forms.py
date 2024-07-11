from fileinput import FileInput
from django import forms
from django.forms import TextInput, FileInput
from tree.models import Tree, Log
from django.contrib.auth.forms import UserCreationForm, AuthenticationForm
from django.contrib.auth.models import User

class TreeForm(forms.ModelForm):
    class Meta:
        model = Tree
        fields = ('name','docfile')

        widgets = {
            'name': TextInput(attrs={
                'placeholder':'Введите название дерева',
                'id': 'result',
            }),
            'docfile': FileInput(attrs={
                'class': "form-control",
                'onchange': 'showFile(this)',
            })
        }

class RegisterUserForm(UserCreationForm):
    class Meta:
        model = User
        fields = ('username','email','password1', 'password2')
        widgets = {
            'username': forms.TextInput(attrs={'class': 'form-input', 'placeholder':'Введите имя'}),
            'password1': forms.EmailInput(attrs={'class': 'form-input', 'placeholder':'Введите email'}),
            'password1': forms.PasswordInput(attrs={'class': 'form-input', 'placeholder':'Введите пароль'}),
            'password2': forms.PasswordInput(attrs={'class': 'form-input', 'placeholder':'Повторите пароль'}),
        }


class LoginUserForm(AuthenticationForm):
    username = forms.CharField(label='Логин', widget=forms.TextInput(attrs={'class': 'form-input', 'placeholder':'Введите имя'}))
    password = forms.CharField(label='Пароль', widget=forms.PasswordInput(attrs={'class': 'form-input', 'placeholder':'Введите пароль'}))

      