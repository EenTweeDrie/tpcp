# TPCP - Trees point cloud processing

## Описание
Разработанная программа позволит проводить комплексную инвентаризацию лесного фонда и получать наиболее полное 
представление о лесных насаждениях. 

Программа позволяет обрабатывать загружаемые в качестве входных данных файлы 
плотных облаков точек деревьев **(LiDAR data)** с целью определения ряда таксационных параметров дерева. 

Процесс определения происходит с помощью нескольких методов машинного обучения и других алгоритмов, среди которых методы кластеризации на основе 
плотности (DBSCAN, OPTICS), аппроксимация окружностей и вычисление минимальной выпуклой оболочки.

## Возможности
- **Регистрация и авторизация.**
- **Добавление и просмотр файлов деревьев.**
- **Расчет таксационных параметров.**
- **Сегментация облака точек:**
  - **_земля,_**
  - **_ствол,_**
  - **_крона (ветки и листья)._**
- **Визуализация и просмотр результатов.**
- **Изменение гиперпараметров.**
- **Формирование отчетов.**


## Демонстрация
### Регистрация и авторизация
<p align="center">
  <img src="about\reg.png">
</p>

### Загрузка файлов
<p align="center">
  <img src="about\upload.png">
</p>

### Список файлов
<p align="center">
  <img src="about\list.png">
</p>

### Визуализация файла
<p align="center">
  <img src="about\rotate.gif">
</p>

### Изменение гиперпараметров
<p align="center">
  <img src="about\hyperparam.png">
</p>

### Формирование отчетов
<p align="center">
  <img src="about\report.png">
</p>

## Таксационные параметры дерева

<p align="center">
  <img src="about\taxparam.png">
</p>

- Высота (H) дерева 
- Длина (L) дерева 
- Диаметр (DBH) дерева 
- Сбег ствола (ST) дерева 
- Объем и площадь выпуклой оболочки кроны (ACCH, VCCH) 
- Высота до основания кроны (HCB) 
- Высота кроны (CH)

##  TPCP vs  [3DForest](https://www.3dforest.eu/)

### Примеры расчетов и визуализации

![Taxation parameters of the tree](about\comparing.png)

|                  | TPCP | 3DForest     |
|------------------|------|--------------|
| Диаметр дерева   | 29,4 см | 30,4 см      |
| Высота дерева    | 18,33 м | 18,33 м      |
| Длина дерева     | 18,76 м | 18,76 м      |
| Высота кроны     | 16,28 м | 17,13 м      |
| Объем оболочки кроны | 329,491 куб. м | 326,410 куб. м |
| Площадь оболочки кроны | 283,822 кв. м | 286,446 кв. м |


### Ошибки программ относительно полевых данных (86 деревьев в выборке)
<table>
  <thead>
    <tr>
      <th></th>
      <th colspan="4">Ошибки TPCP</th>
      <th colspan="4">Ошибки 3DForest</th>
    </tr>
    <tr>
      <th></th>
      <th>DBH LS</th>
      <th>DBH HLS</th>
      <th>Height</th>
      <th>HCB</th>
      <th>DBH LS</th>
      <th>DBH RHT</th>
      <th>Height</th>
      <th>HCB</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>RMSE</td>
      <td>0,83</td>
      <td>0,76</td>
      <td>1,68</td>
      <td>1,81</td>
      <td>0,74</td>
      <td>1,09</td>
      <td>1,70</td>
      <td>2,06</td>
    </tr>
    <tr>
      <td>Ошибка, %</td>
      <td>-1,75</td>
      <td>-1,26</td>
      <td>0,87</td>
      <td>-8,84</td>
      <td>1,42</td>
      <td>-2,48</td>
      <td>1,95</td>
      <td>-11,44</td>
    </tr>
  </tbody>
</table>

### Ошибки измерений программ относительно друг друга

|    | Length | VCCH | ACCH  |
|----|--------|---------------|-------|
| MSE | 0,08   | 78,72         | 47,64 |
| RMSE | 0,28   | 8,87          | 6,90  |
| ME | -0,24  | -7,07         | -5,90 |

## Полное исследование
Подробный обзор исследования представлен в работе:

**[Tree Inventory with LiDAR Data / I. A. Grishin, E. K. Sakharova, S. M. Ustinov [et al.] // International Conference on Neuroinformatics. – Cham: Springer International Publishing, 2022. – С. 3-11](https://link.springer.com/chapter/10.1007/978-3-031-19032-2_1)**


## Презентация
Обзор исследования вы можете посмотреть в **[презентации](about/Presentation.pdf)**:

[![Presentation](about/pres.png)](about/Presentation.pdf)

## Установка и использование

1. **Установка и запуск**:
    ```bash
    conda create --name TPCPenv379 python=3.7.9
    conda activate TPCPenv379
    pip install -r requirements.txt 
    python manage.py migrate
    python manage.py createsuperuser
    python manage.py runserver
    ```
2. **Войдите через superuser или зарегистрируйтесь**
3. **Загрузите файл дерева (примеры в test_data/*.pcd)**
4. **Рассчитатайте файл и скачайте сводку**
