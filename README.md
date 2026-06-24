# Proyecto Final: Navegación Autónoma con Planificación de Rutas BFS en Webots

**Asignatura:** Robótica y Sistemas Autónomos (ICI 4150)
**Carrera:** Ingeniería Civil Informática
**Institución:** Pontificia Universidad Católica de Valparaíso (PUCV)

**Integrantes:**

    * Agustin Tapia
    * Nicolas Soto
    * Julian Toro

---

# 1. Línea Seleccionada

**Línea A: Planificación de Rutas**

El proyecto implementa un sistema de navegación autónoma basado en el algoritmo Breadth-First Search (BFS), complementado con navegación reactiva local mediante sensores infrarrojos y estimación de estado utilizando técnicas de filtrado.

---

# 2. Objetivo del Proyecto

Diseñar e implementar un sistema de navegación autónoma para el robot diferencial e-puck en el simulador Webots, capaz de:

* Representar el entorno mediante una grilla discreta.
* Calcular rutas óptimas utilizando BFS.
* Navegar autónomamente entre una posición inicial y una meta.
* Evitar obstáculos mediante control reactivo basado en sensores.
* Utilizar odometría diferencial para estimar la pose del robot.
* Incorporar técnicas de filtrado para mejorar la calidad de las mediciones.

---

# 3. Descripción del Robot, Sensores y Actuadores

## Plataforma Robótica

Se utilizó el robot móvil diferencial e-puck disponible en Webots.

### Actuadores

El robot posee dos motores diferenciales:

* Left Wheel Motor
* Right Wheel Motor

Configurados en modo de velocidad continua.

Velocidad máxima:

[
\omega_{max}=6.28\ rad/s
]

---

## Sensores de Posición

Para estimar el movimiento del robot se utilizan:

* Left Wheel Sensor
* Right Wheel Sensor

Estos encoders permiten obtener el desplazamiento angular acumulado de cada rueda.

---

## Sensores de Distancia

Se utilizan los sensores infrarrojos:

* ps0 (frontal derecho)
* ps7 (frontal izquierdo)
* ps1 (lateral derecho)
* ps6 (lateral izquierdo)

Estos sensores son utilizados para:

* Detección de obstáculos.
* Activación de navegación reactiva.
* Estimación de distancia frontal filtrada.

---

# 4. Descripción de los Escenarios de Prueba

La arena utilizada posee dimensiones físicas de:

[
2.0 \times 2.0\ m
]

El espacio se discretiza en una grilla de:

[
8 \times 8
]

donde cada celda representa:

[
0.25 \times 0.25\ m
]

---

## Escenario Simple

Contiene una caja de madera ubicada aproximadamente en el centro del entorno.

La matriz de ocupación contiene una única región bloqueada de 2×2 celdas.

El objetivo es evaluar la capacidad del algoritmo BFS para encontrar una ruta alternativa alrededor del obstáculo central.

---

## Escenario Complejo

Incluye:

* La caja central del escenario simple.
* Obstáculos laterales adicionales.
* Corredores estrechos.

Este escenario exige una mayor cantidad de cambios de dirección y pone a prueba:

* El seguimiento de waypoints.
* La precisión de la odometría.
* El comportamiento reactivo ante obstáculos.

---

# 5. Explicación del Algoritmo Implementado

El sistema implementa una arquitectura híbrida compuesta por:

1. Planificación global mediante BFS.
2. Navegación local reactiva basada en sensores.

---

## 5.1 Selección Automática del Escenario

Al iniciar la simulación, el controlador obtiene la ruta del mundo abierto mediante:

```c
wb_robot_get_world_path()
```

Si el nombre contiene la palabra:

```text
complejo
```

se carga la matriz de ocupación correspondiente al escenario complejo.

En caso contrario se utiliza automáticamente el escenario simple.

De esta forma un único controlador puede operar correctamente en ambos mundos.

---

## 5.2 Planificación Global con BFS

La planificación se realiza sobre una matriz de ocupación de 8×8.

Cada celda puede contener:

* 0 → espacio libre.
* 1 → obstáculo.

La búsqueda utiliza conectividad de cuatro vecinos:

* Arriba
* Abajo
* Izquierda
* Derecha

Una vez encontrada la meta se reconstruye el camino utilizando una matriz de padres.

---

## 5.3 Generación de Waypoints

Las celdas calculadas por BFS se transforman a coordenadas métricas de Webots mediante:

[
x=(columna \cdot 0.25)-1.0+0.125
]

[
y=(fila \cdot 0.25)-1.0+0.125
]

Cada punto generado corresponde a un waypoint de navegación.

---

## 5.4 Navegación Local Reactiva

Mientras el robot sigue la ruta BFS, se supervisa constantemente la distancia frontal estimada.

Cuando se detecta una posible colisión:

[
distancia > FRONT_THRESHOLD
]

se activa el modo de evasión.

El robot gira hacia el lado con mayor espacio libre según las mediciones laterales.

Cuando la distancia vuelve a una condición segura:

[
distancia < CLEAR_THRESHOLD
]

se retorna al seguimiento normal de la trayectoria global.

---

# 6. Pseudocódigo de la Solución

```text
Inicio

Detectar escenario abierto

Si escenario contiene "complejo":
    cargar mapa_complejo
Sino:
    cargar mapa_simple

Definir inicio y meta

Ejecutar BFS

Si no existe camino:
    terminar programa

Reconstruir camino

Convertir camino a waypoints métricos

Mientras la simulación esté activa:

    Leer sensores IR

    Aplicar filtro EMA

    Aplicar filtro de Kalman

    Actualizar odometría diferencial

    Obtener waypoint actual

    Si waypoint alcanzado:
        avanzar al siguiente

    Si meta alcanzada:
        detener motores

    Calcular error angular

    Si existe obstáculo frontal:
        activar evasión
        girar hacia lado más despejado
    Sino:
        seguir waypoint mediante control proporcional

    Registrar telemetría

Fin Mientras
```

---

# 7. Relación con los Laboratorios 1 y 2

## Laboratorio 1: Odometría Diferencial

El proyecto reutiliza directamente la odometría desarrollada anteriormente.

A partir de los encoders se calcula:

[
\Delta S_L=r\Delta\theta_L
]

[
\Delta S_R=r\Delta\theta_R
]

El avance lineal del robot es:

[
\Delta S=\frac{\Delta S_R+\Delta S_L}{2}
]

y la variación angular:

[
\Delta \phi=\frac{\Delta S_R-\Delta S_L}{L}
]

donde:

* Radio de rueda:

[
r=0.0205\ m
]

* Distancia entre ruedas:

[
L=0.053\ m
]

Con ello se actualiza continuamente la pose:

[
(x,y,\phi)
]

durante toda la navegación.

---

## Laboratorio 2: Filtrado Sensorial

Las mediciones de distancia son suavizadas utilizando un filtro EMA:

[
EMA_t=\alpha EMA_{t-1}+(1-\alpha)z_t
]

con:

[
\alpha=0.85
]

Posteriormente se aplica un Filtro de Kalman de un estado utilizando:

[
Q=1.0
]

[
R=15.0
]

La estimación obtenida es utilizada para la detección robusta de obstáculos y la activación del modo reactivo.

---

# 8. Resultados Obtenidos

Las métricas se obtuvieron a partir del archivo csv de cada escenario:


### Análisis de Trayectos - `sensor_log.csv`

| Waypoint | Registros | Tiempo Inicial (s) | Tiempo Final (s) | Duración (s) | Promedio X | Promedio Y | Promedio Phi (rad) | Promedio Front |
| :---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| **1** | 57 | 0.128 | 3.712 | 3.584 | 0.3738 | -0.5432 | 1.9700 | 143.9065 |
| **2** | 78 | 3.776 | 8.704 | 4.928 | 0.3741 | -0.2734 | 0.8529 | 148.8324 |
| **3** | 69 | 8.768 | 13.120 | 4.352 | 0.3702 | -0.1068 | 1.8360 | 143.9704 |
| **4** | 66 | 13.184 | 17.344 | 4.160 | 0.3745 | 0.2341 | 1.8934 | 124.9643 |
| **5** | 43 | 17.408 | 20.096 | 2.688 | 0.3720 | 0.4403 | 1.5708 | 77.8417 |
| **6** | 15 | 20.160 | 21.056 | 0.896 | 0.3744 | 0.6112 | 1.5639 | 70.7557 |

### Análisis de Trayectos - `sensor_log11.csv`

| Waypoint | Registros | Tiempo Inicial (s) | Tiempo Final (s) | Duración (s) | Promedio X | Promedio Y | Promedio Phi (rad) | Promedio Front |
| :---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| **1** | 33 | 0.128 | 2.176 | 2.048 | 0.3750 | -0.5307 | 1.5707 | 51.4862 |
| **2** | 42 | 2.240 | 4.864 | 2.624 | 0.3750 | -0.3097 | 1.5708 | 74.2248 |
| **3** | 43 | 4.928 | 7.616 | 2.688 | 0.3750 | -0.0592 | 1.5709 | 71.6065 |
| **4** | 74 | 7.680 | 12.352 | 4.672 | 0.3744 | 0.2129 | 1.6337 | 162.3152 |
| **5** | 65 | 12.416 | 16.512 | 4.096 | 0.3710 | 0.3993 | 1.8479 | 140.5081 |
| **6** | 42 | 16.576 | 19.200 | 2.624 | 0.3744 | 0.6916 | 1.5654 | 73.4226 |
| **7** | 33 | 19.264 | 21.312 | 2.048 | 0.3234 | 0.8375 | 2.6816 | 70.2449 |

---

## Análisis Cualitativo

### Escenario Simple

El robot genera una trayectoria óptima rodeando el obstáculo central y alcanza la meta sin colisiones.

### Escenario Complejo

La ruta calculada requiere múltiples cambios de dirección y seguimiento de corredores estrechos.

La navegación reactiva permite corregir desviaciones y evitar impactos contra obstáculos laterales.

---

# 9. Capturas, Gráficos y Video

## Video de Demostración

Agregar enlace al video:

```text
[ENLACE AQUÍ]
```

---

## Gráfico de Trayectoria

Agregar imágenes obtenidas desde los datos registrados:

```text
images/trayectoria_simple.png
images/trayectoria_compleja.png
```

---

# 10. Instrucciones de Ejecución

Clonar el repositorio:

```bash
git clone https://github.com/USUARIO/REPOSITORIO.git
```

Abrir Webots.

Abrir alguno de los mundos:

```text
worlds/escenario_simple.wbt
```

o

```text
worlds/escenario_complejo.wbt
```

Compilar el controlador.

Ejecutar la simulación.

El controlador detectará automáticamente el escenario cargado y calculará la ruta BFS correspondiente.

---

# 11. Conclusiones

La implementación demostró que BFS permite obtener rutas óptimas en entornos discretizados, garantizando el menor número de celdas recorridas para alcanzar la meta.

La combinación entre:

* planificación global mediante BFS,
* odometría diferencial,
* filtrado EMA,
* filtro de Kalman,
* y navegación reactiva basada en sensores,

permitió construir un sistema autónomo robusto capaz de operar exitosamente en distintos escenarios utilizando un único controlador.

Como trabajo futuro se propone incorporar algoritmos como A*, Dijkstra o SLAM para permitir planificación sobre mapas desconocidos y actualización dinámica de obstáculos.
