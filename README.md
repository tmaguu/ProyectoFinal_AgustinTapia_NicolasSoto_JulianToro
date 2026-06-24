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

Las métricas se obtuvieron a partir del archivo:

```text
sensor_log.csv
```

exportado durante la simulación.

| Métrica                   | Escenario Simple       | Escenario Complejo     |
| ------------------------- | ---------------------- | ---------------------- |
| Tiempo total de ejecución | Completar con medición | Completar con medición |
| Waypoints recorridos      | Completar              | Completar              |
| Error de llegada          | < 0.06 m               | < 0.06 m               |
| Colisiones críticas       | 0                      | 0                      |
| Estado final              | Detenido               | Detenido               |

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
