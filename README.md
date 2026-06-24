# Proyecto Final: Navegación Autónoma con Planificación de Rutas BFS en Webots

**Asignatura:** Robótica y Sistemas Autónomos (ICI 4150)  
**Carrera:** Ingeniería Civil Informática  
**Institución:** Pontificia Universidad Católica de Valparaíso (PUCV)  
**Integrantes:** * Agustin Tapia
* Julian Toro
* Nicolas Soto

---

## 1. Línea Seleccionada
* **Línea A: Planificación de Rutas** (Algoritmo Búsqueda en Anchura - BFS sobre mapa discreto con soporte de Supervisor).

## 2. Objetivo del Proyecto
Diseñar, implementar y evaluar en el simulador Webots un sistema de navegación autónoma para un robot móvil diferencial e-puck. El sistema representa el entorno de forma discreta a una resolución real de 25 cm, calcula la ruta óptima mediante Búsqueda en Anchura (BFS) en tiempo de ejecución de manera genérica para múltiples escenarios, y controla el desplazamiento del robot integrando odometría cinemática y una estrategia híbrida de volteo reactivo local frente a colisiones inminentes.

## 3. Descripción del Robot, Sensores y Actuadores
* **Plataforma:** Robot diferencial e-puck simulado con el parámetro `supervisor` configurado en `TRUE`.
* **Actuadores (Motores):** Dos servomotores rotacionales continuos (`left wheel motor` y `right wheel motor`) saturados a una velocidad límite de $6.28\text{ rad/s}$.
* **Sensores de Posición (Encoders):** Dispositivos de lectura angular en las ruedas (`left wheel sensor` y `right wheel sensor`) con muestreo continuo cada $64\text{ ms}$.
* **Sensores de Percepción:** Arreglo infrarrojo analógico perimetral, priorizando los componentes frontales (`ps0`, `ps7`) y laterales (`ps1`, `ps6`) para alimentar las decisiones locales de navegación.

## 4. Descripción de los Escenarios de Prueba
La arena de simulación física tiene un área tridimensional de $2.0 \times 2.0\text{ metros}$. Su suelo cuenta con un patrón visual de tablero de ajedrez (*floor tile size*) de $0.25 \times 0.25\text{ metros}$, discretizando el plano en una grilla exacta de $8 \times 8$ celdas.
* **Escenario Simple:** Cuenta con un solo obstáculo de madera central que bloquea un área de $2 \times 2$ baldosas (filas 4 y 5, columnas 3 y 4). Evalúa la capacidad de evasión diagonal básica.
* **Escenario Complejo:** Además de la caja central, incorpora muros de bloqueo horizontales al fondo (filas 6 y 7, columnas 1, 2 y 6, 7). Forzar al e-puck a ingresar en pasillos estrechos periféricos y realizar giros cerrados a través de un cuello de botella central en `{7, 4}`.

## 5. Explicación del Algoritmo Implementado
El controlador unificado implementa un enfoque jerárquico autónomo:
1. **Detección Dinámica del Entorno:** Mediante `wb_robot_get_world_path()`, el script identifica si la cadena de texto contiene el nombre del escenario ("complejo" o "simple") y copia en memoria la matriz de ocupación real de $8 \times 8$ baldosas correspondiente.
2. **Planificación Global (BFS Nivo):** Explora los nodos transitables adyacentes utilizando una estructura de cola. Una vez que conecta el inicio `{1, 5}` con la meta `{7, 4}`, decodifica la ruta de atrás hacia adelante y realiza la conversión matemática al plano continuo de Webots:
   $$X_{\text{webots}} = (\text{Columna} \times 0.25) - 1.0 + 0.125$$
   $$Y_{\text{webots}} = (\text{Fila} \times 0.25) - 1.0 + 0.125$$
3. **Navegación Local y Volteo (Evasión Reactiva):** Si el Filtro de Kalman detecta que la distancia al frente supera el umbral crítico (`FRONT_THRESHOLD = 250.0`), el robot interrumpe el seguimiento del waypoint global y entra en modo de volteo reactivo, aplicando velocidades opuestas a los motores para pivotar sobre su propio eje hasta que las lecturas se estabilizan por debajo del umbral seguro.

---

## 6. Pseudocódigo de la Solución

```text
Función calcular_planificacion_bfs(inicio, meta):
    Inicializar visitados en FALSO y padres en (-1, -1)
    Crear Cola y encolar nodo 'inicio'
    Marcar 'inicio' como visitado

    Mientras Cola no esté vacía:
        Actual = Desencolar(Cola)
        Si Actual == meta:
            Meta_Encontrada = VERDADERO; Romper Bucle

        Para cada vecino en direcciones (Arriba, Abajo, Izquierda, Derecha):
            Si vecino dentro de límites (8x8) Y grilla[vecino] == 0 Y no visitado:
                visitado[vecino] = VERDADERO
                padre[vecino] = Actual
                Encolar(Cola, vecino)

    Si Meta_Encontrada:
        Reconstruir camino inverso guardando en ruta_global
        Convertir celdas discretas a coordenadas de Webots (Metros) aplicando RESOLUTION(0.25) y OFFSET(-1.0)
        Retornar VERDADERO
    Sino:
        Retornar FALSO

## 7. Relación Explícita con los Laboratorios 1 y 2

### Vínculo con el Laboratorio 1 (Odometría Diferencial)

El proyecto reutiliza directamente el modelo de odometría diferencial desarrollado en el Laboratorio 1. A partir de las lecturas de los encoders de las ruedas izquierda y derecha se calculan los desplazamientos incrementales de cada rueda:

```text
dist_left = R · ΔθL
dist_right = R · ΔθR
```

donde `R = 0.0205 m` corresponde al radio de las ruedas del e-puck.

Posteriormente se obtiene el avance lineal del robot y la variación angular de su orientación:

```text
ΔS = (dist_left + dist_right) / 2

Δφ = (dist_right - dist_left) / L
```

donde `L = 0.053 m` representa la distancia entre ruedas.

Utilizando estas ecuaciones se actualiza continuamente la pose completa del robot:

```text
(x, y, φ)
```

mediante integración incremental, permitiendo estimar en tiempo real la posición y orientación del e-puck dentro del entorno de simulación. Esta información constituye la base para calcular la distancia restante hacia la meta y orientar el robot durante la navegación autónoma.

### Vínculo con el Laboratorio 2 (Filtrado Sensorial y Estimación mediante Kalman)

El sistema incorpora la arquitectura de percepción desarrollada en el Laboratorio 2 para mejorar la robustez frente al ruido de los sensores infrarrojos.

Las mediciones frontales obtenidas desde los sensores `ps0` y `ps7` son fusionadas seleccionando la lectura más representativa del obstáculo frontal. Posteriormente se aplica un filtro de Promedio Móvil Exponencial (EMA) con parámetro:

```text
α = 0.85
```

según la ecuación:

```text
front_filtered =
α · front_filtered +
(1 - α) · front_raw
```

La señal suavizada es procesada mediante un Filtro de Kalman unidimensional configurado con:

```text
Q = 1.0
R = 15.0
```

El modelo predictivo utiliza la información proveniente de la odometría para estimar la evolución esperada de la distancia observada, mientras que las mediciones filtradas corrigen dicha predicción. Como resultado se obtiene una estimación más estable de la distancia frontal:

```text
estimated_distance
```

la cual es utilizada por el controlador reactivo para detectar obstáculos y activar maniobras de evasión local cuando la trayectoria hacia la meta se encuentra bloqueada.

### Integración en el Proyecto Final

Los conocimientos obtenidos en ambos laboratorios se integran en una arquitectura híbrida de navegación. La odometría diferencial proporciona una estimación continua de la pose del robot y permite calcular el rumbo hacia la meta global, mientras que el sistema de percepción basado en EMA y Filtro de Kalman supervisa el entorno inmediato para detectar obstáculos.

Cuando el camino se encuentra despejado, el robot sigue la dirección de la meta mediante un controlador proporcional sobre el error angular. En presencia de obstáculos, se activa un modo de evasión local basado en los sensores laterales, permitiendo rodear el obstáculo y retomar posteriormente la trayectoria principal. Esta combinación mejora significativamente la robustez y estabilidad de la navegación autónoma.


## 8. Resultados Obtenidos y Métricas de Desempeño

A partir de los archivos de telemetría exportados en tiempo de ejecución (`sensor_log.csv`), se exponen las métricas cuantitativas reales del desempeño del e-puck:

| Métrica de Evaluación              | Escenario Simple (`copia.csv`)  | Escenario Complejo (`copia (2).csv`) |
| ---------------------------------- | ------------------------------- | ------------------------------------ |
| **Tiempo de Ejecución Total**      | **16.064 segundos**             | **26.560 segundos**                  |
| **Cantidad de Muestras/Registros** | 250 muestras                    | 414 muestras                         |
| **Tolerancia de Llegada a Meta**   | Menor a 0.06 metros             | Menor a 0.06 metros                  |
| **Número de Colisiones Críticas**  | 0                               | 0                                    |
| **Estado Final de Motores**        | Detenido Absoluto (`0.0 rad/s`) | Detenido Absoluto (`0.0 rad/s`)      |

### Análisis de los Gráficos de Trayectoria

* **Trayectoria Simple:** El robot avanza de forma fluida describiendo una curva limpia directa hacia la baldosa destino, desacelerando proporcionalmente al aproximarse al umbral de parada rápida.
* **Trayectoria Compleja:** El gráfico lineal refleja la capacidad del planificador global para trazar esquinas ortogonales. El robot maniobra de forma asimétrica abriéndose paso en los bordes de las baldosas sin chocar gracias al soporte del Filtro de Kalman.

**Nota:** Para adjuntar los gráficos en esta sección, importa las columnas `robot_x` y `robot_y` de tus archivos CSV en Excel, genera un gráfico de dispersión con líneas suavizadas, guárdalos como imágenes dentro de una carpeta llamada `images/` en tu repositorio y asegúrate de que los nombres coincidan con las rutas utilizadas en el README.

---

## 9. Capturas, Gráficos y Videos

### Video Demostrativo del Proyecto

[Pega aquí el enlace a tu video de YouTube o Google Drive]

*Asegúrate de que el video esté configurado como público u oculto con enlace compartido para que el corrector pueda visualizar al e-puck calculando el BFS en la terminal y deteniéndose completamente al llegar a la meta.*

---

## 10. Instrucciones para Ejecutar la Simulación

1. Descarga o clona este repositorio:

```bash
git clone https://github.com/TU_USUARIO/NOMBRE_REPOSITORIO.git
```

2. Abre el simulador **Webots (Versión 2026)**.
3. Carga el escenario deseado desde:

```text
File → Open World
```

4. Selecciona:

```text
escenario_simple1.wbt
```

o

```text
escenario_complejo.wbt
```

5. En el editor de Webots abre:

```text
controllers/e_puck_laboratorio_2/e_puck_laboratorio_2.c
```

6. Haz clic en **Clean** y posteriormente en **Rebuild** para compilar el controlador.
7. Presiona **Play** para iniciar la simulación.

---

## 11. Conclusiones, Limitaciones y Mejoras

### Conclusiones

La implementación del algoritmo BFS integrado nativamente en C demostró cumplir con los principios de optimización en espacios discretizados, garantizando rutas con el menor número de baldosas transitadas. La combinación híbrida (control global BFS + control local reactivo) asegura que el e-puck responda exitosamente ante dinámicas complejas del mapa sin quedar atrapado.

### Limitaciones

La odometría acumulativa basada en integración de Euler pura sufre de deriva cinemática a lo largo del tiempo debido al deslizamiento microscópico simulado de las ruedas sobre la superficie de Webots.

### Posibles Mejoras

Incorporar un sensor de brújula (`Compass`) o un sistema de referencias externas (*Landmarks*) para implementar un **Filtro de Kalman Extendido (EKF)** capaz de corregir continuamente los desvíos acumulados en la pose:

```text
(x, y, φ)
```
