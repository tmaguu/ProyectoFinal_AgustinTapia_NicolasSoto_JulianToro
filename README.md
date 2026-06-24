# Proyecto Final: Navegación Autónoma con Planificación de Rutas BFS en Webots

**Asignatura:** Robótica y Sistemas Autónomos 

**Integrantes:** Agustin Tapia, Julian Toro, Nicolas Soto

---

## 1. Línea Seleccionada

* **Línea A: Planificación de Rutas** (Algoritmo Búsqueda en Anchura - BFS sobre mapa discreto).

## 2. Objetivo del Proyecto

Diseñar, implementar y evaluar un sistema de navegación autónoma para un robot móvil diferencial e-puck en el simulador Webots. El sistema debe ser capaz de representar el entorno de forma discreta, calcular la ruta matemáticamente óptima utilizando el algoritmo BFS en tiempo de ejecución, y ejecutarla de forma segura integrando técnicas de filtrado sensorial y odometría diferencial.

## 3. Descripción del Robot, Sensores y Actuadores

* **Plataforma:** Robot diferencial e-puck.
* **Actuadores (Motores):** Dos servomotores configurados en modo de velocidad continua (`left wheel motor` y `right wheel motor`) con una saturación física máxima de `6.28 rad/s`.
* **Sensores de Posición (Encoders):** `left wheel sensor` y `right wheel sensor` activados a un paso de muestreo (`TIME_STEP`) de `64 ms` para medir la rotación angular acumulada.
* **Sensores de Distancia:** Arreglo infrarrojo integrado (`ps0` a `ps7`) utilizando lecturas analógicas escaladas para la detección local de obstáculos.

## 4. Descripción de los Escenarios de Prueba

La arena de simulación cuenta con un área física total de `2.0 × 2.0 m`. El suelo posee un diseño visual con un *floor tile size* de `0.5 × 0.5 m`, discretizando el espacio en una cuadrícula exacta de `4 × 4` baldosas.

### Escenario Simple

Entorno con un obstáculo aislado en la baldosa central `{1,1}`. Permite evaluar la generación de una ruta directa en diagonal desde el origen `{0,0}` hasta el destino intermedio `{2,0}`.

### Escenario Complejo

Laberinto estructurado mediante muros horizontales intercalados (filas de bloques en `{1,1}`, `{1,2}`, `{1,3}` y `{3,1}`, `{3,2}`, `{3,3}`). Obliga al robot a navegar por pasillos estrechos periféricos, forzando giros cerrados de 90° y poniendo a prueba la estabilidad de la odometría.

## 5. Explicación del Algoritmo Implementado

El controlador ejecuta el algoritmo **BFS (Breadth-First Search)** embebido nativamente en C al arrancar la simulación:

1. **Modelación Espacial:** El entorno se mapea en una matriz estática de `4 × 4` donde los espacios libres son `0` y los obstáculos físicos son `1`.
2. **Exploración:** Una cola estática gestiona la expansión de nodos adyacentes (Arriba, Abajo, Izquierda, Derecha). Al alcanzar la celda objetivo, se reconstruye el camino inverso mediante punteros de asignación de padres.
3. **Conversión Cinemática:** Los índices discretos de la matriz (Fila, Columna) se transforman al espacio continuo de Webots usando:

```text
X_webots = (Columna × 0.5) - 1.0 + 0.25
Y_webots = (Fila × 0.5) - 1.0 + 0.25
```

---

## 6. Pseudocódigo de la Solución

```text
Función calcular_planificacion_bfs(inicio, meta):
    Inicializar matriz de visitados en FALSO
    Inicializar matriz de padres en (-1, -1)
    Crear ColaEstática e insertar nodo 'inicio'
    Marcar 'inicio' como visitado

    Mientras ColaEstática no esté vacía:
        Celda_Actual = Desencolar(ColaEstática)

        Si Celda_Actual == meta:
            Meta_Encontrada = VERDADERO
            Romper bucle

        Para cada vecino (Arriba, Abajo, Izquierda, Derecha):
            Si vecino está dentro de los límites (4x4)
               Y no visitado
               Y grilla[vecino] == 0:

                Marcar vecino como visitado
                padre[vecino] = Celda_Actual
                Encolar(ColaEstática, vecino)

    Si Meta_Encontrada:
        Reconstruir camino inverso desde 'meta'
        usando matriz de padres

        Convertir celdas a coordenadas métricas
        aplicando RESOLUTION (0.5)
        y OFFSET (-1.0)

        Retornar VERDADERO

    Sino:
        Retornar FALSO
```

---

## 7. Relación Explícita con los Laboratorios 1 y 2

### Vínculo con Laboratorio 1 (Odometría Completa)

Se extraen los deltas de posición angular de los encoders (`ΔθL`, `ΔθR`) y mediante el modelo cinemático diferencial directo se integra paso a paso la pose del robot (`x`, `y`, `φ`) usando la aproximación de Euler:

```text
ΔS = (r · (ΔθR + ΔθL)) / 2

Δφ = (r · (ΔθR - ΔθL)) / L
```

### Vínculo con Laboratorio 2 (Filtrado y Percepción Frontal)

Las lecturas crudas de los sensores frontales se suavizan mediante un filtro de promedio móvil exponencial (EMA) con `α = 0.85`.

Posteriormente, se procesan mediante un **Filtro de Kalman de un estado** (`Q = 1.0`, `R = 15.0`) para obtener una estimación robusta de la distancia frontal, activando la evasión local reactiva por histéresis si el rumbo global se ve bloqueado imprevistamente.

---

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
