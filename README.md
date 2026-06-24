# Proyecto Final: Navegación Autónoma con Planificación de Rutas BFS en Webots

**Asignatura:** Robótica y Sistemas Autónomos (ICI 4150)  
**Carrera:** Ingeniería Civil Informática  
**Institución:** Pontificia Universidad Católica de Valparaíso (PUCV)  
**Integrantes:** * [Tu Nombre Aquí]  
* [Nombre de Integrante 2, si aplica]  

---

## 1. Línea Seleccionada
* **Línea A: Planificación de Rutas** (Algoritmo Búsqueda en Anchura - BFS sobre mapa discreto).

## 2. Objetivo del Proyecto
Diseñar, implementar y evaluar un sistema de navegación autónoma para un robot móvil diferencial e-puck en el simulador Webots. El sistema debe ser capaz de representar el entorno de forma discreta, calcular la ruta matemáticamente óptima utilizando el algoritmo BFS en tiempo de ejecución, y ejecutarla de forma segura integrando técnicas de filtrado sensorial y odometría diferencial.

## 3. Descripción del Robot, Sensores y Actuadores
* **Plataforma:** Robot diferencial e-puck.
* **Actuadores (Motores):** Dos servomotores configurados en modo de velocidad continua (`left wheel motor` y `right wheel motor`) con una saturación física máxima de $6.28\text{ rad/s}$.
* **Sensores de Posición (Encoders):** `left wheel sensor` y `right wheel sensor` activados a un paso de muestreo (`TIME_STEP`) de $64\text{ ms}$ para medir la rotación angular acumulada.
* **Sensores de Distancia:** Arreglo infrarrojo integrado (`ps0` a `ps7`) utilizando lecturas analógicas escaladas para la detección local de obstáculos.

## 4. Descripción de los Escenarios de Prueba
La arena de simulación cuenta con un área física total de $2.0 \times 2.0\text{ metros}$. El suelo posee un diseño visual con un *floor tile size* de $0.5 \times 0.5\text{ metros}$, discretizando el espacio en una cuadrícula exacta de $4 \times 4$ baldosas.
* **Escenario Simple:** Entorno con un obstáculo aislado en la baldosa central `{1, 1}`. Permite evaluar la generación de una ruta directa en diagonal desde el origen `{0, 0}` hasta el destino intermedio `{2, 0}`.
* **Escenario Complejo:** Laberinto estructurado mediante muros horizontales intercalados (filas de bloques en `{1, 1}, {1, 2}, {1, 3}` y `{3, 1}, {3, 2}, {3, 3}`). Obliga al robot a navegar por pasillos estrechos periféricos, forzando giros cerrados de 90° y poniendo a prueba la estabilidad de la odometría.

## 5. Explicación del Algoritmo Implementado
El controlador ejecuta el algoritmo **BFS (Breadth-First Search)** embebido nativamente en C al arrancar la simulación:
1. **Modelación Espacial:** El entorno se mapea en una matriz estática de $4 \times 4$ donde los espacios libres son `0` y los obstáculos físicos son `1`.
2. **Exploración:** Una cola estática gestiona la expansión de nodos adyacentes (Arriba, Abajo, Izquierda, Derecha). Al alcanzar la celda objetivo, se reconstruye el camino inverso mediante punteros de asignación de padres.
3. **Conversión Cinemática:** Los índices discretos de la matriz (Fila, Columna) se transforman al espacio continuo de Webots usando la ecuación:
   $$X_{\text{webots}} = (\text{Columna} \times 0.5) - 1.0 + 0.25$$
   $$Y_{\text{webots}} = (\text{Fila} \times 0.5) - 1.0 + 0.25$$

---

## 6. Pseudocódigo de la Solución

