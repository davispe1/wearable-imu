# Quickstart — Gait Kinematics Library

---

## 1. Requisitos

Python 3.10 o superior. Instalar dependencias:

```bash
pip install -r requirements.txt
```

Instala automáticamente: `numpy`, `scipy`, `vqf`, `matplotlib`.
**OpenSim no es una dependencia de Python** — se instala por separado si se quiere usar IK.

---

## 2. Formato de los datos de entrada

Los datos de una sesión van en **una carpeta**, con un archivo CSV por nodo:

```
P04_S01_2minWalk/
├── SA.csv        ← pelvis
├── RT.csv        ← muslo derecho
├── RS.csv        ← tibia derecha
└── RF.csv        ← pie derecho
```

Cada CSV tiene estas columnas (9 DOF + tiempo):

| Columna | Contenido | Unidades |
|---|---|---|
| `t_opt_s` | Tiempo compartido (reloj del hub, 0 al inicio) | s |
| `ax ay az` | Aceleración lineal | m/s² |
| `gx gy gz` | Velocidad angular | rad/s |
| `mx my mz` | Campo magnético | u.a. |

> También se acepta un archivo combinado en `<sesión>/raw/data.csv` con una columna extra `node` que identifica cada fila.

Los nombres de columna se resuelven automáticamente — también acepta `acc_x`, `gyr_x`, `t_s`, etc.

---

## 3. Correr el pipeline cinemático

Ejecutar desde la carpeta raíz del proyecto (`gait-opensim/`):

```bash
python -m kinematics.pipeline  data/P04_S01_2minWalk  --csv
```

- Procesa todos los nodos presentes en la carpeta
- Calcula ángulos articulares, eventos de marcha y parámetros espacio-temporales
- `--csv` guarda los resultados en `<sesión>/results/`

**Resultados generados:**

```
P04_S01_2minWalk/results/
├── P04_S01_2minWalk_joint_angles.csv       ← ángulos + velocidad + eventos (una fila por muestra)
├── P04_S01_2minWalk_gait_events.csv        ← tabla de eventos: tiempo, muestra, tipo
└── P04_S01_2minWalk_gait_parameters.json   ← todos los parámetros (ROM, cadencia, zancada…)
```

El pipeline también imprime un resumen en consola con ROM, cadencia y parámetros temporales.

---

## 4. Visualizar los resultados

### Opción A — GUI interactiva (recomendada)

```bash
python -m app.results_gui  data/P04_S01_2minWalk
```

- Abre una ventana con **dos pestañas**:
  - **Plots** — las gráficas (ángulos vs tiempo, señal de eventos, ciclo de marcha promedio)
  - **Parameters** — la tabla de valores resumen (ROM, cadencia, zancada, apoyo/balanceo…)
- Botón **"Open session…"** para cambiar de sesión sin cerrar la ventana
- Toolbar de matplotlib para zoom y pan sobre las gráficas
- También se puede iniciar sin argumentos y abrir la sesión desde el botón:
  ```bash
  python -m app.results_gui
  ```

### Opción B — figura de línea de comandos

```bash
python -m app.viewer  data/P04_S01_2minWalk  --save
```

- `--save` escribe la figura como PNG en `<sesión>/results/`
- Sin `--save` abre una ventana interactiva de matplotlib
- También genera los mismos CSV/JSON del paso anterior si aún no existen

**Ventana de tiempo por defecto:** 12 segundos de marcha estable (excluye giros).

---

## 5. Exportar para OpenSim (opcional)

Solo si se quiere continuar con cinemática inversa en OpenSim OpenSense:

```bash
python -m opensim_export.to_sto  data/P04_S01_2minWalk
```

Genera en `<sesión>/results/`:

```
P04_S01_2minWalk/results/
├── P04_S01_2minWalk_orientations.sto    ← cuaterniones completos para OpenSense
└── P04_S01_2minWalk_calibration.sto     ← pose estática inicial (~1 s)
```

Estos archivos se usan en OpenSim como entrada para el `IMU Placer` y el `IMU IK`. Ver `docs/opensim_steps.md` para los pasos dentro de OpenSim.

Si necesitas una pose de calibración diferente a la del primer segundo (por ejemplo, usando una ventana de reposo explícita), usa:

```bash
# Promedio sobre una ventana estática [T0, T1] segundos
python -m opensim_export.make_calibration  results/P04_orientations.sto  results/P04_calib.sto  --window 0 5

# Auto-selección del instante más cercano a postura neutral
python -m opensim_export.make_calibration  results/P04_orientations.sto  results/P04_calib.sto  --auto-neutral
```

---

## Opciones útiles

| Opción | Efecto |
|---|---|
| `--mode 6D` | Solo giroscopio + acelerómetro (predeterminado, sin magnetómetro) |
| `--mode 9D` | Agrega magnetómetro para heading absoluto |
| `--mode auto` | Usa 9D si hay magnetómetro calibrado, si no usa 6D |
| `--side right` | Fuerza la pierna medida (por defecto se infiere del nombre de los nodos) |
| `--t0 5 --t1 17` | (solo viewer) Rango de tiempo en segundos para la figura |

---

## Resumen de comandos

```bash
# Instalar
pip install -r requirements.txt

# Procesar sesión y guardar resultados
python -m kinematics.pipeline  <carpeta_sesion>  --csv

# Visualizar
python -m app.viewer  <carpeta_sesion>  --save

# Exportar para OpenSim (opcional)
python -m opensim_export.to_sto  <carpeta_sesion>
```
