---
marp: true
theme: default
paginate: true
style: |
  section {
    font-family: 'Segoe UI', Arial, sans-serif;
    font-size: 22px;
    padding: 40px 56px;
  }
  section.title {
    background: #0d1b2a;
    color: #e8f4fd;
    text-align: center;
    display: flex;
    flex-direction: column;
    justify-content: center;
  }
  section.title h1 { font-size: 2em; color: #60b4f5; margin-bottom: 0.2em; }
  section.title h2 { font-size: 1em; color: #9fc8e8; font-weight: 400; }
  h1 { color: #1a3a5c; border-bottom: 3px solid #60b4f5; padding-bottom: 8px; margin-bottom: 18px; }
  h2 { color: #2a5a8c; }
  code { background: #f0f6ff; border-radius: 4px; padding: 2px 6px; font-size: 0.88em; color: #1a3a5c; }
  pre  { background: #f0f6ff; border-left: 4px solid #60b4f5; padding: 14px 18px; border-radius: 0 6px 6px 0; font-size: 0.82em; }
  pre code { background: transparent; padding: 0; }
  table { width: 100%; border-collapse: collapse; font-size: 0.88em; }
  th { background: #1a3a5c; color: white; padding: 8px 12px; }
  td { padding: 7px 12px; border-bottom: 1px solid #d0e4f5; }
  tr:nth-child(even) td { background: #f0f6ff; }
  .box { background: #f0f6ff; border-left: 4px solid #3a7abf; padding: 12px 18px; border-radius: 0 6px 6px 0; margin-top: 16px; }
  .box-green { background: #f0fff4; border-left: 4px solid #38a169; padding: 12px 18px; border-radius: 0 6px 6px 0; margin-top: 16px; }
---

<!-- _class: title -->

# Gait Kinematics Library
## Arquitectura y módulos

4 nodos IMU → ángulos articulares y parámetros de marcha — en Python

---

# ¿Qué hace la biblioteca?

**Entrada:** carpeta con los CSV crudos de los 4 nodos IMU (giroscopio · acelerómetro · magnetómetro)

**Salida:** ángulos de cadera, rodilla y tobillo + parámetros espacio-temporales de la marcha

```
CSV por nodo  →  orientaciones (VQF)  →  ángulos articulares  →  parámetros  →  figura
```

La biblioteca tiene **tres secciones**:

| Sección | Carpeta | Función |
|---|---|---|
| Procesamiento y resultados | `core/` + `kinematics/` | De datos crudos a parámetros |
| Visualización | `app/` | GUI interactiva y figura de línea de comandos |
| Exportación a OpenSim | `opensim_export/` | Archivos `.sto` para IK (opcional) |

---

# Sección 1 — Procesamiento y resultados

`core/` y `kinematics/` forman el núcleo de la biblioteca. El usuario llama a **una sola función** (`analyze_session`) y el resto ocurre internamente.

## `core/` — datos crudos → orientaciones

| Archivo | Qué hace |
|---|---|
| `rawdata.py` | Lee los CSV de cada nodo IMU |
| `config.py` | Define qué sensores hay y en qué segmento (muslo, tibia, pie…) |
| `fusion_vqf.py` | Fusión de giroscopio + acelerómetro con **VQF** → cuaternión por segmento |

## `kinematics/` — orientaciones → parámetros

| Archivo | Qué hace |
|---|---|
| `pipeline.py` | **Punto de entrada:** `analyze_session()` llama a todo lo demás |
| `joint_angles.py` | Cuaterniones → flexión sagital de cadera / rodilla / tobillo |
| `gait_events.py` | Detecta heel strike, toe-off y giros del sujeto |
| `parameters.py` | Calcula ROM, cadencia, tiempo de zancada, longitud de paso… |
| `results.py` | Empaqueta el resultado y exporta CSV / JSON |
| `quaternion.py` | Operaciones matemáticas de cuaterniones (uso interno) |

---

# Por qué se usa VQF

VQF (Laidig & Seel, *Information Fusion* 2023) es un estimador de orientación validado publicado:
- Estima la orientación de cada segmento en el marco terrestre como un cuaternión
- Incluye estimación de bias del giroscopio, detección de reposo y rechazo de perturbaciones magnéticas
- **No se reimplementa:** se importa directamente del paquete oficial `pip install vqf`

**Modo 6D (predeterminado):** solo giroscopio + acelerómetro.
Ángulo de inclinación referenciado a la gravedad — sin necesidad de calibración magnética.

**Modo 9D:** agrega el magnetómetro para heading absoluto.
Útil solo si el magnetómetro está calibrado y el entorno magnético es limpio.

<div class="box-green">

El ángulo articular sagital usa solo la **dirección de la gravedad** → inmune al drift de heading. Por eso 6D es el modo robusto por defecto.

</div>

---

# Sección 2 — Visualización · `app/`

| Archivo | Qué hace |
|---|---|
| `results_gui.py` | **GUI interactiva** (tkinter) con dos pestañas: **Plots** (gráficas) y **Parameters** (tabla). El pipeline corre en segundo plano sin bloquear la ventana. |
| `viewer.py` | **Línea de comandos**: figura de 4 paneles (gráficas + tabla) guardada como PNG. |

Las dos pestañas separan cosas distintas: la figura muestra **series en el tiempo** (cómo se mueve la articulación), la tabla muestra los **valores resumen** (un número por parámetro).

```bash
# GUI interactiva (recomendada)
python -m app.results_gui  data/P04_S01_2minWalk

# Línea de comandos (guarda figura como PNG)
python -m app.viewer  data/P04_S01_2minWalk  --save
```

---

# Sección 3 — Exportación a OpenSim · `opensim_export/`

Esta sección es **opcional** e independiente del análisis cinemático en Python.
OpenSim usa cinemática inversa (IK) con un modelo biomecánico completo — requiere orientaciones por segmento, no ángulos.

| Archivo | Qué hace |
|---|---|
| `to_sto.py` | Escribe los archivos `.sto` que consume OpenSense |
| `segment_map.py` | Mapeo de nombres de nodos a columnas de OpenSim |
| `make_calibration.py` | Genera un `.sto` de calibración desde una ventana estática o selección automática del instante más neutral |

Produce en `<sesión>/results/`:

- `<id>_orientations.sto` — cuaterniones completos (un frame por muestra)
- `<id>_calibration.sto` — pose estática inicial (primer segundo, por defecto)

```bash
# Exportar orientaciones
python -m opensim_export.to_sto  data/P04_S01_2minWalk

# Calibración personalizada (ventana estática o auto-neutral)
python -m opensim_export.make_calibration  results/P04_orientations.sto  results/P04_calib.sto  --window 0 5
```

<div class="box">

OpenSim nunca es una dependencia de Python. Solo consume los `.sto` que genera este módulo.

</div>

---

# Métodos y referencias

Cada parámetro está respaldado por literatura publicada — no es un cálculo ad-hoc. Esto es lo que sustenta cada etapa:

| Etapa / parámetro | Método | Referencia |
|---|---|---|
| Orientación de segmentos | Filtro VQF | Laidig & Seel, *Information Fusion* 91:187–204, 2023 |
| Ángulo articular sagital | Eje funcional + proyección de gravedad | Seel, Raisch & Schauer, *Sensors* 14(4):6891–6909, 2014 |
| Eventos de marcha (IC / TO) | Pico de mid-swing en giroscopio de tibia | Aminian *et al.*, *J. Biomech.* 2002; Salarian *et al.*, *IEEE TBME* 2004 |
| Variabilidad (CV tiempo de zancada) | Coeficiente de variación | Hausdorff, *J. NeuroEng. Rehabil.* 2:19, 2005 |
| Longitud de zancada (estimada) | Doble integración con ZUPT del pie | Mariani *et al.*, *J. Biomech.* 43(15):2999–3006, 2010 |
| Exportación OpenSim | OpenSense IK | Al Borno *et al.*, *J. NeuroEng. Rehabil.* 19:22, 2022 |

<div class="box-green">

Lista completa y detalle de cada método en [`docs/kinematics.md`](kinematics.md) (pipeline cinemático) y [`docs/method.md`](method.md) (ruta OpenSim).

</div>

---

<!-- _class: title -->

# Resumen

**4 nodos IMU → orientación VQF validada → ángulos sagitales → eventos de marcha → parámetros espacio-temporales → figura + tabla en Python**

Procesamiento (`core` + `kinematics`) · Visualización (`app`) · Exportación opcional a OpenSim (`opensim_export`)
