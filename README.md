# Sistema de Confort Térmico en Aula de Clases

Proyecto desarrollado en **Arduino** para monitorear temperatura, humedad y luminosidad,
manteniendo condiciones de confort térmico para dos estudiantes dentro de un aula.

## 🧩 Descripción general
El sistema calcula el índice **PMV (Predicted Mean Vote)** y controla diferentes actuadores
según el nivel de confort térmico detectado:

- **PMV > 0.5:** Ambiente caluroso → Se activa el ventilador (LED rojo parpadeante).
- **PMV < -0.7:** Ambiente frío → Se activa el servomotor (LED azul parpadeante).
- **T > 30 °C:** Se activa la alarma (buzzer).

## ⚙️ Componentes principales
- Arduino UNO / Mega
- Sensor DHT11 (temperatura y humedad)
- Sensor LDR (luz ambiental)
- Lector RFID
- Keypad 4x4
- LCD 16x2
- Ventilador (12V con relé)
- Servo motor
- Buzzer

## 📁 Estructura del proyecto
/proyecto_final
- InformeArquitecturaComputacional.docx → Informe del proyecto
- proyecto_final.ino → Código completo del sistema
- Doxyfile → Archivo de configuración para Doxygen
- /doc/html/ → Documentación generada automáticamente


## 📘 Documentación del código
La documentación completa se encuentra en la carpeta `/doc/html/`.
Abre el archivo `index.html` directamente en tu navegador

## 🧠 Autor
**Edward Esteban Dávila Salazar**
**Juan Jose Hurtado Molina**
Universidad del Cauca – Arquitectura Computacional  
Popayán, Colombia – Octubre 2025
