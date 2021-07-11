# Autores 
- Aguirre, Emanuel
- Fernandez, Gonzalo
- Portugal Rios, Julian
- Russo, Pablo

# Motor DC / PID Control
Como soporte para el trabajo final de ingeniería "REINGENIERÍA DE UN SISTEMA DE RESCATE DE PERSONAS EN AMBIENTES ACUÁTICOS PARA EMBARCACIONES (Aguirre, Russo)"
se diseñará un sistema capaz de ser utilizado en el ensayo de las turbinas diseñadas y construidas como parte del mencionado trabajo.

El algoritmo de control será responsable de recibir un valor de velocidad angular  (ω [RPM]) almacenado en memoria del MCU,
comenzará el giro del motor desde un duty cicle del 0% para la señal de control modulada por ancho de pulsos, acelerando el motor hasta la velocidad fijada. 
Luego de un cierto tiempo $t$ se actualizará la velocidad objetivo, y el sistema deberá actualizar el ciclo de trabajo hasta un cierto valor. 

Mediante el uso de los conocimientos adquiridos en el curso de Instrumentación y Control, se pretende:

- Implementar la medición del parámetro: Velocidad Angular Medida (ω [RPM]).
- Implementar un sistema de control PID discreto sobre la plataforma arduino.

# Hardware
Se utilizará un microcontrolador ESP32, SoC de la familia Tensilica Xtensa LX6 160 MHz, del fabricante Espressif.
Se dispone de un motor 775-24v, de escobillas, el cual posee un rotor, que será nuestra planta y su velocidad la variable a controlar.

# Pruebas de funcionamiento
- https://youtu.be/mapwY9u-rUo
- https://youtu.be/vMiuXQFB3TM
- https://youtu.be/jQIFR5dAHVU
