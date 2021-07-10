# Autores 
Aguirre, Emanuel
Fernandez, Gonzalo
Portugal Rios, Julian
Russo, Pablo

# Motor DC / PID Control
Como soporte para el trabajo final de ingeniería "REINGENIERÍA DE UN SISTEMA DE RESCATE DE PERSONAS EN AMBIENTES ACUÁTICOS PARA EMBARCACIONES (Aguirre, Russo)"
se diseñará un sistema capaz de ser utilizado en el ensayo de las turbinas diseñadas y construidas como parte del mencionado trabajo.

El algoritmo de control será responsable de recibir un valor de velocidad angular ($\omega_0$ [RPM]) almacenado en memoria del MCU,
comenzará el giro del motor desde un duty cicle del 0% para la señal de control modulada por ancho de pulsos, acelerando el motor hasta la velocidad fijada. 
Luego de un cierto tiempo $t$ se actualizará la velocidad objetivo, y el sistema deberá actualizar el ciclo de trabajo hasta un cierto valor. 

Mediante el uso de los conocimientos adquiridos en el curso de Instrumentación y Control, se pretende:

 Pre-fijar en el sistema de control una serie de valores de velocidad angular en RPM.
Implementar la medición del parámetro: Velocidad Angular Medida [$\omega_m$] 

# Hardware
Se dispone de un motor 775-24v, de escobillas, el cual posee un rotor, que será nuestra planta y su velocidad la variable a controlar.