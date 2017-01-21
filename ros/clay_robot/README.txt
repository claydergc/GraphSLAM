Curso: EL5206 Laboratorio de Inteligencia Computacional y Robotica
Actividad: Introducción a Stage

A continuación se detalla el contenido de Tutorial.zip

I. 	Carpeta scripts:

1. 	Plantilla general: stage_controller.py 
	Descripción: controlador de ejemplo. A partir de esta plantilla se pueden desarrollar todas las actividades del curso.
	Al cargar el script se inicia un ciclo en que el robot avanza con velocidad constante; sin embargo, en el código también
	hay ejemplos (comentados) de uso de funciones de la librería robot_utilities.

2. 	Librería: robot_utilities.py
	Descripción: Definición de la clase Robot, incluye funciones nSteps, moveTill, reset_perim, angle_tf, etc

II. 	Carpeta worlds:

1.	world1.world
	Descripción: carga escenario 'empty.png' con un robot y 3 bloques (2 grandes y 1 pequeño)
	Los de mayor tamaño pueden ser utilizados como obstáculos.
	Los de menor tamaño fueron pensados como indicadores de meta (útil en el campos de potencial)

2.	world_cave.world
	Descripción: carga escenario 'cave.png' con un robot y un bloque pequeño.

Nota: 	Los archivos .world pueden ser modificados para cambiar la configuración inicial de los bloques y del robot,
	o bien para incluir más elementos.
	Tener en cuenta que en que estos cambios deben incluirse en el código python. Ejemplo:
	Si la posición inicial del robot declarado en world1.file es pose [ -2 -2 0 90] y se cambia a [4 4 0 180],
	Las siguientes líneas del script en python
		init_x = -2.0
		init_y = -2.0
		init_angle = 90.0
	deben cambiar a:
		init_x = 4.0
		init_y = 4.0
		init_angle = 180


III.	Tutorial.pdf
	Descripción: instrucciones de instalación y ejecución + ejercicio introductorio.