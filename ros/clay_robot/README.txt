Curso: EL5206 Laboratorio de Inteligencia Computacional y Robotica
Actividad: Introducci�n a Stage

A continuaci�n se detalla el contenido de Tutorial.zip

I. 	Carpeta scripts:

1. 	Plantilla general: stage_controller.py 
	Descripci�n: controlador de ejemplo. A partir de esta plantilla se pueden desarrollar todas las actividades del curso.
	Al cargar el script se inicia un ciclo en que el robot avanza con velocidad constante; sin embargo, en el c�digo tambi�n
	hay ejemplos (comentados) de uso de funciones de la librer�a robot_utilities.

2. 	Librer�a: robot_utilities.py
	Descripci�n: Definici�n de la clase Robot, incluye funciones nSteps, moveTill, reset_perim, angle_tf, etc

II. 	Carpeta worlds:

1.	world1.world
	Descripci�n: carga escenario 'empty.png' con un robot y 3 bloques (2 grandes y 1 peque�o)
	Los de mayor tama�o pueden ser utilizados como obst�culos.
	Los de menor tama�o fueron pensados como indicadores de meta (�til en el campos de potencial)

2.	world_cave.world
	Descripci�n: carga escenario 'cave.png' con un robot y un bloque peque�o.

Nota: 	Los archivos .world pueden ser modificados para cambiar la configuraci�n inicial de los bloques y del robot,
	o bien para incluir m�s elementos.
	Tener en cuenta que en que estos cambios deben incluirse en el c�digo python. Ejemplo:
	Si la posici�n inicial del robot declarado en world1.file es pose [ -2 -2 0 90] y se cambia a [4 4 0 180],
	Las siguientes l�neas del script en python
		init_x = -2.0
		init_y = -2.0
		init_angle = 90.0
	deben cambiar a:
		init_x = 4.0
		init_y = 4.0
		init_angle = 180


III.	Tutorial.pdf
	Descripci�n: instrucciones de instalaci�n y ejecuci�n + ejercicio introductorio.