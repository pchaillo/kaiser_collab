## Polhemus Connexion

 Contient l'ensemble des scripts afin de se connecter la capteur Polhemus liberty

## Python

Contient tous les scripts pour connecter le capteur et stocker les trames dans un ficher texte.
Contient aussi une scène SOFA, avec un exemple de python controller pour importer les données dans SOFA

## ROS / ROS2 

## Matlab

Juste un copié-collé de code existant : non testé.




## Allow usb port

To launch the script, you have to do the following commands to open the usb port. You have to do that every time you reconnect the sensor as te device change.

```console
~ $ lsusb
Bus 001 Device 014: ID 0f44:ff20 Polhemus Liberty 2.0
~ $ lsusb | grep 014 | sed -nr 's|Bus (.*) Device ([^:]*):.*|/dev/bus/usb/\1/\2|p'
/dev/bus/usb/001/014
~ $ sudo chmod 777 /dev/bus/usb/001/014
```