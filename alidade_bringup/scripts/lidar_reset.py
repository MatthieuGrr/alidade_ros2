#!/usr/bin/env python3
"""Reveille le RPLIDAR C1 avant que le driver ne lui parle. Sans ROS.

    lidar_reset.py [port]        # /dev/rplidar par defaut

ATTENTION : CE SCRIPT NE SUFFIT PAS. Il leve un etat du capteur, ce qui est
necessaire, mais le LiDAR ne demarre toujours pas de facon fiable. Le diagnostic
complet est ci-dessous ; le correctif, lui, reste a faire et il est ailleurs.

LE SYMPTOME. Apres une mise sous tension, le C1 repond parfaitement aux
commandes d'etat -- numero de serie, firmware, sante a 0 -- ACCEPTE la commande
de balayage, renvoie son descripteur en 3 ms, puis le driver abandonne :

    [rplidar] Cannot start scan: '80008002'
    [rplidar] Failed to set scan mode

Ce message a fait soupconner, depuis septembre 2026, le support mecanique,
l'alimentation USB, puis une alimentation dediee et un passage en UART direct.
Aucun n'etait en cause.

LA CAUSE, mesuree des deux cotes le 6 octobre 2026, capteur sur l'UART du Pi :

    premier octet de donnees apres la commande  2102 a 2335 ms
    delai que le driver accorde avant d'abandonner  2042 a 2504 ms

C'est une course, et le driver la perd le plus souvent. La marge va de -0.3 a
+0.4 s selon les tirages, ce qui explique aussi pourquoi les relances finissaient
parfois par passer, au bout de 15 s ou de 7 minutes.

CE QUE FAIT CE SCRIPT, et pourquoi c'est insuffisant. Sans RESET prealable
(0xA5 0x40) le capteur n'emet RIEN, meme en attendant 3 s : verifie sur trois
essais consecutifs, 0 octet. Apres un reset il balaie toujours, et la fenetre ne
se referme pas -- a 0, 2, 5, 10 et 20 s d'attente, meme delai de 2.2 s et
~50 000 octets en 4 s. Le reset est donc NECESSAIRE. Il ne touche simplement pas
a la course ci-dessus.

TROIS VARIANTES ESSAYEES, TOUTES INSUFFISANTES :
  - reset seul, puis on rend la main              -> 57 echecs
  - laisser le moteur TOURNER pour le driver      -> 0 echec mais 0 scan : il
    ouvre un port qui deverse deja des donnees, les prend pour la reponse a ses
    commandes d'etat, et se bloque (SIGKILL necessaire)
  - lancer le moteur puis l'arreter avant de rendre la main -> 23 echecs ; le
    rotor s'arrete pendant les 4 a 5 s que met le driver a faire son dialogue

Aucune n'atteint la vraie contrainte : il faudrait que le rotor tourne a
l'instant PRECIS ou le driver demande le balayage, et cet instant n'est pas
accessible de l'exterieur.

LE CORRECTIF RESTE A FAIRE : allonger le delai du driver. Le paquet apt
ros-jazzy-rplidar-ros ne l'expose pas, c'est une constante du SDK Slamtec
(2000 ms). Il faut cloner Slamtec/rplidar_ros, porter ce delai a 5 s et compiler
dans le workspace -- system/sources.sh est fait pour epingler ce genre de depot.

Le C1 n'a pas de pilotage moteur separe : sa datasheet precise que « the motor
can't start or stop alone, and its working state depends on the laser scan
command ». Il n'y a donc pas de broche a actionner.
"""
import sys
import time

import serial

PORT = '/dev/rplidar'
BAUD = 460800            # fixe sur le C1
STOP = 0x25
RESET_CMD, SCAN = 0x40, 0x20
REVEIL = 2.5             # s, le capteur se presente puis se tait
LANCEMENT = 4.0          # s, de quoi laisser le moteur atteindre son regime


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    try:
        lien = serial.Serial(port, BAUD, timeout=2)
    except serial.SerialException as e:
        # Pas une erreur fatale pour le service : sans LiDAR la pile ne
        # demarrera pas de toute facon, et le dire est plus utile qu'un echec.
        print(f'lidar_reset : {port} inaccessible ({e})', file=sys.stderr)
        return 0
    with lien:
        lien.write(bytes([0xA5, STOP]))
        lien.flush()
        time.sleep(0.3)
        lien.reset_input_buffer()
        lien.write(bytes([0xA5, RESET_CMD]))
        lien.flush()
        time.sleep(REVEIL)
        # Au reveil le capteur annonce son modele ; le lire vide le tampon et
        # confirme qu'il a bien redemarre.
        bavard = lien.read(lien.in_waiting or 1)
        lien.reset_input_buffer()

        # Puis on LANCE le moteur, et on le laisse tourner : pas de STOP avant
        # de fermer. C'est tout l'interet de ce script.
        lien.write(bytes([0xA5, SCAN]))
        lien.flush()
        lien.timeout = 0.5
        octets, debut, premier = 0, time.monotonic(), None
        while time.monotonic() - debut < LANCEMENT:
            bloc = lien.read(4096)
            if bloc and premier is None:
                premier = time.monotonic() - debut
            octets += len(bloc)

        # Puis on l'arrete et on VIDE le tampon : le driver doit trouver un port
        # muet pour son dialogue d'etat. Le rotor, lui, met plusieurs secondes a
        # s'arreter -- c'est sur cette erre que le driver demarrera.
        lien.write(bytes([0xA5, STOP]))
        lien.flush()
        time.sleep(0.4)
        lien.reset_input_buffer()
        lien.timeout = 0.2
        while lien.read(4096):
            pass
    texte = bavard.decode('ascii', 'replace').strip().replace('\n', ' ')
    delai = f'{premier*1000:.0f} ms' if premier else 'JAMAIS'
    print(f'lidar_reset : {port} reveille ({len(bavard)} o) {texte} | '
          f'moteur lance, premier echo a {delai}, {octets} octets')
    return 0


if __name__ == '__main__':
    sys.exit(main())
