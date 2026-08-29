"""timdr_robot/bridges — mosty integracyjne do zewnetrznych magistrali
(ROS2, MQTT, OPC-UA).
================================================================================
**WSZYSTKIE MOSTY W TYM PAKIECIE SA STUBAMI KONTRAKTU, NIE DZIALAJACYMI
INTEGRACJAMI.** Zaden z nich nie laczy sie z prawdziwym brokerem/node'em/
serwerem. Kazdy uzywa DEFENSYWNEGO importu opcjonalnej biblioteki (wzorzec
z fusion-tools/api.py dla h5py: `try: import X; except Exception: X=None`)
- jesli biblioteka (rclpy / paho-mqtt / asyncua) nie jest zainstalowana,
  most dziala w trybie "dry run": zapisuje, CO by opublikowal, do
  `published_log`, ale nic nie wysyla. To pozwala napisac i przetestowac
  reszte integracji (kod wywolujacy `bridge.publish_status(event)`) zanim
  jakakolwiek prawdziwa biblioteka/magistrala jest dostepna w srodowisku.

Cel: pokazac DOKLADNY ksztalt kontraktu (jakie metody, jakie argumenty,
jaki format wiadomosci), zeby podlaczenie prawdziwej biblioteki pozniej
bylo wypelnieniem TODO w jednym miejscu (`_publish_real()`), a nie
przepisywaniem calej reszty kodu.
"""
