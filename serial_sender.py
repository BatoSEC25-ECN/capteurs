"""
@file serial_sender.py
@brief Module pour envoyer les données vers un port série.
"""

import serial

class SerialSender:
    """
    @class SerialSender
    @brief Gère l'envoi de données RTCM vers un port série.
    """
    def __init__(self, port, baudrate=115200):
        """
        @brief Initialise l'objet SerialSender.
        @param port Nom du port série.
        @param baudrate Débit de communication série.
        """
        self.port_name = port
        self.baudrate = baudrate
        self.serial_port = None

    def open(self):
        """
        @brief Ouvre le port série.
        """
        self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=1)

    def send(self, data):
        """
        @brief Envoie des données sur le port série.
        @param data Données à envoyer (bytes).
        """
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(data)

    def close(self):
        """
        @brief Ferme le port série.
        """
        if self.serial_port:
            self.serial_port.close()
