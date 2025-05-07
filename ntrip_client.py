"""
@file ntrip_client.py
@brief Module pour connexion NTRIP et réception de données RTCM.
"""

import socket
import base64

class NtripClient:
    """
    @class NtripClient
    @brief Client NTRIP simple pour recevoir des corrections RTCM.
    """
    def __init__(self, server, port, mountpoint, username, password, callback):
        """
        @brief Initialise le client NTRIP.
        @param server Adresse IP ou nom du serveur.
        @param port Port du serveur.
        @param mountpoint Point de montage.
        @param username Nom d'utilisateur NTRIP.
        @param password Mot de passe NTRIP.
        @param callback Fonction appelée pour chaque paquet reçu.
        """
        self.server = server
        self.port = port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.callback = callback
        self.running = False

    def run(self):
        """
        @brief Démarre la connexion NTRIP et transmet les données au callback.
        """
        print(">> Try to connect.")
        request = (
            f"GET /{self.mountpoint} HTTP/1.0\r\n"
            f"Host: {self.server}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP client/1.0\r\n"
        )

        if self.username and self.password:
            auth = base64.b64encode(f"{self.username}:{self.password}".encode('utf-8')).decode('utf-8')
            request += f"Authorization: Basic {auth}\r\n"

        request += "\r\n"

        print(">> Connected to NTRIP caster successfully.")

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.server, self.port))
            s.sendall(request.encode('utf-8'))

            response = s.recv(4096)
            if b"200 OK" not in response:
                raise ConnectionError("NTRIP connection failed. Server response: " + response.decode(errors="ignore"))

            self.running = True
            while self.running:
                data = s.recv(1024)
                if not data:
                    break
                self.callback(data)

    def stop(self):
        """
        @brief Arrête la réception de données.
        """
        self.running = False
