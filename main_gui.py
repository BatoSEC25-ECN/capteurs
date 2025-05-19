"""
@file main_gui.py
@brief Interface graphique pour connecter à Centipede et envoyer les données RTCM à un port série.
@details Utilise les modules ntrip_client.py et serial_sender.py
"""
from tkinter.scrolledtext import ScrolledText
from rtcm_parser import detect_rtcm_message_type  # que tu vas créer ensuite
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports
import time 

from ntrip_client import NtripClient
from serial_sender import SerialSender

class RTCMForwarderApp:
    """
    @class RTCMForwarderApp
    @brief Classe principale pour l'application GUI.
    """
    def __init__(self, root):
        """
        @brief Initialise l'interface graphique.
        @param root Fenêtre Tkinter principale.
        """
        self.root = root
        self.root.title("RTCM Forwarder to Serial")

        # NTRIP parameters
        self.server_var = tk.StringVar()
        self.port_var = tk.StringVar(value="2101")
        self.mountpoint_var = tk.StringVar()
        self.username_var = tk.StringVar()
        self.password_var = tk.StringVar()

        # Serial port
        self.serial_port_var = tk.StringVar()
        self.serial_ports = []

        # Client and sender
        self.ntrip_client = None
        self.serial_sender = None
        self.running = False

        self.create_widgets()

    def create_widgets(self):
        """
        @brief Crée les widgets de l'interface graphique.
        """

        frame = ttk.Frame(self.root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frame, text="Server:").grid(row=0, column=0, sticky="e")
        ttk.Entry(frame, textvariable=self.server_var).grid(row=0, column=1)

        ttk.Label(frame, text="Port:").grid(row=1, column=0, sticky="e")
        ttk.Entry(frame, textvariable=self.port_var).grid(row=1, column=1)

        ttk.Label(frame, text="Mountpoint:").grid(row=2, column=0, sticky="e")
        ttk.Entry(frame, textvariable=self.mountpoint_var).grid(row=2, column=1)

        ttk.Label(frame, text="Username:").grid(row=3, column=0, sticky="e")
        ttk.Entry(frame, textvariable=self.username_var).grid(row=3, column=1)

        ttk.Label(frame, text="Password:").grid(row=4, column=0, sticky="e")
        ttk.Entry(frame, textvariable=self.password_var, show="*").grid(row=4, column=1)

        ttk.Button(frame, text="Refresh Serial Ports", command=self.refresh_serial_ports).grid(row=5, column=0, columnspan=2)

        self.serial_ports_combo = ttk.Combobox(frame, textvariable=self.serial_port_var, state="readonly")
        self.serial_ports_combo.grid(row=6, column=0, columnspan=2, pady=5)

        self.start_button = ttk.Button(frame, text="Start", command=self.start_forwarding)
        self.start_button.grid(row=7, column=0, pady=10)

        self.stop_button = ttk.Button(frame, text="Stop", command=self.stop_forwarding, state="disabled")
        self.stop_button.grid(row=7, column=1, pady=10)
        
        ttk.Label(frame, text="Serial Monitor / Logs:").grid(row=8, column=0, columnspan=2)
        self.log_box = ScrolledText(frame, width=60, height=10, state="disabled")
        self.log_box.grid(row=9, column=0, columnspan=2, pady=5)
        ttk.Label(frame, text="Fréquence d'envoi (Hz):").grid(row=5, column=0, sticky="e")
        self.freq_var = tk.StringVar(value="1.0")
        ttk.Entry(frame, textvariable=self.freq_var).grid(row=5, column=1)


    def refresh_serial_ports(self):
        """
        @brief Met à jour la liste des ports séries disponibles.
        """
        self.serial_ports = [port.device for port in serial.tools.list_ports.comports()]
        self.serial_ports_combo["values"] = self.serial_ports
        if self.serial_ports:
            self.serial_ports_combo.current(0)

    def start_forwarding(self):
        """
        @brief Démarre la connexion Centipede et l'envoi vers le port série.
        """
        if not self.serial_port_var.get():
            messagebox.showerror("Error", "Please select a serial port.")
            return

        try:
            freq = float(self.freq_var.get())
            if freq <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Erreur", "La fréquence doit être un nombre positif.")
            return
        try:
            self.serial_sender = SerialSender(self.serial_port_var.get())
            self.serial_sender.open()

            # self.ntrip_client = NtripClient(
            #     server=self.server_var.get(),
            #     port=int(self.port_var.get()),
            #     mountpoint=self.mountpoint_var.get(),
            #     username=self.username_var.get(),
            #     password=self.password_var.get(),
            #     callback=self.serial_sender.send
            # )
            self._last_send_time = 0  # Initialise le timer

            def process_data(data):
                now = time.time()
                if now - self._last_send_time >= 1.0 / freq:
                    self._last_send_time = now
                    self.serial_sender.send(data)
                    hex_str = " ".join(f"{b:02X}" for b in data)
                    self.log(f"[RAW] {hex_str}")

                    types = detect_rtcm_message_type(data)
                    for t in types:
                        self.log(f"[RTCM] Type {t} détecté")


            self.ntrip_client = NtripClient(
                server=self.server_var.get(),
                port=int(self.port_var.get()),
                mountpoint=self.mountpoint_var.get(),
                username=self.username_var.get(),
                password=self.password_var.get(),
                callback=process_data
            )


            self.ntrip_thread = threading.Thread(target=self.ntrip_client.run, daemon=True)
            self.ntrip_thread.start()

            self.running = True
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def stop_forwarding(self):
        """
        @brief Arrête la connexion Centipede et ferme le port série.
        """
        if self.running:
            self.ntrip_client.stop()
            self.serial_sender.close()
            self.running = False
            self.start_button.config(state="normal")
            self.stop_button.config(state="disabled")
    
    def log(self, msg):
        """
        @brief Affiche un message dans la zone de log.
        @param msg Message texte à afficher.
        """
        self.log_box.config(state="normal")
        self.log_box.insert(tk.END, msg + "\n")
        self.log_box.see(tk.END)
        self.log_box.config(state="disabled")


if __name__ == "__main__":
    root = tk.Tk()
    app = RTCMForwarderApp(root)
    root.mainloop()
