#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TRX Bridge pour QEMU Calypso
Pont entre osmo-bts-trx et QEMU avec clock indications
"""

import socket
import threading
import time
import select
import sys

class TRXBridge:
    def __init__(self):
        # Ports par defaut OsmoTRX (BTS -> Bridge)
        self.bts_ctrl_port = 5700
        self.bts_data_tx_port = 5701
        self.bts_data_rx_port = 5702
        
        # Ports QEMU
        self.qemu_ctrl_port = 4729
        self.qemu_ctrl_recv_port = 4829
        
        # Etat
        self.running = False
        self.frame_number = 0
        self.trx_powered_on = False
        self.sockets = {}
        
        print("=" * 70)
        print("TRX Bridge - Pont osmo-bts-trx <-> QEMU Calypso")
        print("=" * 70)
        
    def setup_sockets(self):
        """Creer tous les sockets UDP"""
        try:
            # Socket CTRL: Recevoir du BTS (port 5700)
            self.sockets['ctrl_recv'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sockets['ctrl_recv'].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sockets['ctrl_recv'].bind(('127.0.0.1', self.bts_ctrl_port))
            self.sockets['ctrl_recv'].setblocking(False)
            print(f"[OK] Ecoute BTS sur port {self.bts_ctrl_port}")
            
            # Socket CTRL: Envoyer au BTS
            self.sockets['ctrl_send'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # Socket CTRL: Envoyer a QEMU (port 4729)
            self.sockets['qemu_ctrl'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"[OK] Envoi vers QEMU port {self.qemu_ctrl_port}")
            
            # Socket CTRL: Recevoir de QEMU (port 4829)
            self.sockets['qemu_recv'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sockets['qemu_recv'].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sockets['qemu_recv'].bind(('127.0.0.1', self.qemu_ctrl_recv_port))
            self.sockets['qemu_recv'].setblocking(False)
            print(f"[OK] Ecoute QEMU sur port {self.qemu_ctrl_recv_port}")
            
            # Socket pour les clock indications vers BTS (port 5702)
            self.sockets['clock_send'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"[OK] Clock indications vers port {self.bts_data_rx_port}")
            
            print("[OK] Tous les sockets crees")
            return True
            
        except Exception as e:
            print(f"[ERREUR] Impossible de creer les sockets: {e}")
            return False
    
    def handle_ctrl_command(self, cmd, addr):
        """Traiter les commandes de controle du BTS"""
        try:
            cmd_str = cmd.decode('utf-8').strip()
            
            # Afficher la commande recue
            print(f"[BTS->Bridge] {cmd_str}")
            
            # Parser la commande
            parts = cmd_str.split()
            if len(parts) < 2 or parts[0] != 'CMD':
                return
            
            command = parts[1]
            params = parts[2:] if len(parts) > 2 else []
            
            # Gerer les commandes speciales
            if command == 'RFMUTE':
                # Accepter RFMUTE sans le transmettre a QEMU
                response = f"RSP RFMUTE 0 {' '.join(params)}"
                self.send_response_to_bts(response)
                return
            
            elif command == 'SETFORMAT':
                # Rejeter SETFORMAT (forcer mode legacy)
                response = "RSP ERR 1"
                self.send_response_to_bts(response)
                return
            
            elif command == 'POWERON':
                self.trx_powered_on = True
                print("\n" + "=" * 70)
                print(">>> TRX POWERON - DEMARRAGE DES CLOCK INDICATIONS <<<")
                print("=" * 70 + "\n")
                # Transmettre a QEMU
                self.forward_to_qemu(cmd)
                return
            
            elif command == 'POWEROFF':
                self.trx_powered_on = False
                print("\n" + "=" * 70)
                print(">>> TRX POWEROFF - ARRET DES CLOCK INDICATIONS <<<")
                print("=" * 70 + "\n")
                self.frame_number = 0
                # Transmettre a QEMU
                self.forward_to_qemu(cmd)
                return
            
            # Transmettre toutes les autres commandes a QEMU
            self.forward_to_qemu(cmd)
            
        except Exception as e:
            print(f"[ERREUR] Traitement commande: {e}")
    
    def forward_to_qemu(self, cmd):
        """Transmettre la commande a QEMU"""
        try:
            self.sockets['qemu_ctrl'].sendto(cmd, ('127.0.0.1', self.qemu_ctrl_port))
            print(f"[Bridge->QEMU] {cmd.decode('utf-8').strip()}")
        except Exception as e:
            print(f"[ERREUR] Envoi a QEMU: {e}")
    
    def send_response_to_bts(self, response):
        """Envoyer une reponse au BTS"""
        try:
            msg = response.encode('utf-8')
            self.sockets['ctrl_send'].sendto(msg, ('127.0.0.1', self.bts_ctrl_port))
            print(f"[Bridge->BTS] {response}")
        except Exception as e:
            print(f"[ERREUR] Envoi reponse au BTS: {e}")
    
    def send_clock_indication(self):
        """Envoyer une indication d'horloge au BTS"""
        if not self.trx_powered_on:
            return
        
        msg = f"IND CLOCK {self.frame_number}"
        try:
            # Envoyer sur le port TRXD RX (5702)
            self.sockets['clock_send'].sendto(
                msg.encode('utf-8'), 
                ('127.0.0.1', self.bts_data_rx_port)
            )
            
            # Afficher toutes les 217 frames (~1 seconde)
            if self.frame_number % 217 == 0:
                print(f"[Clock] Frame {self.frame_number} (temps: ~{self.frame_number * 4.615 / 1000:.1f}s)")
            
            # Incrementer le frame number (modulo hyperframe GSM)
            self.frame_number = (self.frame_number + 1) % 2715648
            
        except Exception as e:
            print(f"[ERREUR] Clock indication: {e}")
    
    def clock_thread(self):
        """Thread pour envoyer les clock indications toutes les 4.615ms"""
        print("[Thread Clock] Demarre")
        
        while self.running:
            if self.trx_powered_on:
                self.send_clock_indication()
            time.sleep(0.004615)  # 4.615ms = duree d'une trame GSM
    
    def listen_qemu_responses(self):
        """Ecouter les reponses de QEMU et les transmettre au BTS"""
        print(f"[Thread QEMU] Ecoute sur port {self.qemu_ctrl_recv_port}")
        
        while self.running:
            try:
                ready = select.select([self.sockets['qemu_recv']], [], [], 0.1)
                if ready[0]:
                    data, addr = self.sockets['qemu_recv'].recvfrom(1024)
                    response = data.decode('utf-8').strip()
                    print(f"[QEMU->Bridge] {response}")
                    
                    # Transmettre au BTS
                    self.sockets['ctrl_send'].sendto(data, ('127.0.0.1', self.bts_ctrl_port))
                    print(f"[Bridge->BTS] {response}")
                    
            except socket.error:
                pass
            except Exception as e:
                if self.running:
                    print(f"[ERREUR] Reception QEMU: {e}")
    
    def ctrl_thread(self):
        """Thread pour gerer les commandes de controle du BTS"""
        print(f"[Thread Control] Ecoute sur port {self.bts_ctrl_port}")
        
        while self.running:
            try:
                ready = select.select([self.sockets['ctrl_recv']], [], [], 0.1)
                if ready[0]:
                    data, addr = self.sockets['ctrl_recv'].recvfrom(1024)
                    self.handle_ctrl_command(data, addr)
                    
            except socket.error:
                pass
            except Exception as e:
                if self.running:
                    print(f"[ERREUR] Reception BTS: {e}")
    
    def run(self):
        """Demarrer le bridge"""
        # Creer les sockets
        if not self.setup_sockets():
            print("[ERREUR] Impossible de demarrer le bridge")
            return 1
        
        self.running = True
        
        # Demarrer les threads
        threads = [
            threading.Thread(target=self.clock_thread, daemon=True, name="Clock"),
            threading.Thread(target=self.ctrl_thread, daemon=True, name="Control"),
            threading.Thread(target=self.listen_qemu_responses, daemon=True, name="QEMU"),
        ]
        
        for t in threads:
            t.start()
        
        print("\n" + "=" * 70)
        print("BRIDGE ACTIF - En attente de commandes")
        print("Appuyez sur Ctrl+C pour arreter")
        print("=" * 70 + "\n")
        
        try:
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n\n" + "=" * 70)
            print("Arret du bridge...")
            print("=" * 70)
            self.running = False
            
            # Attendre que les threads se terminent
            for t in threads:
                t.join(timeout=1)
            
            # Fermer les sockets
            for sock in self.sockets.values():
                try:
                    sock.close()
                except:
                    pass
            
            print("[OK] Bridge arrete proprement\n")
            return 0

def main():
    bridge = TRXBridge()
    return bridge.run()

if __name__ == '__main__':
    sys.exit(main())
