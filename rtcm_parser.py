"""
@file rtcm_parser.py
@brief Parser minimal pour détecter les types de messages RTCM à partir des trames binaires.
"""

def detect_rtcm_message_type(data):
    """
    @brief Détecte les types de messages RTCM dans une trame.
    @param data Données binaires reçues (bytes).
    @return Liste des types détectés (entiers).
    """
    types = []
    i = 0
    while i < len(data) - 5:
        if data[i] == 0xD3:
            # Longueur indiquée sur 2 bits dans le 2e et 3e octet (10 bits)
            length = ((data[i + 1] & 0x03) << 8) | data[i + 2]
            if i + 3 + length > len(data):
                break  # Incomplet
            payload = data[i + 3:i + 3 + length]
            if len(payload) >= 3:
                msg_type = (payload[0] << 4) | (payload[1] >> 4)
                types.append(msg_type)
            i += 3 + length + 3  # +3 pour le CRC
        else:
            i += 1
    return types
