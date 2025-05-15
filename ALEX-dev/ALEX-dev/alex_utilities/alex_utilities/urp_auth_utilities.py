import hashlib

def get_encoded_room_id(access_token:str) -> str:
    """Using the access token, calcuate the specific encoded room id for the clients to connect"""
    encoded_room = hashlib.sha256(access_token.encode()).hexdigest()
    return encoded_room
