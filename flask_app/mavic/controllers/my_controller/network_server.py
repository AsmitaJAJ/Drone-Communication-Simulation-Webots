import lora_simulator
import os
import requests  #for http requests
def receive_sos():
    sos_message=lora_simulator.receive()
    print(f"Receiver sos: {sos_message}")
    return sos_message
SERVER_URL = os.getenv("SERVER_URL", "http://localhost:5001/receive_sos")
def forward_sos(sos_message):
    resp=requests.post(SERVER_URL, json={'message': sos_message})
    print("Forwarded to app server") 
    

if __name__=="__main__":
    
    while True:
        message=receive_sos()
        if message:
            forward_sos(message)
            
            