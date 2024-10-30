from collections import deque #implemented using doubly linked list, faster than accessing a normal list


message_q=deque()
def send(message):
    message_q.append(message) #O(1)
    print(f"Message queued {message}")
    return

def receive():
    if message_q:
        message=message_q.popleft()
        print("Message received" )
        return message
    