import sys
sys.path.append('/opt/ue4_ws/PythonAPI/carla/dist/carla-0.9.13-py2.7-linux-x86_64.egg')
sys.path.append('/opt/ue4_ws/PythonAPI/carla')
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
print(client.get_client_version())
print(client.get_server_version())  
