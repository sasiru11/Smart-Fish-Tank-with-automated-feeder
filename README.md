# Title
### Smart Fish Tank with Automatic Feeder
# Introduction 
Aquariums require constant attention to maintain a healthy environment for fish, including monitoring water quality, temperature, and feeding schedules. However, busy lifestyles make it challenging for aquarium owners to manage these tasks consistently. 
This system integrates sensors to measure water temperature and turbidity, an automated feeder to dispense food at scheduled intervals, and actuators to regulate the tank’s environment. The ESP32 microcontroller acts as the brain of the system, ensuring real-time monitoring and control. With features like potential mobile app and web app integration, users can easily keep track of their aquarium’s condition.
By reducing manual intervention, this smart fish tank not only enhances fish well-being but also simplifies aquarium maintenance, making it an ideal solution for both hobbyists and professionals.


![Image Alt](https://github.com/sasiru11/Smart-Fish-Tank-with-automated-feeder/blob/09ab06b6015e40c02f85ae48638871659e477116/Architecture%20diagram.png)


# Circuit Design 

 ![Image Alt](https://github.com/sasiru11/Smart-Fish-Tank-with-automated-feeder/blob/7c3608731ac642c57301baf60d72a194be4bf603/Design.jpg)
 
# Technologies used  
 ## 1. Microcontroller & Computing Platform
##### •	ESP32 – A powerful microcontroller with built-in Wi-Fi and Bluetooth, used to control sensors, actuators, and enable remote monitoring.
## 2. Sensors for Water Quality Monitoring
##### •	DS18B20 Temperature Sensor – Measures water temperature.
##### •	Turbidity Sensor – Monitors water clarity to detect contamination or dirt buildup.
## 3. Actuators for Automation
##### •	Relay-controlled Heater – Adjusts water temperature as needed.
##### •	Water Filter System – Operates when water becomes turbid.
##### •	Automatic Feeder – Dispenses fish food at scheduled intervals.
## 4. Communication & Data Processing
##### •	GPIO Pins – Used to connect sensors and actuators directly to the ESP32.
##### •	Wi-Fi (ESP32) – Enables remote monitoring and potential app integration.
## 5. User Interface
##### •	Web application and Mobile application

![Image Alt](https://github.com/sasiru11/Smart-Fish-Tank-with-automated-feeder/blob/538a72ae6e2e09eda3ca8ff6bd4a213e14d8cf4d/App%20Interface.jpg)

## 6. Software & Development Tools
##### •	VS Code
##### •	ESP IDF
##### •	Flutter
##### •	GitHub – Version control and code repository management.

# References 	
##### • https://www.researchgate.net/publication/345690191_Smart_Aquarium_Management_System
# Team & Mentor 
#### Nidukani O.G.H		2019/E/085
#### Gimhana M.K.G.C		2020/E/045
#### Janananda J.S.M.R.R.B	2020/E/057
#### Gedara S.G.S.N.S		2020/E/210

