# Title
Smart Fish Tank with Automatic Feeder
# Introduction 
Aquariums require constant attention to maintain a healthy environment for fish, including monitoring water quality, temperature, and feeding schedules. However, busy lifestyles make it challenging for aquarium owners to manage these tasks consistently. 
This system integrates sensors to measure water temperature and turbidity, an automated feeder to dispense food at scheduled intervals, and actuators to regulate the tank’s environment. The ESP32 microcontroller acts as the brain of the system, ensuring real-time monitoring and control. With features like an LCD display and potential mobile app integration, users can easily keep track of their aquarium’s condition.
By reducing manual intervention, this smart fish tank not only enhances fish well-being but also simplifies aquarium maintenance, making it an ideal solution for both hobbyists and professionals.

# Circuit Design  
# Technologies used  
 1. Microcontroller & Computing Platform
•	ESP32 – A powerful microcontroller with built-in Wi-Fi and Bluetooth, used to control sensors, actuators, and enable remote monitoring.
2. Sensors for Water Quality Monitoring
•	DS18B20 Temperature Sensor – Measures water temperature.
•	Turbidity Sensor – Monitors water clarity to detect contamination or dirt buildup.
3. Actuators for Automation
•	Relay-controlled Heater – Adjusts water temperature as needed.
•	Water Filter System – Operates when water becomes turbid.
•	Automatic Feeder – Dispenses fish food at scheduled intervals.
4. Communication & Data Processing
•	I2C Protocol – Used for sensor communication (e.g., LCD display).
•	GPIO Pins – Used to connect sensors and actuators directly to the ESP32.
•	Wi-Fi (ESP32) – Enables remote monitoring and potential app integration.
5. User Interface & Display
•	LCD Display (with I2C module) – Shows real-time sensor data for easy monitoring.
•	Mobile App (Planned) – Could be developed in the future using Flutter or another framework for remote access.
6. Software & Development Tools
•	Platform IO
•	ESP IDF
•	GitHub – Version control and code repository management.

# References 	
# Team & Mentor 
Nidukani O.G.H		2019/E/085
Gimhana M.K.G.C		2020/E/045
Janananda J.S.M.R.R.B	2020/E/057
Gedara S.G.S.N.S		2020/E/210


