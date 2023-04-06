# Ben-Gurion university's final project of the "Digital Computer Structure" course report.
Radar Detector System
</br> </br>
## Definition and purpose of the project
The project's purpose is to create an MCU-based radar system for detecting and monitoring objects in space using an ultrasonic distance meter and a servo motor. 
The system will scan a 180-degree section using servo motor movement, with a measurement range of 2m-4.5m. The angular movement control of the servo motor is based on pulse-width modulation (PWM).</br>
</br>
The project uses full-duplex asynchronous serial communication in the RS-232 standard between the KL25Z controller and the PC.
</br>
• Physical layer: full-duplex, RS-232 protocol</br>
• link layer: using a UART peripheral component in the KL25Z controller and a communication controller on the computer side.</br>
</br>
The project includes a graphical user interface (UI) for communication between the controller and the computer.</br>
</br>
The system includes three parts:</br>
• Radar Detector System - monitoring objects in space in the range of 180 degrees at a specified distance.</br>
• Telemeter - dynamically displays real-time distance measurements from the distance sensor.</br>
• Script Mode - activates the entire system according to a script file containing pre-defined high-level commands.</br>
</br>
## User Interface
The main menu of the user interface includes three modes of operation: Scan (Radar Detector System), Script mode, and Telemeter mode.</br>

![image](https://user-images.githubusercontent.com/62146535/230337050-a4daf6f1-d557-4360-8e7d-2cba3becf3ea.png)

### Telemeter

![image](https://user-images.githubusercontent.com/62146535/230337211-783bf692-6ddf-4158-9880-474d0d441068.png)

In the Telemeter menu, you can select an angle for scanning, and pressing the Start button will display the following screen:

![image](https://user-images.githubusercontent.com/62146535/230337412-6dffabe5-a2a1-4106-9f20-cd12be2f6105.png)

On this screen, you can see the dynamic monitoring of objects detected by the distance sensor at the chosen angle. The measured distance is displayed in centimeters.

### Scan (Radar Detector System)

![image](https://user-images.githubusercontent.com/62146535/230337713-e60ea843-13b4-46a8-b4ca-190333e8aa83.png)


In the Scan menu, you can set a masking distance, which is a value (in centimeters), beyond which any measured distance is considered outside the range and is not considered. After pressing the Start button, the following screen will appear:

![image](https://user-images.githubusercontent.com/62146535/230337836-a1f69955-402e-4b80-892f-ed6eb9375590.png)

When this screen is displayed, the sensor will move 180 degrees and monitor objects within that range. If the sensor detects an object, it will be marked in red.

### Script Mode

![image](https://user-images.githubusercontent.com/62146535/230338062-9cdfb893-13c9-40ba-9910-3fcc98066030.png)


In this menu, you can select a text file stored on the computer and send it to the controller (up to three files can be sent). The file will be sent to the controller by clicking on the send button, and it will be stored on the controller side. A confirmation message will be sent from the controller to the computer.
After sending the file, the file name shows in the box below. You can select the file in this box and run it in the controller by pressing the execute button. The commands will be translated according to the following table:

![image](https://user-images.githubusercontent.com/62146535/230338354-af7b90b6-8fc9-425d-a05a-007785ba24ab.png)

## General description of the hardware and software performance

### Software

The software used in this project is divided into two parts: the computer side and the controller side. The computer side software is written in Python, and we utilized the PySimpleGUI library to create the user interface. We also used the PySerial library to handle communication between the computer and the controller.</br>

We have created the following functions to facilitate communication between the software on the computer side and the controller:</br>
• send_to_controller: sends a string to the controller</br>
• send_file_to_controller: sends a text file to the controller</br>
• send_file_content_to_controller: sends the content of a text file to the controller</br>
• uart_cfg: configures communication parameters with the controller</br>
• start_telemeter: receives information from the controller and initializes Telemeter mode</br>
• telemeter_draw: generates the graphical display for Telemeter mode</br>
• draw_telemeter: receives information from the controller and initializes Radar Detector System mode</br>
• start_sonar: receives information from the controller and initializes Radar Detector System mode</br>
• draw: produces the graphical display for Radar Detector System mode</br>
• calc_points: calculates two points based on radius and angle</br>
• execute_thread: receives messages from the controller when it is in Script Mode.</br>

The software on the controller side is organized into layers and based on FSM. We created a data structure to maintain the files stored in the controller,</br> which includes the following fields:</br>
• files_number: a variable that holds the number of files sent from the computer to the controller.</br>
• valid_files: an array that holds flags for each file - indicating whether it is valid or not.</br>
• files_name: a two-dimensional array that holds the names of all the files.</br>
• files: an array that holds pointers to the first character of each file.</br>
• files_size: an array that holds the sizes of the files.</br></br>
In the code files on the controller side, we handle interrupts for UART, DMA, PIT, and TPM. Additionally, we have added several functions:</br>
• print_num: This function prints a number on the LCD screen.</br>
• min: This function returns the minimum of two terms.</br>
• receive_file_content: This function enables DMA and programs it with the destination address and the number of bytes to transfer.</br>
• convert_hex: This function converts a hexadecimal number to a decimal number.</br>

### Hardware

In this project we used the KL25Z controller:</br>

![image](https://user-images.githubusercontent.com/62146535/230339693-bba4c98d-0dfd-49f3-8acf-c3fa054052aa.png)

The main modules we used are the UART and DMA modules. In addition, we used a Servo motor and distance sensor.

### UART

UART is an asynchronous serial communication component used for transmitting and receiving information. This component can receive and transmit information simultaneously. Here is how the component works:</br></br>
Reception - the reception module has a shifter and a buffer that is used for receiving information. The information is received bit by bit, and the shifter moves the received information until it is filled. When all the information is filled, the shifter moves it to the buffer, and an interrupt occurs asking the processor to read the information from the buffer. Therefore, in the reception module, there is a transition from serial to parallel information.</br></br>
Transmission - The transmission module also has a shifter and a buffer that is used for transmitting information. The information to be transmitted is received in the buffer and then goes to the shifter. From the shifter, the information is sent bit by bit. In the transmission module, there is a transition from parallel to serial information.

![image](https://user-images.githubusercontent.com/62146535/230340216-9d3122af-ff9f-421c-9796-eada0691f6e3.png)

### DMA

The DMA module enables data transfer between memory and peripheral components without using the CPU. The use of DMA enables faster data transfer and allows the CPU to perform other tasks simultaneously. Furthermore, DMA has several different channels that can be used to transfer data, with each channel linked to another component. In this project, the transfer of files from the computer to the controller is done using a DMA channel with the help of UART.

![image](https://user-images.githubusercontent.com/62146535/230340504-a81cde5f-f0f1-429e-b4de-de2610b0b2e7.png)

### Interrupt Controller

The system on the controller side is based on interrupts generated by the UART, the DMA, and the controller buttons. The interrupts generated by the UART occur upon receiving each character from the computer. The resulting events from the DMA occur when the measurement transfer is completed once the value in the BCR register drops to 0.

### Flowchart of the Design of the FSM-Based System

![image](https://user-images.githubusercontent.com/62146535/230341016-e3643a54-7eb4-4e0e-a2a3-12b60f500a1d.png)


