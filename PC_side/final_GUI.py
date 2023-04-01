import PySimpleGUI as sg
import serial as ser
import os
import time
import math
import pygame
import threading

line_list = []
green_list = []
red_list = []
GREEN = (0, 255, 0)
F_GREEN = (0, 180, 0)
F_RED = (180, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)


def start_telemeter(degree):  # This function starts the telemeter mode
    pygame.init()
    size = (900, 700)
    game_display = pygame.display.set_mode(size)
    pygame.display.set_caption("Telemeter")
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
        game_display.fill((0, 0, 0))
        raw_data = s.readline()
        distance = int(raw_data.decode('utf-8'))

        pygame.font.init()
        draw_telemeter(degree, distance, game_display)
        pygame.display.update()


def start_sonar(limit=400):  # This function starts the Radar Detector System mode
    pygame.init()
    size = (900, 700)
    game_display = pygame.display.set_mode(size)
    pygame.display.set_caption("Sonar")
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        try:
            raw_data = s.readline()
            distance = int(raw_data.decode('utf-8').split('_')[0])
            degree = int(raw_data.decode('utf-8').split('_')[1])
        except:
            s.reset_input_buffer()
            continue
        degree = math.ceil((degree - 98) * (180 / 351))

        draw(degree, distance, game_display, limit)

        pygame.display.update()


def calc_points(radius, angle):  # This function calculates to points according to an angle and a radius
    x = math.cos(math.radians(angle)) * radius
    y = math.sin(math.radians(angle)) * radius

    return x, y


line_list.append(((50, 600), (850, 600)))
for i in range(12):
    j = i + 1
    ang = j * 15
    x, y = calc_points(400, ang)
    line_list.append(((450 - x, 600 - y), (450, 600)))


def draw_telemeter(degree, distance, game_display):  # This function draws the objects for the Telemeter mode
    game_display.fill((0, 0, 0))
    if distance >= 5:
        distance -= 5
    font = pygame.font.SysFont('Calibri', 50)
    text = font.render("Distance: " + str(distance) + "cm", False, RED)
    game_display.blit(text, (300, 100))

    if distance * 6 > 400:
        distance = 66

    for line in line_list:
        pygame.draw.line(game_display, GREEN, line[0], line[1], 1)

    x1, y1 = calc_points(400, 180 - degree)
    x2, y2 = calc_points(distance * 6, 180 - degree)
    pygame.draw.line(game_display, GREEN, (450, 600), (450 - x2, 600 - y2), 4)

    pygame.draw.line(game_display, RED, (450 - x1, 600 - y1), (450 - x2, 600 - y2), 4)

    green_list.append([(450, 600), (450 - x2, 600 - y2), 360])

    red_list.append([(450 - x1, 600 - y1), (450 - x2, 600 - y2), 360])


def draw(degree, distance, game_display, limit):  # This function draws the objects for the Radar Detector System
    global green_list, red_list
    game_display.fill((0, 0, 0))

    for line in line_list:
        pygame.draw.line(game_display, GREEN, line[0], line[1], 1)

    for line in green_list:
        pygame.draw.line(game_display, F_GREEN, line[0], line[1], 4)
        line[2] -= 1

        if line[2] < 0:
            green_list.remove(line)

    for line in red_list:
        pygame.draw.line(game_display, F_RED, line[0], line[1], 4)
        line[2] -= 1

        if line[2] < 0:
            red_list.remove(line)

    if distance > limit or distance * 6 > 400:
        distance = 66

    if degree >= 177 or degree <= 4:
        green_list = []
        red_list = []

    x1, y1 = calc_points(400, 180 - degree)
    x2, y2 = calc_points(distance*6, 180 - degree)
    pygame.draw.line(game_display, GREEN, (450, 600), (450 - x2, 600 - y2), 4)

    pygame.draw.line(game_display, RED, (450 - x1, 600 - y1), (450 - x2, 600 - y2), 4)

    green_list.append([(450, 600), (450 - x2, 600 - y2), 360])

    red_list.append([(450 - x1, 600 - y1), (450 - x2, 600 - y2), 360])


def send_to_controller(message):  # This file sends a message to the controller
    while 1:
        txChar = message + '\n'  # enter key to transmit
        if (txChar == ''): break
        bytesChar = bytes(txChar, 'ascii')
        s.write(bytesChar)
        if s.out_waiting == 0: break


def send_file_to_controller(file):  # This function sends a file to the controller
    with open(file, "rt", encoding="utf-8") as txt_file:
        file_size = os.path.getsize(file)
        file_name = os.path.basename(file)
        if file_size > 8000:
            print("Error!!!!")
            return
        text = txt_file.read()
        send_to_controller(file_name)
        send_to_controller(str(file_size))
        send_file_content_to_controller(file, file_size)
        s.reset_input_buffer()
        while True:
            if s.in_waiting > 0:
                msg = s.readline()
                msg = msg.decode('utf-8')
                break
        sg.Window("Ack", [[sg.Text(msg)]]).read()


def send_file_content_to_controller(file, file_size):  # This function sends a file content to the controller
    text_file = open(file, "rb")
    text = text_file.read()

    start = 0
    end = 1024
    while file_size > 0:
        if file_size <= 1024:
            end = start + file_size
            s.write(text[start:end])
            break
        s.write(text[start:end])
        time.sleep(1)
        start += 1024
        end += 1024
        file_size -= 1024
    text_file.close()


def uart_cfg(baudrate=9600, com='COM3'):  # This function does the configuration for the uart
    s = ser.Serial('COM3', baudrate=baudrate, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)   # timeout of 1 sec so that the read and write operations are blocking,
                                # after the timeout the program continues
    firstReceive = True
    enableTX = False
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()
    return s, firstReceive, enableTX


def execute_thread():  # This function receives a message from the controller in Script mode to activate the sonar
    global green_list
    global red_list
    while True:
        while True:
            if s.in_waiting > 0:
                command = s.readline()
                command = command.decode('utf-8')
                break

        if command.split("_")[0] == "telemeter":
            start_telemeter(int(command.split("_")[1]))
            send_to_controller('s')
            continue
        elif command.split("_")[0] == "scan":
            start_sonar()
            send_to_controller('s')
            s.reset_input_buffer()
            continue
        elif command.split("_")[0] == "EXIT":
            s.reset_input_buffer()
            green_list = []
            red_list = []
            return


main_menu = [
    [sg.Button("Telemeter", key="telemeter_menu_button")],
    [sg.Button("Scan",  key="scan_menu_button")],
    [sg.Button("Script Mode", key="script_menu_button")],
]

telemeter_menu = [
    [sg.Text("Choose a degree:")],
    [sg.InputText(key="telemeter_input", size=(15, 15))],
    [sg.Button("Start", key="telemeter_send_button")],
    [sg.Button("Back", key="telemeter_back_button")],
]

scan_menu = [
    [sg.Text("Write the masking distance:")],
    [sg.InputText(key="scan_input", size=(15, 15))],
    [sg.Button("Start Scan", key="start_scan_button")],
    [sg.Button("Back", key="scan_back_button")],
]

script_menu = [
    [sg.Text("Choose the file you want to send:")],
    [sg.In(), sg.FileBrowse(key="file_path", file_types=(("Text Files", "*.txt"),))],
    [sg.Button("Send", key="file_send_button")],
    [sg.Text("Choose file to execute:")],
    [sg.Listbox(values=[], enable_events=True, size=(45, 5), key="script_Listbox")],
    [sg.Button("Execute", key="file_execute_button")],
    [sg.Button("Back", key="file_back_button")]
]

layout = [[
    sg.Column(main_menu, key="main_menu"),
    sg.Column(telemeter_menu, key="telemeter_menu", visible=False),
    sg.Column(scan_menu, key="scan_menu", visible=False),
    sg.Column(script_menu, key="script_menu", visible=False),
]]


window = sg.Window(title="Menu", layout=layout, margins=(150, 150))
s, firstReceive, enableTX = uart_cfg()
file_list = []


def main():
    global green_list
    global red_list
    while True:
        event, values = window.read()
        if event == sg.WIN_CLOSED:
            break

        if event == "telemeter_menu_button":
            window["main_menu"].update(visible=False)
            window["telemeter_menu"].update(visible=True)

        if event == "scan_menu_button":
            window["main_menu"].update(visible=False)
            window["scan_menu"].update(visible=True)

        if event == "telemeter_back_button":
            window["main_menu"].update(visible=True)
            window["telemeter_menu"].update(visible=False)
            send_to_controller('0')

        if event == "scan_back_button":
            window["main_menu"].update(visible=True)
            window["scan_menu"].update(visible=False)
            send_to_controller('0')

        if event == "telemeter_send_button":
            send_to_controller('0')
            send_to_controller('2')
            deg = values["telemeter_input"]
            deg = math.ceil(int(deg) * (351 / 180)) + 98
            send_to_controller(str(deg))
            start_telemeter(int(values["telemeter_input"]))
            window["telemeter_input"].update("")

        if event == "start_scan_button":
            send_to_controller('1')
            start_sonar(int(values["scan_input"]))
            window["scan_input"].update("")
            send_to_controller('0')
            green_list = []
            red_list = []

        if event == "script_menu_button":
            window["main_menu"].update(visible=False)
            window["script_menu"].update(visible=True)

        if event == "file_back_button":
            window["main_menu"].update(visible=True)
            window["script_menu"].update(visible=False)
            send_to_controller('0')

        if event == "file_send_button":
            send_to_controller('3')
            send_file_to_controller(values["file_path"])
            file_name = values["file_path"].split('/')[-1]
            file_list.append(file_name)
            if len(file_list) > 3:
                file_list.pop(0)
            window["script_Listbox"].update(file_list)
            # threading.Thread(target=ack_message).start()

        if event == "file_execute_button":
            send_to_controller('4')
            send_to_controller(values["script_Listbox"][0])
            threading.Thread(target=execute_thread).start()

    window.close()


if __name__ == '__main__':
    main()
