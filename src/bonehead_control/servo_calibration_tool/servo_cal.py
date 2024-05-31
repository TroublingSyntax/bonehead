from tkinter import *
from tkinter import ttk

import serial
from serial import *
from serial.tools import list_ports

class data():
    def __init__(self):
        self.serial_selection = None
        self.serial_selection_index = None
        self.current_port = None
        self.lbox2_selection = None
        self.is_connected = False
        self.status = StringVar(value='Not Connected')
        self.status_label = StringVar(value='Connect')
        self.num_ports = 0
        self.serial_ports = None
        self.serial_info = None
        self.len_max = 20

        self.fr1 = StringVar(value='292')
        self.fr2 = StringVar(value='292')
        self.fr3 = StringVar(value='292')

        self.fl1 = StringVar(value='292')
        self.fl2 = StringVar(value='292')
        self.fl3 = StringVar(value='292')

        self.br1 = StringVar(value='292')
        self.br2 = StringVar(value='292')
        self.br3 = StringVar(value='292')

        self.bl1 = StringVar(value='292')
        self.bl2 = StringVar(value='292')
        self.bl3 = StringVar(value='292')

        self.header = 'DEFAULT'
        self.pwm = 292
        self.serial_msg = ''

root = Tk()
root.title('Bonehead Servo Calibration')
now_data = data()

mainframe = ttk.Frame(root)
mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

servo_title = ttk.Label(mainframe, text='Select a servo to calibrate')
servo_title.grid(column=0, row=2)

""" SERVO DROPDOWN STUFF"""

servo_choices = ['Servo Selection', 'Front Right Theta 1', 'Front Right Theta 2', 'Front Right Theta 3',
                 'Front Left Theta 1', 'Front Left Theta 2', 'Front Left Theta 3',
                 'Back Right Theta 1', 'Back Right Theta 2', 'Back Right Theta 3',
                 'Back Left Theta 1', 'Back Left Theta 2', 'Back Left Theta 3',]

servo_list = StringVar(value=servo_choices)

combo = ttk.Combobox(mainframe,
                     state='readonly',
                     values=servo_choices,
                     justify='right',
                     height=5,
                     width=15,
                    )
combo.current(0)

combo.grid(column=0, row=3, sticky=(E,W))

""" SERIAL PORT DROPDOWN STUFF"""

serial_combo = ttk.Combobox(mainframe,
                            state='readonly',
                            values=now_data.serial_info,
                            justify='right',
                            height=10,
                            width=now_data.len_max)
serial_combo.grid(column=4, row=0, padx=10)

def update_serial():

    now_data.serial_ports = list_ports.comports(include_links=True)

    now_data.num_ports = len(now_data.serial_ports)

    now_data.serial_info = [None] * now_data.num_ports

    for port in range(len(now_data.serial_info)):
        now_data.serial_info[port] = now_data.serial_ports[port].device + "   <--- (" + now_data.serial_ports[port].manufacturer + ")"
    if now_data.num_ports == 0:
        serial_combo.set('')
    else:   
        serial_combo.set(now_data.serial_info[0])
    if now_data.serial_info == None:
        now_data.serial_info = ['']
        serial_combo.current(0)
    now_data.len_max = 20
    for m in now_data.serial_info:
        if len(m) > now_data.len_max:
            now_data.len_max = len(m)
    serial_combo.configure(values=now_data.serial_info)
    serial_combo.configure(width=now_data.len_max)
    serial_combo.update()
    return

update_serial()

refresh_button = ttk.Button(mainframe,
                            text='Refresh',
                            command=update_serial
                            )
refresh_button.grid(column=2, row=0, padx=20)

ttk.Label(mainframe, text="Serial Device: ", justify='right').grid(column=3, row=0, sticky=(W, E))

status_label = ttk.Label(mainframe, textvariable=now_data.status).grid(column=5,row=1,sticky=N)

"""CONNECT BUTTON STUFF"""

def connect_serial():
    if not now_data.is_connected:
        try:
            index = serial_combo.current()
            new_device = now_data.serial_ports[index].device
            now_data.current_port = serial.Serial(new_device, 19200, timeout=1)
        except IndexError:
            return
        written = now_data.current_port.write('v'.encode())
        if written > 0:
            now_data.is_connected = True
            now_data.status.set('Connected')
            now_data.status_label.set('Disconnect')
            now_data.header = 'CONNECTED'
            return
        else:
            now_data.is_connected = False
            now_data.status.set('Not Connected')
            now_data.status_label.set('Connect')
            return
    if now_data.is_connected:
        now_data.current_port.close()
        now_data.status.set('Not Connected')
        now_data.status_label.set('Connect')
        now_data.is_connected = False
        now_data.header = 'DISCONNECTED'
    return

connect_button = ttk.Button(mainframe, textvariable=now_data.status_label, command=connect_serial).grid(column=5, row=0, padx=10)

"""PWM SLIDER"""
slider = Scale(mainframe, from_=100, to=500, orient='horizontal')
slider.grid(column=2, columnspan=3, row=3, padx=20, sticky=(W,E))

def send_serial():
    if now_data.header == 'DISCONNECTED':
        return
    else:
        match combo.current():
            case 1:  # fr1
                now_data.header = 'a'
            case 2:  # fr2
                now_data.header = 'b'
            case 3:  # fr3
                now_data.header = 'c'
            case 4:  # fl1
                now_data.header = 'd'
            case 5:  # fl2
                now_data.header = 'e'
            case 6:  # fl3
                now_data.header = 'f'
            case 7:  # br1
                now_data.header = 'g'
            case 8:  # br2
                now_data.header = 'h'
            case 9:  # br3
                now_data.header = 'i'
            case 10: # bl1
                now_data.header = 'j'
            case 11: # bl2
                now_data.header = 'k'
            case 12: # bl3
                now_data.header = 'l'
            case _:
                return
        now_data.pwm = str(slider.get())
        now_data.serial_msg = now_data.header + now_data.pwm + 'z'
        now_data.current_port.write(now_data.serial_msg.encode())
        print(now_data.serial_msg)
    return

""" SEND BUTTON"""
send_button = ttk.Button(mainframe, text='Send', command=send_serial)
send_button.grid(column=5, row=3, padx=10, sticky=(N,S,E,W))

"""SAVE BUTTON"""
def save_servo_cal():
    match combo.current():
        case 1:  # fr1
            now_data.fr1.set(str(slider.get()))
        case 2:  # fr2
            now_data.fr2.set(str(slider.get()))
        case 3:  # fr3
            now_data.fr3.set(str(slider.get()))
        case 4:  # fl1
            now_data.fl1.set(str(slider.get()))
        case 5:  # fl2
            now_data.fl2.set(str(slider.get()))
        case 6:  # fl3
            now_data.fl3.set(str(slider.get()))
        case 7:  # br1
            now_data.br1.set(str(slider.get()))
        case 8:  # br2
            now_data.br2.set(str(slider.get()))
        case 9:  # br3
            now_data.br3.set(str(slider.get()))
        case 10: # bl1
            now_data.bl1.set(str(slider.get()))
        case 11: # bl2
            now_data.bl2.set(str(slider.get()))
        case 12: # bl3
            now_data.bl3.set(str(slider.get()))
        case _:
            return
    return

save_button = ttk.Button(mainframe, text='Save', command=save_servo_cal)
save_button.grid(column=5, row=4, padx=10, pady=20, sticky=(N,S,E,W))

""" SERVO CAL LABELS"""
theta1_label = ttk.Label(mainframe, text='Theta 1:').grid(column=0, row=6)
theta2_label = ttk.Label(mainframe, text='Theta 2:').grid(column=0, row=7)
theta3_label = ttk.Label(mainframe, text='Theta 3:').grid(column=0, row=8)

fr_label = ttk.Label(mainframe, text='Front Right').grid(column=1, row=5)
fl_label = ttk.Label(mainframe, text='Front Left').grid(column=2, row=5)
br_label = ttk.Label(mainframe, text='Back Right').grid(column=3, row=5)
bl_label = ttk.Label(mainframe, text='Back Left').grid(column=4, row=5)

"""FRONT RIGHT"""

#fr1_label = ttk.Label(mainframe, text='fr1: ').grid(column=0, row=5, pady=15)
#fr2_label = ttk.Label(mainframe, text='fr2: ').grid(column=0, row=6, pady=15)
#fr3_label = ttk.Label(mainframe, text='fr3: ').grid(column=0, row=7, pady=15)

fr1 = ttk.Label(mainframe, textvariable=now_data.fr1,).grid(column=1, row=6, pady=15)
fr2 = ttk.Label(mainframe, textvariable=now_data.fr2,).grid(column=1, row=7, pady=15)
fr3 = ttk.Label(mainframe, textvariable=now_data.fr3,).grid(column=1, row=8, pady=15)

"""FRONT LEFT"""

#fl1_label = tnow_data.header = 'a'tk.Label(mainframe, text='fl1: ').grid(column=2, row=5, pady=15)
#fl2_label = ttk.Label(mainframe, text='fl2: ').grid(column=2, row=6, pady=15)
#fl3_label = ttk.Label(mainframe, text='fl3: ').grid(column=2, row=7, pady=15)

fl1 = ttk.Label(mainframe, textvariable=now_data.fl1).grid(column=2, row=6, pady=15)
fl2 = ttk.Label(mainframe, textvariable=now_data.fl2).grid(column=2, row=7, pady=15)
fl3 = ttk.Label(mainframe, textvariable=now_data.fl3).grid(column=2, row=8, pady=15)

"""BACK RIGHT"""

#br1_label = ttk.Label(mainframe, text='br1: ').grid(column=4, row=5, pady=15)
#br2_label = ttk.Label(mainframe, text='br2: ').grid(column=4, row=6, pady=15)
#br3_label = ttk.Label(mainframe, text='br3: ').grid(column=4, row=7, pady=15)

br1 = ttk.Label(mainframe, textvariable=now_data.br1).grid(column=3, row=6, pady=15)
br2 = ttk.Label(mainframe, textvariable=now_data.br2).grid(column=3, row=7, pady=15)
br3 = ttk.Label(mainframe, textvariable=now_data.br3).grid(column=3, row=8, pady=15)

"""BACK LEFT"""

#bl1_label = ttk.Label(mainframe, text='bl1: ').grid(column=6, row=5, pady=15)
#bl2_label = ttk.Label(mainframe, text='bl2: ').grid(column=6, row=6, pady=15)
#bl3_label = ttk.Label(mainframe, text='bl3: ').grid(column=6, row=7, pady=15)

bl1 = ttk.Label(mainframe, textvariable=now_data.bl1).grid(column=4, row=6, pady=15)
bl2 = ttk.Label(mainframe, textvariable=now_data.bl2).grid(column=4, row=7, pady=15)
bl3 = ttk.Label(mainframe, textvariable=now_data.bl3).grid(column=4, row=8, pady=15)

"""MAIN LOOP"""

root.mainloop()