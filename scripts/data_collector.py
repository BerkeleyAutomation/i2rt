import cv2
from i2rt.robots.get_robot import get_yam_robot
from minimum_gello import DEFAULT_ROBOT_PORT, ClientRobot
from PIL import Image, ImageTk
from robodm import Trajectory
import subprocess
import threading
import time
from tkinter import Tk, Frame, Label, Button, filedialog
import subprocess
class DataCollector:
    def __init__(self, root, config):
        self.root = root
        self.config = config
        subprocess.run(["/home/yam/i2rt/scripts/reset_all_can.sh"])

        self.camera = None
        self.system_enabled = False
        self.recording = False
        self.video_thread = None

        self.follower_process = None
        self.leader_process = None
        self.leader_client = None

        self.trajectory = None

        self.replay_trajectory = None
        self.replay_robot = None

        main_frame = Frame(self.root)
        main_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.bind('q', lambda event: self.disable_system())

        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)

        font_style = ('Arial', 60)

        self.camera_viewer = Label(main_frame, text='System Disabled', font=font_style, background='black', foreground='white')
        self.camera_viewer.grid(row=0, column=0, sticky='nsew')
        self.camera_viewer.columnconfigure(0, weight=1)
        self.camera_viewer.rowconfigure(0, weight=1)

        control_frame = Frame(main_frame)
        control_frame.grid(row=1, column=0, sticky='ew', pady=10)

        self.enable_btn = Button(control_frame, text='System Enable', font=font_style, command=self.enable_system)
        self.enable_btn.grid(row=0, column=0, padx=10)

        self.start_btn = Button(control_frame, text='Demo Start', font=font_style, command=self.start_demo, state='disabled')
        self.start_btn.grid(row=0, column=1, padx=10)

        self.stop_btn = Button(control_frame, text='Demo Stop', font=font_style, command=self.stop_demo, state='disabled')
        self.stop_btn.grid(row=0, column=2, padx=10)

        self.disable_btn = Button(control_frame, text='System Disable', font=font_style, command=self.disable_system, state='disabled')
        self.disable_btn.grid(row=0, column=3, padx=10)

        self.replay_btn = Button(control_frame, text='Replay Demo', font=font_style, command=self.replay_demo)
        self.replay_btn.grid(row=0, column=4, padx=10)

    def enable_system(self):
        self.camera = cv2.VideoCapture(self.config['camera']['index'])

        self.follower_process = subprocess.Popen([
            'python', '/home/yam/i2rt/scripts/minimum_gello.py',
            '--gripper', self.config['follower']['gripper'],
            '--mode', 'follower',
            '--can-channel', self.config['follower']['channel'],
            '--bilateral_kp', str(self.config['follower']['bilateral_kp']),
        ], stdout=subprocess.DEVNULL)
        self.leader_process = subprocess.Popen([
            'python', '/home/yam/i2rt/scripts/minimum_gello.py',
            '--gripper', self.config['leader']['gripper'],
            '--mode', 'leader',
            '--can-channel', self.config['leader']['channel'],
            '--bilateral_kp', str(self.config['leader']['bilateral_kp']),
        ], stdout=subprocess.DEVNULL)
        # TODO: read data from follower instead
        self.leader_client = ClientRobot(port=self.config['leader']['port'], host=self.config['leader']['host'])

        self.system_enabled = True
        self.video_thread = threading.Thread(target=self.update)
        self.video_thread.daemon = True
        self.video_thread.start()

        self.enable_btn.config(state='disabled')
        self.start_btn.config(state='normal')
        self.disable_btn.config(state='normal')
        self.replay_btn.config(state='disabled')

    def start_demo(self):
        if not self.system_enabled:
            return

        if self.trajectory is None:
            self.trajectory = Trajectory(path=f'{self.config["robodm"]["data_dir"]}/{time.strftime("%Y%m%d_%H%M%S")}.vla', mode='w', video_codec=self.config['robodm']['video_codec'])

        self.recording = True
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')

    def update(self):
        while self.system_enabled:
            start_time = time.time()
            if self.camera is not None:
                frame = cv2.cvtColor(self.camera.read()[1], cv2.COLOR_RGB2BGR)

                # TODO: this ui stuff takes nearly all of the time and slows down the pipeline quite a bit
                label_width = self.camera_viewer.winfo_width()
                label_height = self.camera_viewer.winfo_height()

                frame_height, frame_width, _ = frame.shape
                scale = min(label_width / frame_width, label_height / frame_height)
                frame = cv2.resize(frame, (int(frame_width * scale), int(frame_height * scale)))
                photo = ImageTk.PhotoImage(Image.fromarray(frame))

                self.root.after(0, lambda: self.camera_viewer.config(image=photo, text=''))
                self.root.after(0, lambda: setattr(self.camera_viewer, 'image', photo))
                mid_time = time.time()

                if self.trajectory is not None:
                    self.trajectory.add('camera/rgb', frame)

            if self.leader_client is not None:
                joint_pos = self.leader_client.get_joint_pos()

                if self.trajectory is not None:
                    self.trajectory.add('robot/joint_pos', joint_pos)

            end_time = time.time()
            # print(f'{int(1000 * (mid_time - start_time))}, {int(1000 * (end_time - mid_time))}')

            # TODO: bring this back to control the update frequency
            # time.sleep(1 / self.config['update_frequency'])

        self.root.after(0, lambda: self.camera_viewer.config(image='', text='System Disabled'))

    def stop_demo(self):
        self.recording = False

        if self.trajectory is not None:
            self.trajectory.close()
            self.trajectory = None

        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')

    def disable_system(self):
        self.system_enabled = False

        if self.recording is not None:
            self.stop_demo()

        if self.video_thread is not None:
            self.video_thread.join(timeout=0.5)

        if self.camera is not None:
            self.camera.release()
            self.root.after(0, lambda: self.camera_viewer.config(image='', text='System Disabled'))
            self.camera = None

        if self.follower_process is not None:
            self.follower_process.terminate()
            self.follower_process = None

        if self.leader_process is not None:
            self.leader_process.terminate()
            self.leader_process = None

        if self.leader_client is not None:
            self.leader_client.close()
            self.leader_client = None

        if self.replay_trajectory is not None:
            self.replay_trajectory.close()
            self.replay_trajectory = None

        if self.replay_robot is not None:
            self.replay_robot.close()
            self.replay_robot = None

        self.enable_btn.config(state='normal')
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='disabled')
        self.disable_btn.config(state='disabled')
        self.replay_btn.config(state='normal')

    def replay_demo(self):
        if self.trajectory is not None:
            return

        self.enable_btn.config(state='disabled')

        file_path = filedialog.askopenfilename(
            title="Select a .vla file to replay",
            filetypes=[("VLA files", "*.vla")]
        )

        if not file_path:
            self.enable_btn.config(state='normal')
            return

        self.replay_trajectory = Trajectory(path=file_path, mode='r')
        data = self.replay_trajectory.load()
        self.replay_trajectory.close()
        self.replay_trajectory = None

        self.replay_robot = get_yam_robot(channel=self.config['follower']['channel'])

        for i in range(len(data['camera/rgb'])):
            frame = data['camera/rgb'][i]

            label_width = self.camera_viewer.winfo_width()
            label_height = self.camera_viewer.winfo_height()

            frame_height, frame_width, _ = frame.shape
            scale = min(label_width / frame_width, label_height / frame_height)
            frame = cv2.resize(frame, (int(frame_width * scale), int(frame_height * scale)))
            photo = ImageTk.PhotoImage(Image.fromarray(frame))

            # TODO: confirm the ui stuff actually works
            self.root.after(0, lambda: self.camera_viewer.config(image=photo, text=''))
            self.root.after(0, lambda: setattr(self.camera_viewer, 'image', photo))

            self.replay_robot.command_joint_pos(data['robot/joint_pos'][i])

            # TODO: reduce this to avoid jerky output motion
            time.sleep(0.1)

        self.replay_robot.close()
        self.replay_robot = None
        self.root.after(0, lambda: self.camera_viewer.config(image='', text='System Disabled'))

        self.enable_btn.config(state='normal')

    def on_destroy(self):
        if self.system_enabled:
            self.disable_system()
            cv2.destroyAllWindows()
        self.root.destroy()

if __name__ == '__main__':
    config = {
        'camera': {
            'index': 0,
        },
        'follower': {
            'gripper': 'yam_compact_small',
            'channel': 'can0',
            'bilateral_kp': 0.2,
        },
        'leader': {
            'gripper': 'yam_compact_small',
            'channel': 'can1',
            'bilateral_kp': 0.2,
            'host': '127.0.0.1',
            'port': DEFAULT_ROBOT_PORT,
        },
        'robodm': {
            'data_dir': '/home/yam/i2rt/robot_demos',
            'video_codec': 'rawvideo',
        },
        'update_frequency': 30,
    }

    root = Tk()
    root.title('Data Collector')
    root.geometry(f'{root.winfo_screenwidth()}x{root.winfo_screenheight()}')
    app = DataCollector(root, config)
    root.protocol('WM_DELETE_WINDOW', app.on_destroy)
    root.mainloop()
