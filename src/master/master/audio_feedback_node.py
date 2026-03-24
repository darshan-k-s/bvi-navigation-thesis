#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import time

# ── Severity classification ───────────────────────────────────
def get_severity(text):
    t = text.upper()
    if any(w in t for w in ['DANGER', 'STOP', 'RAMP DOWN', 'STEP DOWN']):
        return 'DANGER'
    elif any(w in t for w in ['CAUTION', 'WARN', 'SLOW', 'RAMP UP', 'OBSTACLE']):
        return 'WARN'
    return 'CLEAR'

def play_tone(severity):
    if severity == 'DANGER':
        for _ in range(3):
            subprocess.Popen(
                ['speaker-test', '-t', 'sine', '-f', '880', '-l', '1', '-p', '300'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            ).wait()
            time.sleep(0.1)
    elif severity == 'WARN':
        subprocess.Popen(
            ['speaker-test', '-t', 'sine', '-f', '520', '-l', '1', '-p', '500'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        ).wait()

def speak(text, speed=145, pitch=55):
    subprocess.Popen(
        ['espeak', '-s', str(speed), '-p', str(pitch), text],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

def kill_speech():
    subprocess.call(['pkill', '-f', 'espeak'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class AudioFeedbackNode(Node):
    def __init__(self):
        super().__init__('audio_feedback_node')

        self.sub = self.create_subscription(
            String, '/navigation/guidance', self.guidance_callback, 10)

        self.last_spoken     = ''
        self.last_speak_time = 0.0
        self.lock            = threading.Lock()

        self.cooldown = {
            'DANGER': 1.5,
            'WARN':   3.0,
            'CLEAR':  6.0,
        }
        self.get_logger().info("Audio feedback node started")

    def guidance_callback(self, msg):
        text     = msg.data.strip()
        severity = get_severity(text)
        now      = time.time()

        with self.lock:
            same_msg   = (text == self.last_spoken)
            time_since = now - self.last_speak_time

            if same_msg and time_since < self.cooldown[severity]:
                return

            if severity == 'DANGER':
                t = threading.Thread(target=self._danger_output, args=(text,), daemon=True)
            elif severity == 'WARN':
                t = threading.Thread(target=self._warn_output,  args=(text,), daemon=True)
            else:
                t = threading.Thread(target=self._clear_output, args=(text,), daemon=True)

            t.start()
            self.last_spoken     = text
            self.last_speak_time = now

    def _danger_output(self, text):
        kill_speech()
        time.sleep(0.05)
        play_tone('DANGER')
        speak(text, speed=160, pitch=60)
        print(f'\033[91m\033[1m  🔊  [DANGER] {text}\033[0m')

    def _warn_output(self, text):
        play_tone('WARN')
        time.sleep(0.55)
        speak(text, speed=150, pitch=55)
        print(f'\033[93m\033[1m  🔊  [WARN]   {text}\033[0m')

    def _clear_output(self, text):
        speak(text, speed=140, pitch=50)
        print(f'\033[92m\033[1m  🔊  [CLEAR]  {text}\033[0m')


def main():
    rclpy.init()
    node = AudioFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    kill_speech()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
