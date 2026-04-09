#!/usr/bin/env python3
"""
audio_feedback_node.py — Three feedback modes for BVI navigation study.

Modes (set via ROS2 param at launch):
  tones   — beeps only, no speech  (Mode A)
  speech  — Piper TTS only         (Mode B)
  hybrid  — tone first + speech    (Mode C)

Launch:
  ros2 run master audio_feedback_node --ros-args -p mode:=tones
  ros2 run master audio_feedback_node --ros-args -p mode:=speech
  ros2 run master audio_feedback_node --ros-args -p mode:=hybrid

Piper voice model expected at:
  ~/piper_voices/en_GB-alan-medium.onnx
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import queue
import time
import re
import os

# ── Severity helpers ─────────────────────────────────────────
def get_severity(text):
    t = text.upper()
    if any(w in t for w in ['DANGER', 'STOP', 'RAMP DOWN', 'STEP']):
        return 'DANGER'
    elif any(w in t for w in ['CAUTION', 'WARN', 'SLOW', 'RAMP UP', 'OBSTACLE', 'TURNING']):
        return 'WARN'
    return 'CLEAR'


def guidance_to_speech(text):
    """Convert guidance string to natural spoken sentence."""
    t = text

    # Remove special chars
    t = t.replace('—', ', ')
    t = t.replace('–', ', ')
    t = t.replace('➤', '')
    t = re.sub(r'[()]', '', t)

    # Convert distances: 0.6m → 0.6 metres
    t = re.sub(r'(\d+\.?\d*)m\b', lambda m: m.group(1) + ' metres', t)

    # Soften capitalised words
    t = t.replace('DANGER', 'Danger')
    t = t.replace('STOP', 'Stop')
    t = t.replace('WARN', 'Warning')

    # Clean up extra spaces
    t = re.sub(r'\s+', ' ', t).strip()
    return t


# ── Tone player ───────────────────────────────────────────────
def play_tone(severity, block=True):
    """Play a beep. block=False returns immediately."""
    if severity == 'DANGER':
        cmds = [
            ['speaker-test', '-t', 'sine', '-f', '880',
             '-l', '1', '-p', '250'],
        ] * 3
    elif severity == 'WARN':
        cmds = [['speaker-test', '-t', 'sine', '-f', '520',
                 '-l', '1', '-p', '400']]
    else:
        return

    def _play():
        for cmd in cmds:
            subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            ).wait()
            if severity == 'DANGER':
                time.sleep(0.05)

    if block:
        _play()
    else:
        threading.Thread(target=_play, daemon=True).start()


# ── Piper TTS ─────────────────────────────────────────────────
PIPER_MODEL = os.path.expanduser(
    '~/piper_voices/en_GB-alan-medium.onnx')
PIPER_AVAILABLE = os.path.exists(PIPER_MODEL)


def speak(text, block=True):
    """Speak text using Piper TTS → aplay pipeline."""
    if not PIPER_AVAILABLE:
        # Graceful fallback: try espeak
        cmd = ['espeak', '-s', '145', '-v', 'en', text]
        if block:
            subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL).wait()
        else:
            threading.Thread(
                target=lambda: subprocess.Popen(
                    cmd, stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL).wait(),
                daemon=True).start()
        return

    def _speak():
        piper = subprocess.Popen(
            ['piper', '--model', PIPER_MODEL, '--output_raw'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        aplay = subprocess.Popen(
            ['aplay', '-r', '22050', '-f', 'S16_LE', '-t', 'raw', '-'],
            stdin=piper.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        piper.stdin.write(text.encode())
        piper.stdin.close()
        piper.stdout.close()
        aplay.wait()

    if block:
        _speak()
    else:
        threading.Thread(target=_speak, daemon=True).start()


# ── Main node ─────────────────────────────────────────────────
class AudioFeedbackNode(Node):
    def __init__(self):
        super().__init__('audio_feedback_node')

        self.declare_parameter('mode', 'hybrid')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        if self.mode not in ('tones', 'speech', 'hybrid'):
            self.get_logger().warn(
                f"Unknown mode '{self.mode}', defaulting to 'hybrid'")
            self.mode = 'hybrid'

        self.get_logger().info(
            f"Audio feedback node started — mode: {self.mode.upper()}")

        if not PIPER_AVAILABLE and self.mode in ('speech', 'hybrid'):
            self.get_logger().warn(
                f"Piper model not found at {PIPER_MODEL}. "
                "Falling back to espeak. Install Piper for better quality.")

        # Queue so audio never overlaps: drop if busy
        self.audio_queue  = queue.Queue(maxsize=1)
        self.prev_guidance = ''
        self.audio_busy    = False

        self.sub = self.create_subscription(
            String, '/navigation/guidance',
            self.guidance_callback, 10)

        # Worker thread processes the audio queue
        threading.Thread(target=self._audio_worker, daemon=True).start()

    def guidance_callback(self, msg):
        text = msg.data.strip()
        if not text or text == self.prev_guidance:
            return
        self.prev_guidance = text

        severity = get_severity(text)

        # Don't queue if worker is busy AND new msg is non-urgent
        if self.audio_busy and severity != 'DANGER':
            return

        # Replace queue item if urgent
        try:
            self.audio_queue.put_nowait((text, severity))
        except queue.Full:
            if severity == 'DANGER':
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    pass
                self.audio_queue.put_nowait((text, severity))

    def _audio_worker(self):
        while True:
            text, severity = self.audio_queue.get()
            self.audio_busy = True
            try:
                self._play(text, severity)
            except Exception as e:
                self.get_logger().error(f"Audio error: {e}")
            finally:
                self.audio_busy = False

    def _play(self, text, severity):
        spoken = guidance_to_speech(text)
        self.get_logger().info(
            f"[{self.mode.upper()}] {severity} → '{spoken}'")

        if self.mode == 'tones':
            # ── Mode A: beeps only ────────────────────────
            play_tone(severity, block=True)

        elif self.mode == 'speech':
            # ── Mode B: speech only ───────────────────────
            speak(spoken, block=True)

        elif self.mode == 'hybrid':
            # ── Mode C: short tone then speech ───────────
            # For DANGER: urgent tone fires immediately,
            # speech follows right after
            if severity == 'DANGER':
                # Short sharp beep (non-blocking) then speak
                subprocess.Popen(
                    ['speaker-test', '-t', 'sine', '-f', '880',
                     '-l', '1', '-p', '150'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL).wait()
                speak(spoken, block=True)
            elif severity == 'WARN':
                # Single warn tone then speech
                subprocess.Popen(
                    ['speaker-test', '-t', 'sine', '-f', '520',
                     '-l', '1', '-p', '300'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL).wait()
                speak(spoken, block=True)
            else:
                # CLEAR: speech only, no tone
                speak(spoken, block=True)


def main():
    rclpy.init()
    rclpy.spin(AudioFeedbackNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
