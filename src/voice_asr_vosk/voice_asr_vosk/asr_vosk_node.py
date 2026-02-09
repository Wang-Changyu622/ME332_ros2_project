#!/usr/bin/env python3
import json
import queue
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import sounddevice as sd
from vosk import Model, KaldiRecognizer


class VoskASRNode(Node):
    def __init__(self):
        super().__init__('vosk_asr_node')

        #配置模型路径
        model_path = "/home/daniel/models/vosk/vosk-model-small-cn-0.22"
        self.get_logger().info(f"加载 Vosk 模型: {model_path}")
        self.model = Model(model_path)
        self.sample_rate = 16000
        self.rec = KaldiRecognizer(self.model, self.sample_rate)
        self.pub = self.create_publisher(String, '/voice_cmd_text', 10)
        self.audio_q = queue.Queue()

        # 录音设备参数
        self.channels = 1
        self.block_duration = 0.5

        self._start_audio_stream()
        self._start_recognition_thread()

        self.get_logger().info("VoskASRNode 已启动，开始监听麦克风...")

    #音频采集部分
    def _audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"音频状态警告: {status}")
        self.audio_q.put(bytes(indata))

    def _start_audio_stream(self):
        self.stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=int(self.sample_rate * self.block_duration),
            dtype='int16',
            channels=self.channels,
            callback=self._audio_callback
        )
        self.stream.start()

    #识别线程
    def _start_recognition_thread(self):
        t = threading.Thread(target=self._recognize_loop, daemon=True)
        t.start()

    def _recognize_loop(self):
        self.get_logger().info("识别线程已启动")
        buffer_text = ""

        while rclpy.ok():
            data = self.audio_q.get()
            if self.rec.AcceptWaveform(data):
                result_json = self.rec.Result()
                result = json.loads(result_json)
                text = (result.get("text") or "").strip()
                if text:
                    self._handle_sentence(text)
            else:
                partial_json = self.rec.PartialResult()
                partial = json.loads(partial_json).get("partial", "").strip()

    def _handle_sentence(self, text: str):
        self.get_logger().info(f"识别到句子: '{text}'")

        if len(text) < 2:
            return

        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"已发布到 /voice_cmd_text: '{text}'")


def main(args=None):
    rclpy.init(args=args)
    node = VoskASRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stream.stop()
    node.stream.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
