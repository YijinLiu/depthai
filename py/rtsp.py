from queue import Queue
import signal
import threading

from absl import app
from absl import flags
import cv2
import depthai
import gi
import numpy as np

gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject, GLib

FLAGS = flags.FLAGS

flags.DEFINE_enum('camera', 'rgb', ['left', 'right', 'rgb', 'depth'], 'Camera')
flags.DEFINE_enum('resolution', '4k', ['1080p', '4k', '12mp'], 'Sensor resolution')
flags.DEFINE_enum('depth_resolution', '720p', ['720p', '800p', '400p'], 'Sensor resolution')
flags.DEFINE_float('fps', 10, 'FPS')
flags.DEFINE_enum('encoder', 'h265_main',
                  ['h264_baseline', 'h264_high', 'h264_main', 'h265_main', 'mjpeg'], 'Encoder')
flags.DEFINE_integer('bitrate', 0, '');
flags.DEFINE_string('rtsp_path', '', '');
flags.DEFINE_bool('use_udp', False, '');
flags.DEFINE_integer('rtsp_port', 5554, 'RTSP port')
flags.DEFINE_integer('gst_debug', 3, 'GST_DEBUG')

class MediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, encoder: str, width:int, height: int, queue_size: int):
        GstRtspServer.RTSPMediaFactory.__init__(self)

        parser = ''
        rtp = None
        if encoder == 'h265_main':
            parser = '! h265parse'
            rtp = 'rtph265pay'
        elif (encoder == 'h264_baseline' or encoder == 'h264_high' or
              encoder == 'h264_main'):
            parser = '! h264parse'
            rtp = 'rtph264pay'
        elif encoder == 'mjpeg':
            rtp = 'rtpjpegpay'
        self._gst_pipeline = (
            f'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME '
            f'{parser} ! {rtp} name=pay0 config-interval=1 pt=96')
        print(f'Will use "{self._gst_pipeline}".')

        self._pkt_queue = Queue(maxsize = queue_size)

    def send_pkt(self, pkt: depthai.ImgFrame):
        while self._pkt_queue.full():
            self._pkt_queue.get()
        self._pkt_queue.put(pkt)

    def _on_need_data(self, src, length):
        pkt = self._pkt_queue.get()
        data = pkt.getData()
        #print(f'Sending packet {pkt.getSequenceNum()}/{len(data)} ...')
        buf = Gst.Buffer.new_wrapped(data.tobytes())
        retval = src.emit('push-buffer', buf)
        if retval != Gst.FlowReturn.OK:
            print(retval)

    def do_create_element(self, url):
        return Gst.parse_launch(self._gst_pipeline)

    def do_configure(self, rtsp_media):
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name('source')
        appsrc.connect('need-data', self._on_need_data)

def main(argv):
    pipeline = depthai.Pipeline()

    videoEnc = pipeline.createVideoEncoder()
    profile = depthai.VideoEncoderProperties.Profile.H265_MAIN
    if FLAGS.encoder == 'h264_baseline':
        profile = depthai.VideoEncoderProperties.Profile.H264_BASELINE
    elif FLAGS.encoder == 'h264_high':
        profile = depthai.VideoEncoderProperties.Profile.H264_HIGH
    elif FLAGS.encoder == 'h264_main':
        profile = depthai.VideoEncoderProperties.Profile.H264_MAIN
    elif FLAGS.encoder == 'h265_main':
        profile = depthai.VideoEncoderProperties.Profile.H265_MAIN
    elif FLAGS.encoder == 'mjpeg':
        profile = depthai.VideoEncoderProperties.Profile.MJPEG

    width = 0
    height = 0
    if FLAGS.camera == 'depth':
        left = pipeline.createMonoCamera()
        left.setBoardSocket(depthai.CameraBoardSocket.LEFT)
        right = pipeline.createMonoCamera()
        right.setBoardSocket(depthai.CameraBoardSocket.RIGHT)
        if FLAGS.depth_resolution == '720p':
            left.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
            right.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_720_P)
            videoEnc.setDefaultProfilePreset(1280, 720, FLAGS.fps, profile)
            width = 1280
            height = 720
        elif FLAGS.depth_resolution == '800p':
            left.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_800_P)
            right.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_800_P)
            videoEnc.setDefaultProfilePreset(1280, 800, FLAGS.fps, profile)
            width = 1280
            height = 800
        elif FLAGS.depth_resolution == '400p':
            left.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
            right.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
            videoEnc.setDefaultProfilePreset(640, 400, FLAGS.fps, profile)
            width = 640
            height = 400
        depth = pipeline.createStereoDepth()
        depth.setConfidenceThreshold(200)
        depth.setMedianFilter(depthai.StereoDepthProperties.MedianFilter.KERNEL_7x7)
        depth.setLeftRightCheck(False)
        depth.setExtendedDisparity(False)
        depth.setSubpixel(False)
        left.out.link(depth.left)
        right.out.link(depth.right)
        depth.disparity.link(videoEnc.input)
    else:
        cam = pipeline.createColorCamera()
        if FLAGS.camera == 'rgb':
            cam.setBoardSocket(depthai.CameraBoardSocket.RGB)
        elif FLAGS.camera == 'left':
            cam.setBoardSocket(depthai.CameraBoardSocket.LEFT)
        elif FLAGS.camera == 'right':
            cam.setBoardSocket(depthai.CameraBoardSocket.RIGHT)
        cam.setFps(FLAGS.fps)
        cam.setInterleaved(False)

        if FLAGS.resolution == '1080p':
            cam.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
            videoEnc.setDefaultProfilePreset(1920, 1080, FLAGS.fps, profile)
            width = 1920
            height = 1080
        elif FLAGS.resolution == '4k':
            cam.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_4_K)
            videoEnc.setDefaultProfilePreset(3840, 2160, FLAGS.fps, profile)
            width = 3840
            height = 2160
        elif FLAGS.resolution == '12mp':
            cam.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_12_MP)
            videoEnc.setDefaultProfilePreset(4056, 3040, FLAGS.fps, profile)
            width = 4056
            height = 3040
        cam.video.link(videoEnc.input)

    videoEnc.setFrameRate(FLAGS.fps)
    videoEnc.setNumBFrames(0)
    videoEnc.setKeyframeFrequency(int(FLAGS.fps))
    if FLAGS.bitrate > 0:
        videoEnc.setBitrate(FLAGS.bitrate)
    print(f'bitrate: {videoEnc.getRateControlMode()}/{videoEnc.getBitrate()}')
    videoOut = pipeline.createXLinkOut()
    videoOut.setStreamName('encoded')
    videoEnc.bitstream.link(videoOut.input)

    rtsp_path = FLAGS.rtsp_path
    if not rtsp_path:
        rtsp_path = f'/{FLAGS.camera}_{FLAGS.resolution}'
    rtsp_server = GstRtspServer.RTSPServer()
    rtsp_server.set_address('0.0.0.0')
    rtsp_server.set_service(str(FLAGS.rtsp_port))
    media_factory = MediaFactory(FLAGS.encoder, width, height, int(FLAGS.fps))
    media_factory.set_shared(True)
    rtsp_server.get_mount_points().add_factory(rtsp_path, media_factory)
    rtsp_server.attach(None)
    Gst.init(None)
    def _thread_glib():
        loop = GLib.MainLoop()
        loop.run()
    t = threading.Thread(target = _thread_glib)
    t.start()

    print(f'Starting streaming {FLAGS.camera}@{FLAGS.resolution} to {rtsp_path} ...')

    with depthai.Device(pipeline) as device:
        queue = device.getOutputQueue(name='encoded', maxSize=30, blocking=True)
        while True:
            media_factory.send_pkt(queue.get())

if __name__ == '__main__':
  app.run(main)
