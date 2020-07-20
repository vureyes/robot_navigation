#!/usr/bin/env python
import roslib; roslib.load_manifest('sound_play')
import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

last_sound = None
sound_client = None

def sound_node():
    rospy.init_node("sound_node", anonymous=True)
    rospy.Subscriber("/sound_string", String, accion_sound_cb)
    global sound_client
    sound_client = SoundClient()
    rospy.spin()

def accion_sound_cb(state):
    global last_sound
    sound = sound_client.voiceSound(state.data)
    sound.play()
    last_sound = state.data


if __name__ == '__main__':
    try:
        sound_node()
    except rospy.ROSInterruptException:
        pass