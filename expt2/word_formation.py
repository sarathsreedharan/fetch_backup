import yaml
import tf
import time
import rospy
import copy
from multiprocessing import Queue

X_DELTA = 0.02
Y_DELTA = 0.02
Z_DELTA = 0.04
HOLDING_DELTA = 0.5
ONTABLE_HEIGHT = 0.900
HOLDING_HEIGHT = 1.031 
class monitor:
    '''
        A class to monitor the current human actions
    '''
    def __init__(self, sequence, marker_config, output_state_queue, output_state_pos_queue, output_action_queue, input_robot_action_queue, verbose=False, debug=False):
        """
        """
        #rospy.init_node('monitor')
        self.verbose = verbose
        self.debug = debug
        self.human_action = sequence
        self.current_action = None
        self.prev_state_pos = {}
        self.current_state_pos = {}
        self.loc_marker_pos = {}
        self.full_state = []
        self.prev_full_state = []
        self.first_time_table_flag = True
        self.first_time_state_flag = True
        self.current_visible_list = set()
        

        with open(marker_config) as config_fd:
            tmp_config = yaml.load(config_fd)
        for key in tmp_config.keys():
            setattr(self, key, tmp_config[key])
        self.tf_listener = tf.TransformListener()
        time.sleep(2)
        for obj in self.objects:
            self.current_state_pos[obj] = []
        del tmp_config

    def get_marker_position(self, marker):
        marker_pos = ()
        marker_orient = ()
        #try:
        #tf_listener = tf.TransformListener()
        #if self.tf_listener.frameExists(marker):
        try:
            print "Marker exists for", marker
            marker_pos, marker_orient = self.tf_listener.lookupTransform('/base_link', marker,rospy.Time(0) )
            #print marker_pos
        #except Exception as exc:
        #    print "Marker doesn't exists for", marker, exc

        except:
            #time.sleep(1)
            print "Reached here for marker", marker
            try:
                marker_pos, marker_orient = self.tf_listener.lookupTransform('/base_link', marker, rospy.Time(0))
            except:
                print "Skipping"
        return marker_pos, marker_orient

    def get_current_state(self):
        #time.sleep(0.5)
        #self.tf_listener.clear()
        self.prev_state_pos = self.current_state_pos
        self.current_state_pos = {}
        self.prev_visible_list = copy.deepcopy(self.current_visible_list)
        self.current_visible_pos = set()
        for obj in self.objects:
            try:
                marker_pos, marker_orient = self.get_marker_position(self.marker_map[obj])
            except Exception as exc:
                #print "Failed for obj ", obj, "with msg",exc
                marker_pos = ()
                marker_orient = ()
            if len(marker_pos) != 0:
                #print "Are we here for vlkock",obj
                self.current_state_pos[obj] = list(marker_pos)
                self.current_visible_list.add(obj)
            else:
                #print "keys ", self.prev_state_pos.keys()
                self.current_state_pos[obj] = self.prev_state_pos[obj]
            for marker in self.location_markers:
                marker_pos, marker_orient = self.get_marker_position(marker)
                self.loc_marker_pos[marker] = list(marker_pos)
            self.first_time_table_flag = False

    def monitor(self):
            #time.sleep(10)
        self.get_current_state()
        return True


