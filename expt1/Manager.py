import sys
import os
from plan_monitoring import monitor as monitor_local
from RobotExecutor import RobotActionExecutor
from multiprocessing import Process
from Queue import Queue
import thread
import rospy
import time
import json
from std_msgs.msg import String
import subprocess
import yaml
from state_monitor.srv import monitor
import shutil

WAIT_LIMT = 6

pathname = os.path.dirname(sys.argv[0])
result_plan_file = "/tmp/current_plan"

rospy.init_node('test_manager')

orig_domain = sys.argv[1]
orig_problem = sys.argv[2]
prob_template = sys.argv[3]

INIT_STATE_STRING = "<%init_state%>"


humanActionQueue = Queue()
currentStateQueue = Queue()
currentStatePosQueue = Queue()
robotActionQueue = Queue()

lastHumanAction = None
completedHumanAction = None
human_action_list = []
current_pos_list = []

last_seen_state = None
last_seen_pos = None

CURRENT_FEATS = "/tmp/final_features"
CURRENTPREFIX = "/tmp/plan_prefix"

prefix_set = False


def find_plan(prob_domain, prob_file, plan_count = 0):
    global prefix_set
    sol_file = "/tmp/plan"
    try:
        os.remove(sol_file)
    except:
        print "Just ignore"
    if prefix_set:
       prefix_str = " '"+ CURRENTPREFIX + "'" 
    else:
       prefix_str = ""
    plan_cmmnd = "/usr/bin/java -jar '/home/yochan/test_expl_run_new.jar' '"+prob_domain+"' '"+prob_file+"' '" + sol_file + "'" + prefix_str
    print plan_cmmnd
    ret_status = subprocess.call(plan_cmmnd,shell=True)
    if ret_status != 0:
        return []
    shutil.copy(CURRENT_FEATS, CURRENTPREFIX)
    with open(CURRENT_FEATS) as c_fd:
        feature_list = c_fd.readlines()
    new_features = feature_list[:plan_count]
    with open(CURRENTPREFIX, 'w') as c_fd:
        for feat in new_features:
            c_fd.write(feat)
    prefix_set = True
    with open(sol_file) as p_fd:
        return map(str.strip, p_fd.readlines())

def replan(curr_state, prob_domain, prob_template, plan_count = 0):
    tmp_file = "/tmp/ptob_file"
    # Assuming the template contains the goal
    for item in curr_state:
        if "pre_flag" in item:
            curr_state.remove(item)
    if "(human_can_prep_action)"  in curr_state:
        curr_state.remove("(human_can_prep_action)") 
    if "(robot_can_prep_action)" not in curr_state:
        curr_state.append("(robot_can_prep_action)") 

    with open(prob_template) as templ_fd:
        prob_templ_str_raw = map(str.strip, templ_fd.readlines())
        prob_templ_str = '\n'.join(prob_templ_str_raw)
    new_prob_templ_str = prob_templ_str.replace("<%init-state%>", "\n".join(curr_state))
    with open(tmp_file, 'w') as tmp_fd:
        tmp_fd.write(new_prob_templ_str)
    plan =  find_plan(prob_domain, tmp_file, plan_count)
    #if len(plan) == 0:
    #    for item in curr_state:
    #        if "pre_flag" in item:
    #            curr_state.remove(item)
    #    if "(human_can_prep_action)" not in curr_state:
    #        curr_state.append("(human_can_prep_action)")
    #    if "(robot_can_prep_action)" in curr_state:
    #        curr_state.remove("(robot_can_prep_action)")
    #    with open(prob_template) as templ_fd:
    #        prob_templ_str_raw = map(str.strip, templ_fd.readlines())
    #        prob_templ_str = '\n'.join(prob_templ_str_raw)
    #    new_prob_templ_str = prob_templ_str.replace("<%init-state%>", "\n".join(curr_state))
    #    with open(tmp_file, 'w') as tmp_fd:
    #        tmp_fd.write(new_prob_templ_str)
    #    plan =  find_plan(prob_domain, tmp_file, plan_count)
    return plan

def send_robot_action():
    rate = rospy.Rate(1)
    pub_rob = rospy.Publisher('completed_robot_action', String, queue_size=10) 
    while True:
        if not robotActionQueue.empty():
            pub_rob.publish(robotActionQueue.get())
        rate.sleep()
            

def check_human_action(last_seen):
    global lastHumanAction
    global completedHumanAction
    if completedHumanAction != None and last_seen.data != completedHumanAction and last_seen.data != "NOOP":
        print "Mismatch in the last seen action",last_seen.data
        exit(0)

def add_human_action(data):
    if data.data != "NOOP":
        if len(human_action_list)!= 0:
            last_seen = human_action_list[-1]
        else:
            last_seen = None
        if data.data != last_seen:
            #print "Seen human action",data.data
            humanActionQueue.put(data.data)
        human_action_list.append(data.data)
def collect_human_action():
    while True:
        #print "We are here"
        rospy.Subscriber('scene_monitor_action', String, add_human_action)

def check_for_human(humanQueue, configs):
    last_status = ""
    while True:
        if humanQueue.empty():
            human_status = humanQueue.get()
            if human_status != last_status:
                last_status = human_status
                if human_status == "in":
                    for ind in len(configs['speed_params']):
                        rospy.set(configs['speed_params'][ind], configs['slow_speed'][ind])
                else:
                    for ind in len(configs['speed_params']):
                        rospy.set(configs['speed_params'][ind], configs['normal_speed'][ind])
 
def get_current_state(data):
    global last_seen_state
    if data.data !=  last_seen_state or  currentStateQueue.empty():
       print "BUHAHAHA -> new state added",data.data," is queue empty ",currentStateQueue.empty()
       last_seen_state = data.data
       currentStateQueue.put(data.data)

def get_current_pos(data):
    global last_seen_pos
    if data.data != last_seen_pos:
        last_seen_pos = data.data
        currentStatePosQueue.put(data.data)

raw_plan = []
test_queue = Queue()
monit = monitor_local([],'sample.yaml',currentStateQueue, currentStatePosQueue, humanActionQueue, robotActionQueue)
monit.detect_state()
robo_exec = RobotActionExecutor(monit.location_markers, monit.objects, monit.loc_marker_pos)

with open('sample.yaml') as cg_fd:
     configs = yaml.load(cg_fd)

#    raw_plan = pl_fd.readlines()

#plan = []
#for act in raw_plan:
#    plan.append(act.strip())
curr_plan = find_plan(orig_domain, orig_problem)
#monit_process = Process(target=monit.monitor)
#monit_process.start()
#print curr_plan
monit_thread = thread.start_new_thread(send_robot_action,())
#monit_thread = thread.start_new_thread(collect_human_action,())
#time.sleep(10000)
print "Execution starting"
current_pos = monit.current_state_pos
#exit(0)
goal_reached = False
expected_human_action = ""
state_srv = rospy.ServiceProxy('state_monitor', monitor)

def get_precondition(action_parts):
    preconditions = []
    action_name = action_parts[0].lower()
    if "unstack" in action_name:
        handempty = "(r_handempty)"
        on_pred = "(on "+ action_parts[1] + " " + action_parts[2] + ")"
        clear_pred = "(clear "+ action_parts[1] + ")"
        preconditions = [handempty, on_pred, clear_pred]
    elif "pickup" in action_name or "pick-up" in action_name:
        handempty = "(r_handempty)"
        ontable_pred = "(ontable "+ action_parts[1] + ")"
        clear_pred = "(clear "+ action_parts[1] + ")"
        preconditions = [handempty, ontable_pred, clear_pred]
    elif "stack"  in action_name:
        holding = "(r_holding " + action_parts[1] + ")"
        clear_pred = "(clear "+ action_parts[2] + ")"
        preconditions = [ holding, clear_pred]
    elif "putdown" in action_name or "put-down" in action_name:
        holding = "(r_holding " + action_parts[1] + ")"
        preconditions = [ holding]
    return preconditions

def get_effects(action_parts):
    effects = []
    action_name = action_parts[0].lower()
    if "unstack" in action_name:
        holding = "(holding " + action_parts[1] + ")"
        clear_pred = "(clear "+ action_parts[2] + ")"
        effects = [holding, clear_pred]
    elif "pickup" in action_name or "pick-up" in action_name:
        holding = "(holding " + action_parts[1] + ")"
        effects = [holding]
    elif "stack"  in action_name:
        handempty = "(handempty)"
        on_pred = "(on "+ action_parts[1] + " " + action_parts[2] + ")"
        clear_pred = "(clear "+ action_parts[1] + ")"
        effects = [handempty, on_pred, clear_pred]
    elif "putdown" in action_name or "put-down" in action_name:
        handempty = "(handempty)"
        ontable_pred = "(ontable "+ action_parts[1] + ")"
        clear_pred = "(clear "+ action_parts[1] + ")"
        effects = [handempty, ontable_pred, clear_pred]
    return effects


def wait_for_preconditions(action):
    action_parts = action.split('(')[1].split(')')[0].split(' ')
    preconditions = get_precondition(action_parts)
    precondition_found = False
    curr_time = time.time()
    print "precondition is", preconditions
    current_state = None
    old_time = curr_time
    while not precondition_found:
        srvr_msg = json.loads(str(state_srv("test").state))
        print "curr_time 1",curr_time
        curr_time = time.time()
        print "curr_time 2",curr_time
        if (curr_time - old_time) > WAIT_LIMT:
            print "Wait time crossed"
            return False
        current_state = srvr_msg["state"] 
        #print current_state
        if current_state:
            precondition_found = True
            for pred in preconditions:
                if pred not in current_state:
                    precondition_found = False

    return True

def add_human_presence(data):
    humanQueue.put(data.data)

def  wait_for_completion(action):
    action_parts = action.split('(')[1].split(')')[0].split(' ')
    effects = get_effects(action_parts)
    effects_found = False
    curr_time = time.time()
    print "effects are", effects
    current_state = None
    old_time = curr_time
    while not effects_found:
        srvr_msg = json.loads(str(state_srv("test").state))
        print "curr_time 1",curr_time
        curr_time = time.time()
        print "curr_time 2",curr_time
        if (curr_time - old_time) > WAIT_LIMT:
            print "Wait time crossed"
            return False
        current_state = srvr_msg["state"]
        #print current_state
        if current_state:
            effects_found = True
            for pred in effects:
                if pred not in current_state:
                    effects_found = False

    return True


new_curr_plan = None
replan_initiated = False
while not goal_reached:
    if replan_initiated:
        curr_plan = new_curr_plan 
    exec_status = False
    replan_initiated = False
    new_curr_plan = None
    current_plan_cnt = 0
    for item in curr_plan:
        #time.sleep(0.5)
        if not replan_initiated:
            print "action is ",item
            state_msg = state_srv("test")
            print "state msg",dir(state_msg)
            srvr_msg = json.loads(str(state_msg.state))
            if "robot" in item and item != "robot_noop":
                if "pre" not in item and item != "robot_noop":
                    action_parts = item[7:].split(')')[0].split(' ')
                    print "Getting new positions"
                    current_pos = srvr_msg["state_pos"]
                    print "Getting required preconditions"
                    wait_result = wait_for_preconditions(item)
                    #if wait_result:
                    print "Printing execution", action_parts, current_pos
                    try:
                        srvr_msg = json.loads(str(state_srv("("+ item[7:]).state))
                        exec_status = robo_exec.execute_action(action_parts[0].lower(), action_parts[1:], current_pos, monit.loc_marker_pos)
                    except:
                        wait_result = False
                    if not wait_result:
                        srvr_msg = json.loads(str(state_srv("test").state))
                        current_state = srvr_msg["state"]
                        new_curr_plan = replan(current_state, orig_domain, prob_template, current_plan_cnt + 1)
                        replan_initiated = True
                        current_plan_cnt = 0
                    if exec_status:
                        #srvr_msg = json.loads(str(state_srv("("+ item[7:]).state))
                        srvr_msg = json.loads(str(state_srv("test").state))
                        exec_status = False
                    #while not humanActionQueue.empty():
                    #    last_seen = humanActionQueue.get()
            else:
                if 'noop' not in item:
                    human_status = wait_for_completion(item) #, current_state, current_pos)
                    if not human_status:
                        srvr_msg = json.loads(str(state_srv("test").state))
                        current_state = srvr_msg["state"]
                        new_curr_plan = replan(current_state, orig_domain, prob_template, current_plan_cnt + 1)
                        replan_initiated = True
                        current_plan_cnt = 0
                else:
                #if 'noop' not in item:
                    time.sleep(5)
                #else:
                #    time.sleep(5)
            print "We are here"
            current_plan_cnt = current_plan_cnt + 1

#monit_process.terminate()
