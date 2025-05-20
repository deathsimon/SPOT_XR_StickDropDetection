"""
Reuests coming from the client to poll for each image.
Spots native cameras are running on roughly 10Hz, so don't poll much faster than this.
Upon poll, the currently saved image is returned. If the poll comes faster in, than the ros2 message,
repeat images may be returned!
To check if there are new images, poll IMG_CHANGED
"""

class ApiEntry:
    def __init__(self, name, iden, path):
        self.name = name
        self.id = iden
        self.path = path
    def __eq__(self, item):
        return self.name == item or self.id == item or self.path == item

def grp(upper4 : int, lower4 : int):
    return upper4 << 4 | lower4

def grp_list_incl(upper4 : int, lower41 : int, lower42 : int):
    return list(range(grp(upper4, lower41), grp(upper4, lower42)+1))

class TyApiRequests: # TODO: change all ids to (group_id << 16) | id
    VALUES = [
        ApiEntry("DEPTH_IMG_B",           (0x0 << 4 | 1), "/depth/back"),
        ApiEntry("DEPTH_IMG_FL",          (0x0 << 4 | 2), "/depth/frontleft"),
        ApiEntry("DEPTH_IMG_FR",          (0x0 << 4 | 3), "/depth/frontright"),
        ApiEntry("DEPTH_IMG_L",           (0x0 << 4 | 4), "/depth/left"),
        ApiEntry("DEPTH_IMG_R",           (0x0 << 4 | 5), "/depth/right"),
        ApiEntry("DEPTH_IMG_HAND",        (0x0 << 4 | 6), "/depth/hand"),

        ApiEntry("CAMERA_IMG_B",          (0x0 << 4 | 7),  "/camera/back"),
        ApiEntry("CAMERA_IMG_FL",         (0x0 << 4 | 8),  "/camera/frontleft"),
        ApiEntry("CAMERA_IMG_FR",         (0x0 << 4 | 9),  "/camera/frontright"),
        ApiEntry("CAMERA_IMG_L",          (0x0 << 4 | 10), "/camera/left"),
        ApiEntry("CAMERA_IMG_R",          (0x0 << 4 | 11), "/camera/right"),
        ApiEntry("CAMERA_IMG_HAND",       (0x0 << 4 | 12), "/camera/hand"),

        ApiEntry("DEPTH_FMT",             (0x1 << 4 | 1),  ""),
        ApiEntry("RGB_FMT",               (0x1 << 4 | 2),  ""),
        ApiEntry("IMG_CHANGED",           (0x1 << 4 | 3),  ""),
        ApiEntry("STATUS",                (0x1 << 4 | 4),  ""),
        ApiEntry("JOINT_STATE",           (0x1 << 4 | 5),  ""),
        ApiEntry("MANIPULATION_STATE",    (0x1 << 4 | 6),  ""),
        ApiEntry("ODOMETRY",              (0x1 << 4 | 7),  ""),
        ApiEntry("BODY_POSE",             (0x1 << 4 | 8),  ""),
        ApiEntry("HAND_POSITION",         (0x1 << 4 | 9),  ""),

        ApiEntry("ESTOP_GENTLE",          (0x2 << 4 | 1), "/estop/gentle"),
        ApiEntry("ESTOP_HARD",            (0x2 << 4 | 2), "/estop/hard"),
        ApiEntry("ESTOP_RELEASE",         (0x2 << 4 | 3), "/estop/release"),
        ApiEntry("POWER_OFF",             (0x2 << 4 | 4), "/power_off"),
        ApiEntry("POWER_ON",              (0x2 << 4 | 5), "/power_on"),
        ApiEntry("STAND",                 (0x2 << 4 | 6), "/stand"),
        ApiEntry("SIT",                   (0x2 << 4 | 7), "/sit"),
        ApiEntry("STOP",                  (0x2 << 4 | 8), "/stop"),

        ApiEntry("ARM_STOW",              (0x2 << 4 | 9), "/arm_stow"),
        ApiEntry("ARM_UNSTOW",            (0x2 << 4 | 10), "/arm_unstow"),
        ApiEntry("ARM_CARRY",             (0x2 << 4 | 11), "/arm_carry"),
        ApiEntry("GRIPPER_OPEN",          (0x2 << 4 | 12), "/open_gripper"),
        ApiEntry("GRIPPER_CLOSE",         (0x2 << 4 | 13), "/close_gripper"),

        ApiEntry("MOVE_RELATIVE",         (0x3 << 4 | 1), ""),
        ApiEntry("RC_MOVE_TO_POSE",       (0x3 << 4 | 2), ""),
        ApiEntry("RC_GAZE_AT",            (0x3 << 4 | 3), ""),
        ApiEntry("MANIP_PICK_FROM_IMAGE", (0x3 << 4 | 4), ""),
        ApiEntry("MANIP_PICK_FROM_POINT", (0x3 << 4 | 5), ""),
        ApiEntry("RC_MOVE_RELATIVE",      (0x3 << 4 | 6), ""),
        
        ApiEntry("ARM_VEL",               (0x3 << 4 | 7), "/arm_vel"),
        ApiEntry("HAND_VEL",              (0x3 << 4 | 8), "/hand_vel"),
        ApiEntry("GRIPPER_OPEN_ANGLE",    (0x3 << 4 | 9), "/open_gripper_angle"),
        ApiEntry("SET_ARM_JOINTS",        (0x3 << 4 | 10), "/set_arm_joints"),
        ]
    
    def __init__(self, vals = None):
        if vals:
            self.VALUES = [entry for entry in self.VALUES if entry in vals]
            return
        
        self.CAMERAS = TyApiRequests(grp_list_incl(0, 1, 12))
        self.DEPTH_CAMERAS = TyApiRequests(grp_list_incl(0, 1, 6))
        self.RGB_CAMERAS = TyApiRequests(grp_list_incl(0, 7, 12))
        self.HAND_CAMS = TyApiRequests([6,12])
        
        self.TOPICLESS_REQUESTS = TyApiRequests(grp_list_incl(1, 1, 9))
        
        self.SERVICES = TyApiRequests(grp_list_incl(2, 1, 13))
        self.HAND_SERVICES = TyApiRequests(grp_list_incl(2, 9, 13))
        
        self.ACTIONS = TyApiRequests(grp_list_incl(3, 1, 10))
        
        self._catergories = vars(self)
        self._catergories.pop('VALUES', None)
    
    def __call__(self, item):
        for v in self.VALUES:
            if v == item:
                return v
        return None 
    
    def __contains__(self, item):
        for v in self.VALUES:
            if v == item:
                return True
        return False
    
    def names(self):
        return [x.name for x in self.VALUES]
    
    def ids(self):
        return [x.id for x in self.VALUES]
    
    def paths(self):
        return [x.path for x in self.VALUES]
    
    def items(self):
        return list(zip(self.names(), self.paths()))
    
    def get_category(self, item):
        item = self.__call__(item)
        for k,v in self._catergories:
            if item in v:
                return k
        return "VALUES"

ApiRequests = TyApiRequests()
