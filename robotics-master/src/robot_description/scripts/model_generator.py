from lib import * 
import math

PI = math.pi
default_inertial = [0.0005,0.0005,0.0005]
default_mass = 10
zero_pose = Pose(Location(0,0,0), Orientation(0,0,0))
width = 0.3
height = 2 * width
depth =  width/6

# Robot Arm

ARM_1_LENGTH = width/2
ARM_2_LENGTH = width/2
ARM_RADIUS = width/30

ARM_BASE_RADIUS = width/8
ARM_BASE_LENGTH = width/4

ARM_BASE_TOP_LENGTH = width/10
ARM_BASE_TOP_RADIUS = width/30

arm_mass            = default_mass/10
arm_base_mass       = default_mass
arm_base_top_mass   = default_mass/20

arm_inertial            = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(arm_mass, ARM_1_LENGTH, ARM_RADIUS)
arm_base_inertial       = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(arm_base_mass, ARM_BASE_LENGTH, ARM_BASE_RADIUS)
arm_base_top_inertial   = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(arm_base_top_mass, ARM_BASE_TOP_LENGTH, ARM_BASE_TOP_RADIUS)


armBasePos = Pose(Location(0, 0, 0), Orientation(0, 0, 0))
armBaseLink = CylindericalLink('arm_base', armBasePos, ARM_BASE_LENGTH, arm_base_mass, ARM_BASE_RADIUS, arm_base_inertial)
armBaseTopPos = Pose(Location(armBasePos.loc.x, armBasePos.loc.y, armBasePos.loc.z + ARM_BASE_LENGTH/2 + ARM_BASE_TOP_LENGTH/2), Orientation(0,0,0))
armBaseTopJoiPos = Pose(Location(0, 0, -ARM_BASE_TOP_LENGTH/2), Orientation(0, 0, 0))
armBaseTopLink = CylindericalLink('arm_base_top', armBaseTopPos, ARM_BASE_TOP_LENGTH, arm_base_top_mass, ARM_BASE_TOP_RADIUS, arm_base_top_inertial)
armBase_armBaseTop = RevoluteJoint('armBase_armBaseTop', armBaseTopJoiPos, armBaseTopLink.name, armBaseLink.name, None, None, Orientation(0, 0, 1))


arm1Pos = Pose(Location(armBaseTopPos.loc.x, armBaseTopPos.loc.y, armBaseTopPos.loc.z + ARM_BASE_TOP_LENGTH/2 + ARM_1_LENGTH/2), Orientation(0,0,0))
arm1JoiPos = Pose(Location(0, 0, -ARM_1_LENGTH/2), Orientation(0, 0, 0))
arm1Link = CylindericalLink('arm1', arm1Pos, ARM_1_LENGTH, arm_mass, ARM_RADIUS, arm_inertial)
armBaseTop_arm1 = RevoluteJoint('armBaseTop_arm1', arm1JoiPos, arm1Link.name, armBaseTopLink.name, PI, -PI, Orientation(1, 0, 0))
arm2Pos = Pose(Location(arm1Pos.loc.x, arm1Pos.loc.y, arm1Pos.loc.z + ARM_1_LENGTH/2 + ARM_2_LENGTH/2), Orientation(0,0,0))
arm2JoiPos = Pose(Location(0, 0, -ARM_2_LENGTH/2), Orientation(0, 0, 0))
arm2Link = CylindericalLink('arm2', arm2Pos, ARM_2_LENGTH, arm_mass, ARM_RADIUS, arm_inertial)
arm1_arm2 = RevoluteJoint('arm1_arm2', arm2JoiPos, arm2Link.name, arm1Link.name, PI, -PI, Orientation(1, 0, 0))

# Plugin
# ----------------------------------------------
arm_control_plugin = Plugin("arm_controller","libarm_controller.so", {})




#Gripper 

palm_radius     = width/20
finger_radius   = width/80
palm_length     = width/20
finger_length   = width/6
palm_mass       = default_mass/30
finger_mass     = default_mass/40
palm_inertial   = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(palm_mass, palm_length, palm_radius)
finger_inertial = math_helper(Orientation(0, 0, 0)).inertial_cylinderical(finger_mass, finger_length, finger_radius)


palm_pos = Pose(Location(arm2Pos.loc.x, arm2Pos.loc.y, arm2Pos.loc.z + ARM_2_LENGTH/2 + palm_length/2), Orientation(0,0,0))

palm = CylindericalLink("palm", Pose( Location( palm_pos.loc.x, palm_pos.loc.y, palm_pos.loc.z ), Orientation( palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), palm_length, palm_mass, palm_radius, palm_inertial )

finger_one      = CylindericalLink("finger_one",    Pose( Location(palm_pos.loc.x, palm_pos.loc.y+(palm_radius/2), palm_pos.loc.z+(palm_length/2)+(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_two      = CylindericalLink("finger_two",    Pose( Location(palm_pos.loc.x, palm_pos.loc.y-(palm_radius/2), palm_pos.loc.z+(palm_length/2)+(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_three    = CylindericalLink("finger_three",  Pose( Location(palm_pos.loc.x+(palm_radius/2), palm_pos.loc.y, palm_pos.loc.z+(palm_length/2)+(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_four     = CylindericalLink("finger_four",   Pose( Location(palm_pos.loc.x-(palm_radius/2), palm_pos.loc.y, palm_pos.loc.z+(palm_length/2)+(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)

finger_one_tip  = CylindericalLinkWithSensor("finger_one_tip",   Pose( Location(palm_pos.loc.x, palm_pos.loc.y+(palm_radius/2), palm_pos.loc.z+(palm_length/2)+((3*finger_length)/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_two_tip  = CylindericalLinkWithSensor("finger_two_tip",   Pose( Location(palm_pos.loc.x, palm_pos.loc.y-(palm_radius/2), palm_pos.loc.z+(palm_length/2)+((3*finger_length)/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_three_tip= CylindericalLinkWithSensor("finger_three_tip", Pose( Location(palm_pos.loc.x+(palm_radius/2), palm_pos.loc.y, palm_pos.loc.z+(palm_length/2)+((3*finger_length)/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)
finger_four_tip = CylindericalLinkWithSensor("finger_four_tip",  Pose( Location(palm_pos.loc.x-(palm_radius/2), palm_pos.loc.y, palm_pos.loc.z+(palm_length/2)+((3*finger_length)/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), finger_length, finger_mass, finger_radius, finger_inertial)

palm_joint  = RevoluteJoint("palm_joint", Pose( Location(0, 0, -(palm_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "palm", arm2Link.name, PI, -PI, Orientation(0, 0, 1))

finger_one_joint    = RevoluteJoint("finger_one_joint",     Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_one",   "palm",  0,     PI/4, Orientation(1, 0, 0))
finger_two_joint    = RevoluteJoint("finger_two_joint",     Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_two",   "palm", -PI/4,  0,    Orientation(1, 0, 0))
finger_three_joint  = RevoluteJoint("finger_three_joint",   Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_three", "palm",  PI/4,  0,    Orientation(0, 1, 0))
finger_four_joint   = RevoluteJoint("finger_four_joint",    Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_four",  "palm",  0,    -PI/4, Orientation(0, 1, 0))

finger_one_tip_joint    = RevoluteJoint("finger_one_tip_joint",   Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_one_tip",   "finger_one",  -PI/2, 0,    Orientation(1, 0, 0))
finger_two_tip_joint    = RevoluteJoint("finger_two_tip_joint",   Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_two_tip",   "finger_two",   0,    PI/2, Orientation(1, 0, 0))
finger_three_tip_joint  = RevoluteJoint("finger_three_tip_joint", Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_three_tip", "finger_three", 0,   -PI/2, Orientation(0, 1, 0))
finger_four_tip_joint   = RevoluteJoint("finger_four_tip_joint",  Pose( Location(0, 0, -(finger_length/2)), Orientation(palm_pos.orie.x, palm_pos.orie.y, palm_pos.orie.z)), "finger_four_tip",  "finger_four",  PI/2, 0,    Orientation(0, 1, 0))

gripper_plugin = Plugin("gripper_plugin", "libgripper_plugin.so", {})

#Build The model
sdf = Sdf.createSdfDoc()
root = Sdf.getRootElement()
links_joints = [
# Arm links and joints
armBaseLink, armBaseTopLink, armBase_armBaseTop, arm1Link, armBaseTop_arm1, arm2Link, arm1_arm2,
# Gripper links and joints
palm, palm_joint, finger_one, finger_one_joint, finger_two, finger_two_joint, finger_three, finger_three_joint, 
finger_four, finger_four_joint, finger_one_tip, finger_one_tip_joint, finger_two_tip, finger_two_tip_joint, 
finger_three_tip, finger_three_tip_joint, finger_four_tip, finger_four_tip_joint, 
gripper_plugin, arm_control_plugin]#,tsp_plugin]

model = Model("robot",links_joints)

#Append to root element and create sdf File.
root.appendChild(model)
Sdf.createSdfFile("../models/model.sdf")
