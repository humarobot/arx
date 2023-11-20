import mujoco as mj
import numpy as np
from mujoco_base import MuJoCoBase
from mujoco.glfw import glfw
import rospy
import rospkg
from std_msgs.msg import Float64MultiArray,Bool
from geometry_msgs.msg import Pose,Twist



class UltronSim(MuJoCoBase):
  def __init__(self, xml_path):
    super().__init__(xml_path)
    self.simend = 1000.0
    self.rate = rospy.Rate(500)
    # print('Total number of DoFs in the model:', self.model.nv)
    # print('Generalized positions:', self.data.qpos)  
    # print('Generalized velocities:', self.data.qvel)
    # print('Actuator forces:', self.data.qfrc_actuator)
    # print('Actoator controls:', self.data.ctrl)
    # mj.set_mjcb_control(self.controller)
    # * Set subscriber and publisher
    self.pubJoints = rospy.Publisher('/ultron_mj/jointsPosVel', Float64MultiArray, queue_size=10)

    rospy.Subscriber("/ultron_mj/jointsTorque", Float64MultiArray, self.controlCallback) 
    # * show the model
    mj.mj_step(self.model, self.data)
    # enable contact force visualization
    self.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        self.window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    # Update scene and render
    mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                        mj.mjtCatBit.mjCAT_ALL.value, self.scene)
    mj.mjr_render(viewport, self.scene, self.context)
    

  def controlCallback(self, data):
    self.data.ctrl[:] = data.data[:]

  def reset(self):
    # Set camera configuration
    self.cam.azimuth = 89.608063
    self.cam.elevation = -11.588379
    self.cam.distance = 5.0
    self.cam.lookat = np.array([0.0, 0.0, 1.5])

  # def controller(self, model, data):
  #   self.data.ctrl[0] = 100
  #   pass

  def simulate(self):
    while not glfw.window_should_close(self.window):
      simstart = self.data.time

      while (self.data.time - simstart <= 1.0/60.0 and not self.pause_flag):
        # Step simulation environment
        mj.mj_step(self.model, self.data)
        # * Publish joint positions and velocities
        jointsPosVel = Float64MultiArray()
        # get last 10 element of qpos and qvel
        qp = self.data.qpos.copy()
        qv = self.data.qvel.copy()
        jointsPosVel.data = np.concatenate((qp,qv))
        self.pubJoints.publish(jointsPosVel)
        self.rate.sleep()


      if self.data.time >= self.simend:
          break

      # get framebuffer viewport
      viewport_width, viewport_height = glfw.get_framebuffer_size(
          self.window)
      viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

      # Update scene and render
      mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                          mj.mjtCatBit.mjCAT_ALL.value, self.scene)
      mj.mjr_render(viewport, self.scene, self.context)

      # swap OpenGL buffers (blocking call due to v-sync)
      glfw.swap_buffers(self.window)

      # process pending GUI events, call GLFW callbacks
      glfw.poll_events()

    glfw.terminate()

def main():
    # ros init
    rospy.init_node('ultron_sim', anonymous=True)

    # get xml path
    rospack = rospkg.RosPack()
    rospack.list()
    hector_desc_path = rospack.get_path('ultron_pos')
    xml_path = hector_desc_path + "/urdf/ultron.xml"
    sim = UltronSim(xml_path)
    sim.reset()
    sim.simulate()

if __name__ == "__main__":
    main()
