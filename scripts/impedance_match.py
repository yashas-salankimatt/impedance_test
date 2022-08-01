import numpy as np
import mujoco_py
from scipy.spatial.transform import Rotation as R


class idcontrol():
    def __init__(self,sim,x0):
        self.ee_site='mep:ee'
        self.sim=sim
        self.J0=np.vstack((np.reshape(self.sim.data.get_site_jacp(self.ee_site)[:-3],(3,-1)),np.reshape(self.sim.data.get_site_jacr(self.ee_site)[:-3],(3,-1))))
        # self.xd = np.array(self.sim.data.get_site_xpos('target'))
        # self.qd = R.from_quat(np.array([0,0,0,1])).as_matrix()
        self.xdes=np.zeros(6)
        self.xdes_1=np.zeros(6)
        self.xdes_2=np.zeros(6)
        self.k1=10*np.eye(6)
        self.m1=5*np.eye(6)
        self.b1=20*np.eye(6)
        self.x0=x0
    
    def idc(self):

        ## Get the position/orientation information from previous runs ##
        dt = 0.1

        ## Get Jacobian of end effector ##
        J=np.vstack((np.reshape(self.sim.data.get_site_jacp(self.ee_site)[:-3],(3,-1)),np.reshape(self.sim.data.get_site_jacr(self.ee_site)[:-3],(3,-1))))
        Jtinv = np.linalg.inv(np.transpose(J))
        Jinv = np.linalg.inv(J)
        ## Get bias terms (coriollis/gravity) ##
        h=self.sim.data.qfrc_bias[:6]

        ## Get full mass matrix ##
        mass_matrix = np.ndarray(shape=(len(self.sim.data.qvel) ** 2,), dtype=np.float64, order='C')
        mujoco_py.cymj._mj_fullM(self.sim.model, mass_matrix, self.sim.data.qM)
        mass_matrix=np.reshape(mass_matrix, (len(self.sim.data.qvel), len(self.sim.data.qvel)))
        
        mass_matrix =  mass_matrix[:6, :6]
        ## Get time derivative of Jacobian ##
        dJdt=(J-self.J0)/self.sim.model.opt.timestep
        self.J0=np.array(J)

        ## Me = J^-T M J^-1
        Me = Jtinv@mass_matrix@Jinv

        ## Ce ##
        Ce = -Jtinv@mass_matrix@Jinv@dJdt@Jinv

        ## Ke ##
        Ke = Jtinv@h

        ## Get inverse of Me and Ce ##
        Meinv = np.linalg.inv(Me)
        try:
            Ceinv = np.linalg.inv(Ce)
        except:
            Ceinv = np.zeros(np.shape(Meinv))

        ## dF = sensor data - calculated data ##
        Fcalc=self.m1@self.sim.data.cacc[-1][:]+self.b1@np.reshape((np.array(self.sim.data.body_xvelp[-1][:]),np.zeros(3)),(6,))+self.k1@(np.reshape((np.array(self.sim.data.body_xpos[-1][:]),np.zeros(3)),(6,))-self.x0)
        sensordata = np.append(self.sim.data.sensordata,np.zeros(3))
        dF= sensordata-Fcalc

        self.xdes = (Meinv/dt**2+Ce/dt)@((2/dt*Me+Ce)@self.xdes_1/dt-Me/dt**2@self.xdes_2-Ke+dF)
        self.xdes_2=self.xdes_1
        self.xdes_1=self.xdes
        return self.xdes