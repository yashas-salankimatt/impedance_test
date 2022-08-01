import numpy as np
import mujoco_py
from scipy.spatial.transform import Rotation as R


class idcontrol():
    def __init__(self,sim):
        self.ee_site='mep:ee'
        self.sim=sim
        self.J0=np.vstack((np.reshape(self.sim.data.get_site_jacp(self.ee_site)[:18],(3,-1)),np.reshape(self.sim.data.get_site_jacr(self.ee_site)[:18],(3,-1))))
        
        
        self.xdes=np.zeros(6)
        self.xdes_1=np.zeros(6)
        self.xdes_2=np.zeros(6)
        self.k1=10*np.eye(6)
        self.b1=20*np.eye(6)
        self.m1=60*np.eye(6)
    
    def idc(self,x0,ave_F,dt):

        ## Get the position/orientation information from previous runs ##
        # dt = self.sim.model.opt.timestep

        ## Get Jacobian of end effector ##
        J=np.vstack((np.reshape(self.sim.data.get_site_jacp(self.ee_site)[:18],(3,-1)),np.reshape(self.sim.data.get_site_jacr(self.ee_site)[:18],(3,-1))))
        Jtinv = np.linalg.inv(np.transpose(J))
        Jinv = np.linalg.inv(J)
        ## Get bias terms (coriollis/gravity) ##
        h=self.sim.data.qfrc_bias[:-1]

        ## Get full mass matrix ##
        mm = np.ndarray(shape=(len(self.sim.data.qvel) ** 2,), dtype=np.float64, order='C')
        mujoco_py.cymj._mj_fullM(self.sim.model, mm, self.sim.data.qM)
        mm=np.reshape(mm, (len(self.sim.data.qvel), len(self.sim.data.qvel)))
        mass_matrix=mm[:6,:6]

        ## Get time derivative of Jacobian ##
        dJdt=(J-self.J0)/dt
        self.J0=np.array(J)

        ## Me = J^-T M J^-1
        Me = Jtinv@mass_matrix@Jinv

        ## Ce ##
        Ce = -Jtinv@mass_matrix@Jinv@dJdt@Jinv

        ## Ke ##
        Ke = 0*Jtinv@h

        ## Get inverse of Me and Ce ##
        # Meinv = np.linalg.inv(Me/dt**2)
        # try:
        #     Ceinv = np.linalg.inv(Ce/dt)
        # except:
        #     Ceinv = np.zeros(np.shape(Meinv))
        
        try:
            Meinv = np.linalg.inv(Me/dt**2+Ce/dt)
        except:
            Meinv = np.linalg.inv(Me/dt**2)

        ## dF = sensor data - calculated data ##
        Fcalc=(self.m1@np.reshape([self.sim.data.cacc[-2][3:],np.zeros(3)],(6,))+self.b1@np.reshape([self.sim.data.body_xvelp[-2][:],np.zeros(3)],(6,))+self.k1@(np.reshape([self.sim.data.body_xpos[-2][:],np.zeros(3)],(6,))-x0))
        dF=np.reshape([ave_F,np.zeros(3)],(6,))-Fcalc
        dF[1:]=0
        # dF[1:]=0
        # Ke[1:]=0
        # self.xdes = (Meinv)@((2/dt*Me+Ce)@self.xdes_1-Me/dt@self.xdes_2-dt*(Ke+dF))

        self.xdes = (Meinv)@((2/dt*Me+Ce)@self.xdes_1/dt-Me/dt**2@self.xdes_2-Ke+dF)
        for i in range(len(self.xdes)):
            if self.xdes[i]>0.01:
                self.xdes[i]=0.01
            elif self.xdes[i]<-0.01:
                self.xdes[i]=-0.01
        self.xdes_2=self.xdes_1
        self.xdes_1=self.xdes