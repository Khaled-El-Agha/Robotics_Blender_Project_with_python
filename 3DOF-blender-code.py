import numpy as np
import math
import bpy

#pi=math.pi


import time

from sympy import sin, cos, pi

from sympy import *

L1= symbols('L1') # robot link1 length
L2= symbols('L2') # robot link2 length
L3= symbols('L3') # robot link3 length
import mathutils
#pi=math.pi


### Your FK code here
# Create symbols
d1, d2, d3 = symbols('d1:4') # link offset
a1, a2, a3 = symbols('a1:4') # link length
alpha1,alpha2,alpha3 = symbols('alpha1:4')# twist angle
q1, q2, q3 = symbols('q1:4') # joint angle


# Create Modified DH parameters
DH_Table = {
            alpha1: pi/2, a1:0, d1:    L1, q1: q1,
            alpha2: 0, a2:L2, d2:    0, q2: q2,
            alpha3: 0, a3: L3, d3:  0, q3: q3,
}

def TF_Matrix(alpha,a,d,q):
    TF = Matrix([
                [cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q)],
                [sin(q),cos(q)*cos(alpha),-cos(q)*sin(alpha), a*sin(q)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0,1]])
    return TF

T0_1 = TF_Matrix(alpha1, a1,d1,q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha2, a2,d2,q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha3, a3,d3,q3).subs(DH_Table)

T0_3=simplify(T0_1*T1_2*T2_3)

EEFx= symbols('EEFx') # robot link1 length
EEFy= symbols('EEFy') # robot link1 length
EEFz= symbols('EEFy') # robot link1 length

EEFx=T0_3[0,3]
EEFy=T0_3[1,3]
EEFz=T0_3[2,3]

Jac00=diff(EEFx,q1)
Jac01=diff(EEFx,q2)
Jac02=diff(EEFx,q3)
Jac10=diff(EEFy,q1)
Jac11=diff(EEFy,q2)
Jac12=diff(EEFy,q3)
Jac20=diff(EEFz,q1)
Jac21=diff(EEFz,q2)
Jac22=diff(EEFz,q3)


Jac=Matrix([[Jac00, Jac01, Jac02],[Jac10, Jac11,  Jac12],[Jac20, Jac21,  Jac22]])
simplify(Jac)



def JacInv(Jac):
    return simplify(Jac.inv())
#JacInv(Jac)
#def rotate_bone(angle):

jacobian_inverse=JacInv(Jac)

tnew= np.linspace(0, 3, num=101, endpoint=True)

eefxr=[1,1.05,1.4,1.41]

eefyr=[0,0.05,0.7,0.71]

eefzr=[2,2.05,2.2,2.21]

t=[0,0.5,2.5,3]
#f2 = interpolate.interp1d(t, eefxr, kind='cubic')

zx = np.polyfit(t, eefxr, 4)
fx = np.poly1d(zx)

zy = np.polyfit(t, eefyr, 4)
fy = np.poly1d(zy)

zz = np.polyfit(t, eefzr, 4)
fz = np.poly1d(zz)



import numpy as np
delta_effx=np.diff(fx(tnew),axis=0)
delta_effy=np.diff(fy(tnew),axis=0)
delta_effz=np.diff(fz(tnew),axis=0)
# inverse kinematics loop
x_val=1
y_val=0
z_val=2

q1_val = 0
q2_val=pi/2
q3_val=-pi/2

estimated_eefx=[]
estimated_eefy=[]
for ite in range(len(tnew)-1):
    print(ite)
    jac_inv_value=jacobian_inverse.evalf(subs={q1: q1_val,q2:q2_val,q3: q3_val,L1:1,L2:1,L3:1})
    print(q2_val)
    delta_q=jac_inv_value.dot(np.asarray([delta_effx[ite],delta_effy[ite],delta_effz[ite]]))
    q1_val=q1_val+delta_q[0]
    q2_val=q2_val+delta_q[1]
    q3_val=q3_val+delta_q[2]
    ob = bpy.data.objects['Armature']
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.mode_set(mode='POSE')

    pbone1 = ob.pose.bones['Bone']
    pbone2 = ob.pose.bones['Bone.001']
    pbone3 = ob.pose.bones['Bone.002']
    # select axis in ['X','Y','Z']  <--bone local
    ##pbone1.rotation_euler=mathutils.Euler((0,0,-(pi/2- q1_val)),'XYZ')
    
    pbone1.rotation_euler[1] =     q1_val
    pbone1.keyframe_insert(data_path="rotation_euler" ,frame=ite*10)
    
    pbone2.rotation_euler[2] =    -(pi/2 - q2_val)
    pbone2.keyframe_insert(data_path="rotation_euler" ,frame=ite*10)
    
    pbone3.rotation_euler[2] =      q3_val
    pbone3.keyframe_insert(data_path="rotation_euler" ,frame=ite*10)
    
    #pbone2.rotation_euler=mathutils.Euler((0,0,q2_val),'XYZ')
    
    
    #pbone2.rotation_mode = 'XYZ'
            # select axis in ['X','Y','Z']  <--bone local
    #pbone2.rotation_euler.rotate_axis(axis, q2_val/10)
    bpy.ops.object.mode_set(mode='OBJECT')
            #insert a keyframe
    #pbone2.keyframe_insert(data_path="rotation_euler" ,frame=ite*10)

