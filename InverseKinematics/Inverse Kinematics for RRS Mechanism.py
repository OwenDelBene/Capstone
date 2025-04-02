import numpy as np

# Given from IMU Readings
Roll = np.deg2rad(-2)
Pitch = np.deg2rad(7)
#Roll = np.deg2rad(-1.7298419)
#Pitch = np.deg2rad(5.6588034)
#Pitch = np.deg2rad(2.412186235)
#Roll = np.deg2rad(-1.390335138)

R1 = np.array([[1, 0, 0],
               [0, np.cos(Roll), -1*np.sin(Roll)],
               [0, np.sin(Roll), np.cos(Roll)]])
R2 = np.array([[np.cos(Pitch), 0, np.sin(Pitch)],
               [0, 1, 0],
               [-1*np.sin(Pitch), 0, np.cos(Pitch)]])

R3 = np.array([[1, 0, 0],
               [0, 1, 0],
               [0, 0, 1]])

Rop = R3@R2@R1
rp = np.array([[0],[0],[112.28]]) # Units in mm

def Tai(angle):
    '''tai = np.array([[np.cos(angle), 0, np.sin(angle)],
                    [0, 1, 0],
                    [-np.sin(angle), 0, np.cos(angle)]])'''
    tai = np.array([[np.cos(angle), -1*np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0, 0, 1]])
    '''tai = np.array([[1, 0, 0],
                    [0, np.cos(angle), -1*np.sin(angle)],
                    [0, np.sin(angle), np.cos(angle)]])'''
    return tai

def Tbi(angle):
    '''tbi = np.array([[np.cos(angle), 0, np.sin(angle)],
                    [0, 1, 0],
                    [-np.sin(angle), 0, np.cos(angle)]])'''
    tbi = np.array([[np.cos(angle), -1*np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0, 0, 1]])
    '''tbi = np.array([[1, 0, 0],
                    [0, np.cos(angle), -1*np.sin(angle)],
                    [0, np.sin(angle), np.cos(angle)]])'''
    return tbi

#ra_nX = np.array([[294.74, 0, 0]]).T @ Tai((4*np.pi)/3) 
#ra_pX = np.array([[294.74, 0, 0]]).T @ Tai((2*np.pi)/3) 
ra_nY = Tai((4*np.pi)/3) @ np.array([[294.74], [0], [0]]) 
ra_pY = Tai((2*np.pi)/3) @ np.array([[294.74], [0], [0]])
ra_X  = np.array([[294.74], [0], [0]])

#rb_nX = np.array([[224.4, 0, 0]]).T @ Tbi((4*np.pi)/3) 
#rb_pX = np.array([[224.4, 0, 0]]).T @ Tbi((2*np.pi)/3) 
rb_nY = Tbi((4*np.pi)/3) @ np.array([[224.4], [0], [0]])  
rb_pY = Tbi((2*np.pi)/3) @ np.array([[224.4], [0], [0]]) 
rb_X  = np.array([[224.4], [0], [0]])

def L_bai(rpp,rop,ra,rb):
    return (rpp + (rop@ra)) - rb

L_banY = L_bai(rp, Rop, ra_nY, rb_nY)
L_bapY = L_bai(rp, Rop, ra_pY, rb_pY)
L_baX  = L_bai(rp, Rop, ra_X, rb_X)

L_bc = 48.26 # mm
L_ca = 114.43 # mm

def ypbi(t):
    T = np.array([[np.cos(t), -1*np.sin(t), 0],
                  [np.sin(t), np.cos(t), 0],
                  [0, 0, 1]])
    '''T = np.array([[np.cos(t), 0, np.sin(t)],
                  [0, 1, 0],
                  [-1*np.sin(t), 0, np.cos(t)]])'''
    YPBI = T @ np.array([[-1], [0], [0]]) 
    return YPBI

yp_bnY = ypbi((4*np.pi)/3)
yp_bpY = ypbi((2*np.pi)/3)
#yp_bnX = np.array([0, 1, 0])
#yp_bpX = np.array([0, 1, 0])
yp_bX = np.array([-1, 0, 0])

def alpha(lbc,lca,lbai):
    mag = np.linalg.norm(lbai)
    A = np.arccos((lbc**2 + mag**2 - lca**2)/(2*lbc*mag))
    return A

def beta(lbai,yp):
    #print(np.dot(lbai,yp))
    B = np.arccos(np.dot(lbai.flatten(),yp.flatten())/np.linalg.norm(lbai))
    return B

#print(L_banX)
#print(yp_bnX)
#print(L_banX)
#print(yp_bnX)
#print(np.dot(L_banX.flatten(), yp_bnX))

alpha_ny = alpha(L_bc,L_ca,L_banY)
alpha_py = alpha(L_bc,L_ca,L_bapY)
alpha_x  = alpha(L_bc,L_ca,L_baX)

beta_ny = beta(L_banY,yp_bnY)
beta_py = beta(L_bapY,yp_bpY)
beta_x  = beta(L_baX,yp_bX)

q_nY = np.rad2deg(alpha_ny + beta_ny - (np.pi/2))
q_pY = np.rad2deg(alpha_py + beta_py - (np.pi/2))
q_X  = np.rad2deg(alpha_x + beta_x - (np.pi/2))


#print(alpha_Z)
#print(beta_Z)
print('The Inverse Kinematic Base Motor Angles Are:')
print('-Y =', 90 - q_nY, 'deg')
print('+Y =', 90 - q_pY, 'deg')
print(' X =', 90 - q_X, 'deg')
