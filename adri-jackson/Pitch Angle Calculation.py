#!/usr/bin/env python
# coding: utf-8

# In[1]:


#Adri Jackson
import numpy as np
import matplotlib.pyplot as plt

#constants
u_0 = 1.2566*(10**(-6)) #permeability of free space
M = 8.22*(10**(22)) #magnetization of the Earth
r_e = 6378*(10**3) #radius of the Earth (in meters)

#scaled values taken from field vs. altitude graph, taking max as 
#4.5*10^4, rather than 2.5*10^4, in nanotesla
B_scale_400 = 4.5*(10**4)
B_scale_450 = 4.41*(10**4)
B_scale_500 = 4.32*(10**4)
B_scale_550 = 4.23*(10**4)
B_scale_840 = 3.69*(10**4)
#Values from the World Magnetic Model for December 2024, taken from NOAA
#And approximated from multiple values for longitude
B_NOAA_500 = 4.5*(10**4)
B_NOAA_840 = 4.0*(10**4)

#function to calculate B-field using the dipole equation, taking an altitude
#(in kilometers) as a parameter and returning the field value in nanotesla
def Calc_B_dip(alt):
    B = (u_0*M)/(2*np.pi*((r_e + alt*(10**3))**3))
    n_T = B*(10**9)
    #print('The field at',alt,'km is',n_T,'nT')
    return n_T

#function to calculate the pitch angle where the subscript 1 represents
#the known angle and the subscript 2 represents the target value. takes 
#field values at both altitudes, and the known angle (in degrees) as parameters,
#and returns the unknown pitch angle in degrees
def Calc_pitch_angle(B_1,B_2,a_1):
    alpha = np.arcsin(np.sin(a_1*(np.pi/180))*np.sqrt(B_2/B_1))
    #print("The pitch angle is",alpha*(180/np.pi),"degrees")
    degrees = alpha*(180/np.pi)
    return degrees

#if you want to recreate the initial calculations I did, uncomment the print statement
#in "Calc_B_dip" and "Calc_pitch_angle" as well as everything below. if you choose to run 
#the code to create the graphs, comment out the print statement in the functions to avoid 
#unnecessary output
"""
print("The pitch angles calculated using the dipole values:")
Calc_pitch_angle(Calc_B_dip(450),Calc_B_dip(550),40)
Calc_pitch_angle(Calc_B_dip(450),Calc_B_dip(550),12)
print()
print("The pitch angles calculated using the paper values:")
Calc_pitch_angle(B_scale_450,B_scale_550,40)
Calc_pitch_angle(B_scale_450,B_scale_550,12)
print()
print("Using the Dipole equation, the B-Field values (in nT) are:")
#print(Calc_B_dip(400))
print(Calc_B_dip(450), "for 450km")
#print(Calc_B_dip(500))
print(Calc_B_dip(550), "for 550km")
print()
print("From the paper, the B-field values (in nT) are")
print(B_scale_450, "for 450km")
print(B_scale_550, "for 550km")
"""
print()


# In[2]:


angles = []
i = 0
for i in range(0,91):
    angles.append(i)
    i += 1
alpha = []
for i in range(len(angles)):
    n = Calc_pitch_angle(Calc_B_dip(500),Calc_B_dip(840),i)
    alpha.append(n)
    i += 1
#print(alpha)    
plt.plot(angles,alpha)
plt.xlabel('"Known" Angle (degrees)')
plt.ylabel('Calculated Angle (degrees)')
plt.title('Pitch Angle Distribution For Simple Dipole Equation')
plt.xlim(0,90)
plt.ylim(0,90)
plt.grid()
plt.show


# In[3]:


'''
alpha = []
for i in range(len(angles)):
    n = Calc_pitch_angle(Calc_B_dip(500),Calc_B_dip(500),i)
    alpha.append(n)
    i += 1
#print(alpha)    
plt.plot(angles,alpha)
plt.xlabel('"Known" Angle (degrees)')
plt.ylabel('Calculated Angle (degrees)')
plt.title('Pitch Angle Distribution For Constant Field')
plt.xlim(0,90)
plt.ylim(0,90)
plt.grid()
plt.show
'''
print()


# In[4]:


alpha = []
for i in range(len(angles)):
    n = Calc_pitch_angle(B_scale_500,B_scale_840,i)
    alpha.append(n)
    i += 1
#print(alpha)    
plt.plot(angles,alpha)
plt.xlabel('"Known" Angle (degrees)')
plt.ylabel('Calculated Angle (degrees)')
plt.title('Pitch Angle Distribution For Interpreted Research Values')
plt.xlim(0,90)
plt.ylim(0,90)
plt.grid()
plt.show


# In[5]:


alpha = []
for i in range(len(angles)):
    n = Calc_pitch_angle(B_NOAA_500,B_NOAA_840,i)
    alpha.append(n)
    i += 1
#print(alpha)    
plt.plot(angles,alpha)
plt.xlabel('"Known" Angle (degrees)')
plt.ylabel('Calculated Angle (degrees)')
plt.title('Pitch Angle Distribution From NOAA WMM December 2024 values')
plt.xlim(0,90)
plt.ylim(0,90)
plt.grid()
plt.show


# In[6]:


B_NOAA_IGRF_840 = 4.0332*(10**4)
B_NOAA_IGRF_500 = 4.6272*(10**4)
alpha = []
for i in range(len(angles)):
    n = Calc_pitch_angle(B_NOAA_IGRF_500,B_NOAA_IGRF_840,i)
    alpha.append(n)
    i += 1
#print(alpha)    
plt.plot(angles,alpha)
plt.xlabel('"Known" Angle (degrees)')
plt.ylabel('Calculated Angle (degrees)')
plt.title('Pitch Angle Distribution From NOAA IGRF December 2024 values')
plt.xlim(0,90)
plt.ylim(0,90)
plt.grid()
plt.show

