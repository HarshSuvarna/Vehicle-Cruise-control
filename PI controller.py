import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint


def vehicle_model(v,t,u,load):  #v=velocity(m/s), t=time(s),u=gas_pedal (-150% to 100%)

    Cd = 0.24         #drag coefficient
    rho = 1.225       #air density (kg/m^3)
    A = 5.0           #Cross Section area (m^2)
    Fp = 30           #thrust parameter (N/%pedal)
    m = 500           #vehicle mass (kg)

    #compute the change in state
    dv_dt = (1.0/(m+load)) * (Fp*u - 0.5*rho*Cd*A*v**2)

    return dv_dt

#time simulation
tf = 300
nsteps = 301
delta_t = tf/(nsteps-1)
ts = np.linspace(0,tf,nsteps)




#Initial Conditions
load = 200.0 #kg
v0 =0.0
sum_int= 0
ubias = 0


sp=25 #set point

#Proportional Integral Tuning Parameters 
Kc = (1.0/1.2)  * 5
tauI = 30.0


#Storing vlaues for plotting
gas_store = np.zeros(nsteps)
vs = np.zeros(nsteps)
sp_store=np.zeros(nsteps)
es = np.zeros(nsteps)
ies = np.zeros(nsteps)


#
for i in range(nsteps-1):
    #Changing the setvalue of velocity with time 
    if i == 50:
        sp= 0
    if i == 100:
        sp= 15
    if i == 150:
        sp= 20
    if i == 200:
        sp= 10

    
    sp_store[i+1] = sp    #store value of setpoint for plotting

    error = sp - v0       #calculating error
    es[i+1] = error       #store error values for plotting

    sum_int = sum_int + error*delta_t   #Integrating error

    gas_pedal = ubias + Kc*error + Kc/tauI * sum_int   #gas pedal value using PI controller
    
    #taking care of integral windup and physical constraints of gas pedal
    if gas_pedal>=100:          
        gas_pedal=100
        sum_int = sum_int - error *delta_t
    if gas_pedal<=-50:
        gas_pedal= -50
        sum_int = sum_int - error * delta_t

    gas_store[i+1] = gas_pedal  #storing for plotting
    ies[i+1] = sum_int          #storing integral of error for plotting

    v = odeint(vehicle_model,v0,[0,delta_t],args=(gas_pedal,load))  #getting new values of velocity
    v0 = v[-1]                  #updating value of v0 with new velocity
  
    vs[i+1] = v0                #storing for plotting


#data visualization and plotting

plt.figure()
plt.subplot(2,2,1)
plt.plot(ts,vs,'b-',linewidth=2,label = 'Velocity')
plt.plot(ts,sp_store,'k--',linewidth=2,label='Setpoint')
plt.ylabel('velocity m/s')
plt.legend()
plt.subplot(2,2,2)
plt.plot(ts,gas_store,'r--',linewidth=2,label = 'Gas Pedal')
plt.ylabel('Gas Pedal')
plt.legend()
plt.subplot(2,2,3)
plt.plot(ts,es,'b-',linewidth=2,label = 'Error')
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.legend()
plt.subplot(2,2,4)
plt.plot(ts,ies,'k--',linewidth=2,label = 'Integral error')
plt.ylabel('Integral error')
plt.xlabel('Time (s)')
plt.legend()
plt.show()




    
    
