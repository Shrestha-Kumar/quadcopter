"""Independent Python (numpy) cross-check of matlab/sim_quad.m for the cascaded PID design.
Used to verify the design on a machine without MATLAB. Mirrors params.m, quad_dynamics.m,
design_gains.m (PID) and the PID branch of sim_quad.m. Run: python3 tools/check_sim.py
"""
import numpy as np

g=9.81; m=1.10; L=0.225; d=L/np.sqrt(2)
J=np.diag([0.011,0.011,0.021]); Tmax=8.0; Tidle=0.05*Tmax; kq=0.016; tau_m=0.05; cd=0.10
Ts=1/200; hover=m*g
tau_max=np.array([4*d*Tmax/2]*2+[kq*2*Tmax])
wr, wy, Td = 15., 8., 0.02
w=np.array([wr,wr,wy]); Jd=np.diag(J)
Kang=w/4; Kp=Jd*w; Ki=Kp*w/5; Kd=Kp*Td; Ilim=0.5*tau_max
alt=dict(kp=12.,ki=6.,kd=11.,ilim=0.35*hover)

def dyn(x,Tc,dist,Jt,mt,taum):
    phi,th,psi=x[6:9]; pqr=x[9:12]; T=x[12:16]
    c,s=np.cos,np.sin
    R=np.array([[c(th)*c(psi), s(phi)*s(th)*c(psi)-c(phi)*s(psi), c(phi)*s(th)*c(psi)+s(phi)*s(psi)],
                [c(th)*s(psi), s(phi)*s(th)*s(psi)+c(phi)*c(psi), c(phi)*s(th)*s(psi)-s(phi)*c(psi)],
                [-s(th), s(phi)*c(th), c(phi)*c(th)]])
    tau=np.array([d*(T[1]+T[2]-T[0]-T[3]), d*(T[0]+T[1]-T[2]-T[3]), kq*(T[0]+T[2]-T[1]-T[3])])+dist[3:]
    acc=np.array([0,0,g])+R@np.array([0,0,-T.sum()])/mt-(cd/mt)*x[3:6]+dist[:3]/mt
    ed=np.array([pqr[0]+(pqr[1]*s(phi)+pqr[2]*c(phi))*np.tan(th), pqr[1]*c(phi)-pqr[2]*s(phi),
                 (pqr[1]*s(phi)+pqr[2]*c(phi))/c(th)])
    pd=np.linalg.solve(Jt,tau-np.cross(pqr,Jt@pqr))
    return np.concatenate([x[3:6],acc,ed,pd,(Tc-T)/taum])

def sim(t_end=8, roll_ref=lambda t:np.deg2rad(10)*(t>=1), yaw_ref=lambda t:np.deg2rad(30)*(t>=3),
        alt_ref=lambda t:0.5*(t>=1), dist=lambda t:np.zeros(6), noise=False, mS=1,JS=1,kS=1,tS=1):
    rng=np.random.default_rng(1); dt=1e-3; N=int(t_end/dt); nc=int(Ts/dt)
    x=np.zeros(16); x[12:]=hover/4; Tcmd=x[12:].copy()
    I=np.zeros(3); pm=np.zeros(3); df=np.zeros(3); Ia=0; pa=0; da=0
    log=[]
    for k in range(N):
        t=k*dt
        if k%nc==0:
            e=x[6:9].copy(); r=x[9:12].copy(); a=-x[2]
            if noise:
                e+=np.deg2rad(.15)*rng.standard_normal(3); r+=np.deg2rad(.4)*rng.standard_normal(3); a+=.02*rng.standard_normal()
            ref=np.array([roll_ref(t),0,yaw_ref(t)]); ea=ref-e; ea[2]=np.arctan2(np.sin(ea[2]),np.cos(ea[2]))
            rs=Kang*ea; rs[:2]=np.clip(rs[:2],-np.deg2rad(150),np.deg2rad(150)); rs[2]=np.clip(rs[2],-np.deg2rad(90),np.deg2rad(90))
            er=rs-r; I=np.clip(I+Ki*er*Ts,-Ilim,Ilim)
            df+=0.2*((-(r-pm)/Ts)-df); pm=r
            tc=np.clip(Kp*er+I+Kd*df,-tau_max,tau_max)
            ealt=alt_ref(t)-a
            if abs(ealt)<0.25 or Ia*ealt<0:
                Ia=np.clip(Ia+alt['ki']*ealt*Ts,-alt['ilim'],alt['ilim'])
            da+=0.05*((-(a-pa)/Ts)-da); pa=a
            Tt=(hover+alt['kp']*ealt+Ia+alt['kd']*da)/max(np.cos(e[0])*np.cos(e[1]),.5)
            tp,tt,ty=tc
            Tcmd=np.clip(np.array([Tt/4-tp/(4*d)+tt/(4*d)+ty/(4*kq), Tt/4+tp/(4*d)+tt/(4*d)-ty/(4*kq),
                                   Tt/4+tp/(4*d)-tt/(4*d)+ty/(4*kq), Tt/4-tp/(4*d)-tt/(4*d)-ty/(4*kq)]),Tidle,Tmax)
        Tr=Tcmd*kS; dd=dist(t); Jt=J*JS; mt=m*mS; tm=tau_m*tS
        k1=dyn(x,Tr,dd,Jt,mt,tm);k2=dyn(x+dt/2*k1,Tr,dd,Jt,mt,tm);k3=dyn(x+dt/2*k2,Tr,dd,Jt,mt,tm);k4=dyn(x+dt*k3,Tr,dd,Jt,mt,tm)
        x=x+dt/6*(k1+2*k2+2*k3+k4)
        log.append([t,*np.rad2deg(x[6:9]),-x[2],*Tcmd])
    return np.array(log)

def metrics(t,y,r,t0,y0=0):
    A=r-y0; m=t>=t0; tt=t[m]-t0; yy=y[m]
    os=max(0,(np.max(np.sign(A)*(yy-r)))/abs(A)*100)
    idx=np.where(np.abs(yy-r)>0.02*abs(A))[0]; ts=tt[idx[-1]] if len(idx) else 0
    return os,ts,r-np.mean(yy[-500:])

if __name__=="__main__":
    for name,kw in [("nominal",{}),("noise",dict(noise=True)),("mass+20% J+20% kT-15%",dict(mS=1.2,JS=1.2,kS=.85)),
                    ("roll torque pulse",dict(dist=lambda t:np.array([0,0,0,.08*(4<=t<4.1),0,0])))]:
        o=sim(**kw); t=o[:,0]
        print(f"{name:26s} roll: OS={metrics(t,o[:,1],10,1)[0]:.1f}% ts={metrics(t,o[:,1],10,1)[1]:.2f}s sse={metrics(t,o[:,1],10,1)[2]:.2f}deg |"
              f" yaw: OS={metrics(t,o[:,3],30,3)[0]:.1f}% ts={metrics(t,o[:,3],30,3)[1]:.2f}s |"
              f" alt: OS={metrics(t,o[:,4],.5,1)[0]:.1f}% ts={metrics(t,o[:,4],.5,1)[1]:.2f}s | max pitch={np.max(np.abs(o[:,2])):.2f}deg Tcmd range [{o[:,5:].min():.2f},{o[:,5:].max():.2f}]N")
