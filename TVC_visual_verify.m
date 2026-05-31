%% ========================================================================
%  TVC VISUAL VERIFY -- drop your Control System Designer gains in and fly
%
%  Plays the role of your assignment's Nonlinear_simulation.slx + visualiser:
%  a nonlinear 3D free-flight run that checks the gains you tuned on the
%  linear plant actually work (with saturation, servo lag, sensor noise).
%
%  Inner loop  = PID (matches your "PID with two real zeros")
%  Outer loop  = PD  (matches your outer position compensator)
%  Runs with BASE MATLAB -- no toolboxes required.
% =========================================================================
clear; clc; close all;

%% ====================  PASTE YOUR TUNED GAINS HERE  =====================
% From the inner design:  Cpid = pid(C_inner);  then read .Kp .Ki .Kd
Kp_att = 1.6;   Ki_att = 0.5;   Kd_att = 0.25;     % inner attitude PID
% From the outer design:  [Kp_pos,~,Kd_pos] = piddata(pid(C_outer));
Kp_pos = 0.40;  Kd_pos = 0.33;                     % outer position PD
% =========================================================================

%% ---------- Physical parameters (match tvc_setup.m) --------------------
m=0.5; g=9.81; J=0.008; l=0.10; b_rot=0.002; Tmax=2.5*m*g;
tau_s=0.05; dmax=deg2rad(15); ratemax=deg2rad(600);
tilt_max=deg2rad(20);

%% ---------- Timing & sensors --------------------------------------------
fs=400; dt=1/fs; Tend=8; N=round(Tend/dt); animStep=8;
gyro_noise=deg2rad(1.0); gyro_bias=deg2rad(0.4);
acc_noise=deg2rad(2.0); alpha=0.99; pos_noise=0.01;
iclamp=deg2rad(10);                 % integral anti-windup clamp

%% ---------- Altitude loop gains -----------------------------------------
Kp_z=4.5; Kd_z=2.4; z_des=1.0; x_des=0; y_des=0;

%% ---------- Initial state (off-centre, low) -----------------------------
x=0.5; y=-0.4; z=0.6; vx=0; vy=0; vz=0;
thx=deg2rad(3); thy=deg2rad(-2); wx=0; wy=0; dx=0; dy=0;
thx_est=thx; thy_est=thy; eix=0; eiy=0;
t_poke=4.0; poke_vx=0.8; poke_vy=-0.6;

%% ---------- Logs ---------------------------------------------------------
L.t=zeros(1,N); L.x=L.t; L.y=L.t; L.z=L.t; L.thx=L.t; L.thy=L.t;

%% ---------- Figure -------------------------------------------------------
fig=figure('Color','w','Name','TVC Visual Verify','Position',[80 80 1100 480]);
ax1=subplot(1,2,1); hold(ax1,'on'); grid(ax1,'on'); view(ax1,35,18);
xlim([-1 1]); ylim([-1 1]); zlim([0 1.6]); daspect([1 1 1]);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]'); title('Free-flight (your gains)');
patch([-1 1 1 -1],[-1 -1 1 1],[0 0 0 0],[0.9 0.9 0.9],'FaceAlpha',0.4,'EdgeColor','none');
plot3([0 0],[0 0],[0 1.4],'k:'); plot3(0,0,z_des,'kp','MarkerFaceColor','y','MarkerSize',14);
bodyH=plot3([0 0],[0 0],[0 0],'b-','LineWidth',5);
thrustH=quiver3(0,0,0,0,0,0.2,0,'r','LineWidth',2,'MaxHeadSize',2);
pathH=plot3(nan,nan,nan,'Color',[0.4 0.4 0.4]);
ax2=subplot(1,2,2); hold(ax2,'on'); grid(ax2,'on');
xTr=animatedline('Color','b'); yTr=animatedline('Color','r'); zTr=animatedline('Color',[0 0.6 0]);
yline(0,'k:'); xlabel('time [s]'); ylabel('position [m]'); legend('x','y','z');
title('Position vs time'); xlim([0 Tend]); ylim([-0.6 1.4]);

%% ---------- Main loop ----------------------------------------------------
for k=1:N
    t=(k-1)*dt;
    % sensors + complementary filter
    gx=wx+gyro_bias+gyro_noise*randn; gy=wy+gyro_bias+gyro_noise*randn;
    thx_est=alpha*(thx_est+gx*dt)+(1-alpha)*(thx+acc_noise*randn);
    thy_est=alpha*(thy_est+gy*dt)+(1-alpha)*(thy+acc_noise*randn);
    xm=x+pos_noise*randn; ym=y+pos_noise*randn;

    % altitude -> thrust
    T=max(min(m*g + Kp_z*(z_des-z) + Kd_z*(0-vz), Tmax),0);

    % OUTER position PD -> desired tilt
    thx_des=max(min(-(Kp_pos*(xm-x_des)+Kd_pos*vx), tilt_max),-tilt_max);
    thy_des=max(min(-(Kp_pos*(ym-y_des)+Kd_pos*vy), tilt_max),-tilt_max);

    % INNER attitude PID -> gimbal command
    ex=thx_des-thx_est; ey=thy_des-thy_est;
    eix=max(min(eix+ex*dt,iclamp),-iclamp); eiy=max(min(eiy+ey*dt,iclamp),-iclamp);
    ux=max(min(Kp_att*ex+Ki_att*eix-Kd_att*gx, dmax),-dmax);
    uy=max(min(Kp_att*ey+Ki_att*eiy-Kd_att*gy, dmax),-dmax);

    % servos (rate limit + lag)
    rx=max(min((ux-dx)/tau_s,ratemax),-ratemax); dx=max(min(dx+rx*dt,dmax),-dmax);
    ry=max(min((uy-dy)/tau_s,ratemax),-ratemax); dy=max(min(dy+ry*dt,dmax),-dmax);

    % dynamics (attitude = double integrator + gimbal torque; no gravity term)
    wx=wx+((T*l*sin(dx)-b_rot*wx)/J)*dt; thx=thx+wx*dt;
    wy=wy+((T*l*sin(dy)-b_rot*wy)/J)*dt; thy=thy+wy*dt;
    vx=vx+(T/m*sin(thx+dx))*dt; x=x+vx*dt;
    vy=vy+(T/m*sin(thy+dy))*dt; y=y+vy*dt;
    vz=vz+(T/m*cos(thx)*cos(thy)-g)*dt; z=z+vz*dt;
    if abs(t-t_poke)<dt/2, vx=vx+poke_vx; vy=vy+poke_vy; end

    L.t(k)=t; L.x(k)=x; L.y(k)=y; L.z(k)=z; L.thx(k)=thx; L.thy(k)=thy;

    if mod(k,animStep)==0
        if ~isvalid(fig), break; end
        u=[sin(thx);sin(thy);cos(thx)*cos(thy)]; u=u/norm(u);
        p=[x;y;z]; base=p-u*0.06; top=p+u*0.12;
        set(bodyH,'XData',[base(1) top(1)],'YData',[base(2) top(2)],'ZData',[base(3) top(3)]);
        set(thrustH,'XData',base(1),'YData',base(2),'ZData',base(3), ...
            'UData',0.18*u(1),'VData',0.18*u(2),'WData',0.18*u(3));
        set(pathH,'XData',L.x(1:k),'YData',L.y(1:k),'ZData',L.z(1:k));
        addpoints(xTr,t,x); addpoints(yTr,t,y); addpoints(zTr,t,z);
        drawnow limitrate; pause(dt*animStep);
    end
end

%% ---------- Summary ------------------------------------------------------
figure('Color','w','Name','Verify summary');
subplot(2,1,1); plot(L.t,L.x,'b',L.t,L.y,'r',L.t,L.z,'g','LineWidth',1.2); grid on
ylabel('position [m]'); legend('x','y','z'); title('Returns to centre & holds altitude');
subplot(2,1,2); plot(L.t,rad2deg(L.thx),'b',L.t,rad2deg(L.thy),'r','LineWidth',1.2); grid on
ylabel('tilt [deg]'); xlabel('time [s]'); legend('pitch','roll');