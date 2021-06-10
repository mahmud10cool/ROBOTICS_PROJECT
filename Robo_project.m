%% Robotics project
clear;
clc;
 
% Specifying the link lengths of the robot
% 3 DOF with three revolute joints
L1 = Link('revolute', 'd', 0, 'a', 8, 'alpha', 0);
L2 = Link('revolute', 'd', 0, 'a', 6, 'alpha', 0);
L3 = Link('revolute', 'd', 0, 'a', 2, 'alpha', 0);
 
% Creating the robot model
IMS = SerialLink([L1 L2 L3]);
IMS.name = '1728961 ROBOT';
 
% Time interval and step size
% 5 steps with total time of 1s
t=0:0.2:1; 
 
%% S
% The three values in the array are the three joint angles
% i.e. Sn=[theta1 theta2 theta3]; where n is an integer
S0=[-0.1843 -1.9618 6.8585];
S1=[-0.1656 -1.9659 6.8439];
S2=[-0.2164 -2.0431 6.9719];
S3=[-0.2798 -2.0408 7.0330];
S4=[-0.2821 -1.9928 6.9872];
S5=[-0.2548 -1.9067 6.8739];
S6=[-0.2949 -1.8692 6.8765];
S7=[-0.3886 -1.9127 7.0137];
S8=[-0.3322 -1.9381 6.9826];
 
% Trajectory planning for the letter S
Straj1 = jtraj(S0,S1,t); 
Straj2 = jtraj(S1,S2,t); 
Straj3 = jtraj(S2,S3,t); 
Straj4 = jtraj(S3,S4,t); 
Straj5 = jtraj(S4,S5,t); 
Straj6 = jtraj(S5,S6,t); 
Straj7 = jtraj(S6,S7,t); 
Straj8 = jtraj(S7,S8,t); 
 
hold on
 
% Initialising the variable
Ibra=zeros(4,4);
 
% Specifying the workspace
xlim([-5,5])
ylim([-10,-8])
zlim([-1,5])
 
% The view is set to [180,90] to straighten the sketch
% No absolute starting position was chosen
% All movements are relative to the robot's base and understand-ing
% So the sketch will be done based on the robot's workspace and orientation
 
% Move from origin to starting point
IMS.plot(jtraj([0 0 0],S0,t),'scale',0.6,'linkcolor','r','jointcolor','k','view',[180,90])
 
% 1st segment
for i=1:1:length(t)
    Ibra=IMS.fkine(Straj1(i,:));
    S_letter(i,:)=transl(Ibra); 
    %  Extract the translation component of the pose 
    %  matrix (3Element column vector) stored in JTA 
    %  vector array
    jta=S_letter;
    % Draw track points
    plot2(jta(i,:),'r.')   
    % Trajectory animation
    IMS.plot(Straj1(i,:))
    % Draw a trajectory line
    plot2(S_letter,'b','linewidth',2)         
end
% 2nd segment
for i=1:1:length(t)
    Ibra2=IMS.fkine(Straj2(i,:));
    S_letter2(i,:)=transl(Ibra2);  
    jta2=S_letter2; 
    plot2(jta2(i,:),'r.') 
    IMS.plot(Straj2(i,:)) 
    plot2(S_letter2,'b','linewidth',2)         
end
% 3rd segment
for i=1:1:length(t)
    Ibra3=IMS.fkine(Straj3(i,:));
    S_letter3(i,:)=transl(Ibra3);  
    jta3=S_letter3; 
    plot2(jta3(i,:),'r.')   
    IMS.plot(Straj3(i,:)) 
    plot2(S_letter3,'b','linewidth',2)         
end
% 4th segment
for i=1:1:length(t)
    Ibra4=IMS.fkine(Straj4(i,:));
    S_letter4(i,:)=transl(Ibra4);  
    jta4=S_letter4; 
    plot2(jta4(i,:),'r.')   
    IMS.plot(Straj4(i,:)) 
    plot2(S_letter4,'b','linewidth',2)        
end
% 5th segment
for i=1:1:length(t)
    Ibra5=IMS.fkine(Straj5(i,:));
    S_letter5(i,:)=transl(Ibra5);  
    jta5=S_letter5; 
    plot2(jta5(i,:),'r.')  
    IMS.plot(Straj5(i,:)) 
    plot2(S_letter5,'b','linewidth',2)         
end
% 6th segment
for i=1:1:length(t)
    Ibra6=IMS.fkine(Straj6(i,:));
    S_letter6(i,:)=transl(Ibra6);  
    jta6=S_letter6; 
    plot2(jta6(i,:),'r.')   
    IMS.plot(Straj6(i,:)) 
    plot2(S_letter6,'b','linewidth',2)        
end
% 7th segment
for i=1:1:length(t)
    Ibra7=IMS.fkine(Straj7(i,:));
    S_letter7(i,:)=transl(Ibra7);  
    jta7=S_letter7; 
    plot2(jta7(i,:),'r.')   
    IMS.plot(Straj7(i,:)) 
    plot2(S_letter7,'b','linewidth',2)        
end
% 8th segment
for i=1:1:length(t)
    Ibra8=IMS.fkine(Straj8(i,:));
    S_letter8(i,:)=transl(Ibra8);  
    jta8=S_letter8; 
    plot2(jta8(i,:),'r.')   
    IMS.plot(Straj8(i,:))
    plot2(S_letter8,'b','linewidth',2)         
end
 
%% U
U0=[-0.3780 -1.9532 7.0436];
U1=[-0.4930 -1.9379 7.1433];
U2=[-0.3101 -2.0885 7.1110];
U3=[-0.3112 -2.1117 7.1352];
U4=[-0.5488 -1.9849 7.2461];
U5=[-0.3810 -2.1530 7.2464];
U6=[-0.3979 -2.1811 7.2914];
U7=[-0.4730 -2.1535 7.3389];
 
% Trajectory planning for the letter U
Utraj1 = jtraj(U0,U1,t); 
Utraj2 = jtraj(U1,U2,t); 
Utraj3 = jtraj(U2,U3,t); 
Utraj4 = jtraj(U3,U4,t); 
Utraj5 = jtraj(U4,U5,t); 
Utraj6 = jtraj(U5,U6,t); 
Utraj7 = jtraj(U6,U7,t); 
 
hold on
 
% 1st segment
for i=1:1:length(t)
    Ibra=IMS.fkine(Utraj1(i,:));
    U_letter(i,:)=transl(Ibra);  %#ok<*SAGROW>
    jta=U_letter; 
    plot2(jta(i,:),'y.')  
    IMS.plot(Utraj1(i,:)) 
    plot2(U_letter,'g','linewidth',2)         
end
% 2nd segment
for i=1:1:length(t)
    Ibra2=IMS.fkine(Utraj2(i,:));
    U_letter2(i,:)=transl(Ibra2);   
    jta2=U_letter2; 
    plot2(jta2(i,:),'y.')  
    IMS.plot(Utraj2(i,:)) 
    plot2(U_letter2,'g','linewidth',2)         
end
% 3rd segment
for i=1:1:length(t)
    Ibra3=IMS.fkine(Utraj3(i,:));
    U_letter3(i,:)=transl(Ibra3); 
    jta3=U_letter3; 
    plot2(jta3(i,:),'y.')   
    IMS.plot(Utraj3(i,:)) 
    plot2(U_letter3,'g','linewidth',2)         
end
% 4th segment
for i=1:1:length(t)
    Ibra4=IMS.fkine(Utraj4(i,:));
    U_letter4(i,:)=transl(Ibra4);   
    jta4=U_letter4; 
    plot2(jta4(i,:),'y.')   
    IMS.plot(Utraj4(i,:)) 
    plot2(U_letter4,'g','linewidth',2)         
end
% 5th segment
for i=1:1:length(t)
    Ibra5=IMS.fkine(Utraj5(i,:));
    U_letter5(i,:)=transl(Ibra5);  
    jta5=U_letter5; 
    plot2(jta5(i,:),'y.')   
    IMS.plot(Utraj5(i,:))
    plot2(U_letter5,'g','linewidth',2)         
end
% 6th segment
for i=1:1:length(t)
    Ibra6=IMS.fkine(Utraj6(i,:));
    U_letter6(i,:)=transl(Ibra6);  
    jta6=U_letter6; 
    plot2(jta6(i,:),'y.')   
    IMS.plot(Utraj6(i,:)) 
    plot2(U_letter6,'g','linewidth',2)        
end
% 7th segment
for i=1:1:length(t)
    Ibra7=IMS.fkine(Utraj7(i,:));
    U_letter7(i,:)=transl(Ibra7);  
    jta7=U_letter7; 
    plot2(jta7(i,:),'y.')   
    IMS.plot(Utraj7(i,:)) 
    plot2(U_letter7,'g','linewidth',2)        
end
 
%% H
H0=[-0.5140 -2.0319 7.2582];
H1=[-0.5947 -2.0005 7.3076];
H2=[-0.6735 -1.9994 7.3853];
H3=[-0.6676 -2.0356 7.4155];
H4=[-0.4648 -2.2074 7.3846];
H5=[-0.4933 -2.1416 7.3473];
H6=[-0.6496 -2.1590 7.5210];
H7=[-0.5834 -2.2284 7.5242];
H8=[-0.5767 -2.2006 7.4896];
H9=[-0.7307 -2.0245 7.4676];
 
% Trajectory planning for the letter H
Htraj1 = jtraj(H0,H1,t); 
Htraj2 = jtraj(H1,H2,t); 
Htraj3 = jtraj(H2,H3,t); 
Htraj4 = jtraj(H3,H4,t); 
% Htraj5 = jtraj(H4,H5,t); % Commented to show it is skipped 
Htraj6 = jtraj(H5,H6,t); 
% Htraj7 = jtraj(H6,H7,t); % Commented to show it is skipped
Htraj8 = jtraj(H7,H8,t);
Htraj9 = jtraj(H8,H9,t);
 
hold on
 
% 1st segment
for i=1:1:length(t)
    Ibra=IMS.fkine(Htraj1(i,:));
    H_letter(i,:)=transl(Ibra);  
    jta=H_letter;
    plot2(jta(i,:),'r.') 
    IMS.plot(Htraj1(i,:)) 
    plot2(H_letter,'b','linewidth',2) 
end
% 2nd segment
for i=1:1:length(t)
    Ibra2=IMS.fkine(Htraj2(i,:));
    H_letter2(i,:)=transl(Ibra2); 
    jta2=H_letter2;
    plot2(jta2(i,:),'r.')   
    IMS.plot(Htraj2(i,:)) 
    plot2(H_letter2,'b','linewidth',2)         
end
% 3rd segment
for i=1:1:length(t)
    Ibra3=IMS.fkine(Htraj3(i,:));
    H_letter3(i,:)=transl(Ibra3); 
    jta3=H_letter3;
    plot2(jta3(i,:),'r.')   
    IMS.plot(Htraj3(i,:)) 
    plot2(H_letter3,'b','linewidth',2)        
end
% 4th segment
for i=1:1:length(t)
    Ibra4=IMS.fkine(Htraj4(i,:));
    H_letter4(i,:)=transl(Ibra4);  
    jta4=H_letter4;
    plot2(jta4(i,:),'r.')   
    IMS.plot(Htraj4(i,:)) 
    plot2(H_letter4,'b','linewidth',2)         
end
% 5th segment
for i=1:1:length(t)
    Ibra6=IMS.fkine(Htraj6(i,:));
    H_letter6(i,:)=transl(Ibra6);  
    jta6=H_letter6;
    plot2(jta6(i,:),'r.')   
    IMS.plot(Htraj6(i,:))
    plot2(H_letter6,'b','linewidth',2)        
end
% 6th segment
for i=1:1:length(t)
    Ibra8=IMS.fkine(Htraj8(i,:));
    H_letter8(i,:)=transl(Ibra8);  
    jta8=H_letter8;
    plot2(jta8(i,:),'r.')  
    IMS.plot(Htraj8(i,:)) 
    plot2(H_letter8,'b','linewidth',2)         
end
% 7th segment
for i=1:1:length(t)
    Ibra9=IMS.fkine(Htraj9(i,:));
    H_letter9(i,:)=transl(Ibra9);
    jta9=H_letter9;
    plot2(jta9(i,:),'r.')   
    IMS.plot(Htraj9(i,:)) 
    plot2(H_letter9,'b','linewidth',2)   
end
 
%% A
A0=[-0.6381 -2.2620 7.6125];
A1=[-0.9330 -1.9948 7.6402];
A2=[-0.8000 -2.2479 7.7603];
A3=[-0.8655 -2.2079 7.7858];
A4=[-0.7599 -2.1223 7.5946];
A5=[-0.7184 -2.1473 7.5780];
A6=[-0.8862 -2.1380 7.7366];
 
% Trajectory planning for the letter A
Atraj1 = jtraj(A0,A1,t); 
Atraj2 = jtraj(A1,A2,t); 
Atraj3 = jtraj(A2,A3,t); 
% Atraj4 = jtraj(A3,A4,t); Commented to show it is skipped
Atraj5 = jtraj(A4,A5,t); 
Atraj6 = jtraj(A5,A6,t); 
 
hold on
 
% 1st segment
for i=1:1:length(t)
    Ibra=IMS.fkine(Atraj1(i,:));
    A_letter(i,:)=transl(Ibra); 
    jta=A_letter; 
    plot2(jta(i,:),'y.')  
    IMS.plot(Atraj1(i,:)) 
    plot2(A_letter,'g','linewidth',2)         
end
% 2nd segment
for i=1:1:length(t)
    Ibra2=IMS.fkine(Atraj2(i,:));
    A_letter2(i,:)=transl(Ibra2);   
    jta2=A_letter2; 
    plot2(jta2(i,:),'y.')  
    IMS.plot(Atraj2(i,:)) 
    plot2(A_letter2,'g','linewidth',2)         
end
% 3rd segment
for i=1:1:length(t)
    Ibra3=IMS.fkine(Atraj3(i,:));
    A_letter3(i,:)=transl(Ibra3); 
    jta3=A_letter3; 
    plot2(jta3(i,:),'y.')   
    IMS.plot(Atraj3(i,:)) 
    plot2(A_letter3,'g','linewidth',2)         
end
% 4th segment
for i=1:1:length(t)
    Ibra5=IMS.fkine(Atraj5(i,:));
    A_letter5(i,:)=transl(Ibra5);  
    jta5=A_letter5; 
    plot2(jta5(i,:),'y.')   
    IMS.plot(Atraj5(i,:))
    plot2(A_letter5,'g','linewidth',2)         
end
% 5th segment
for i=1:1:length(t)
    Ibra6=IMS.fkine(Atraj6(i,:));
    A_letter6(i,:)=transl(Ibra6);  
    jta6=A_letter6; 
    plot2(jta6(i,:),'y.')   
    IMS.plot(Atraj6(i,:)) 
    plot2(A_letter6,'g','linewidth',2)        
end
 
%% I
I0=[-0.9260 -2.0976 7.7360];
I1=[-0.9687 -2.0212 7.7023];
I2=[-1.0270 -1.9718 7.7112];
I3=[-1.0373 -2.0022 7.7518];
I4=[-0.9432 -2.2276 7.8833];
 
% Trajectory planning for the letter I
Itraj1 = jtraj(I0,I1,t); 
Itraj2 = jtraj(I1,I2,t); 
Itraj3 = jtraj(I2,I3,t); 
Itraj4 = jtraj(I3,I4,t); 
 
hold on
 
% 1st segment
for i=1:1:length(t)
    Ibra=IMS.fkine(Itraj1(i,:));
    I_letter(i,:)=transl(Ibra);  
    jta=I_letter;
    % Draw track points
    plot2(jta(i,:),'r.')   
    % Trajectory animation
    IMS.plot(Itraj1(i,:))
    % Draw a trajectory line
    plot2(I_letter,'b','linewidth',2)         
end
% 2nd segment
for i=1:1:length(t)
    Ibra2=IMS.fkine(Itraj2(i,:));
    I_letter2(i,:)=transl(Ibra2);  
    jta2=I_letter2; 
    plot2(jta2(i,:),'r.')  
    IMS.plot(Itraj2(i,:)) 
    plot2(I_letter2,'b','linewidth',2)         
end
% 3rd segment
for i=1:1:length(t)
    Ibra3=IMS.fkine(Itraj3(i,:));
    I_letter3(i,:)=transl(Ibra3);  
    jta3=I_letter3; 
    plot2(jta3(i,:),'r.')   
    IMS.plot(Itraj3(i,:)) 
    plot2(I_letter3,'b','linewidth',2)         
end
% 4th segment
for i=1:1:length(t)
    Ibra4=IMS.fkine(Itraj4(i,:));
    I_letter4(i,:)=transl(Ibra4);  
    jta4=I_letter4; 
    plot2(jta4(i,:),'r.')   
    IMS.plot(Itraj4(i,:)) 
    plot2(I_letter4,'b','linewidth',2)        
end
 
%% M
M0=[-1.0349 -2.0364 7.7836];
M1=[-1.1270 -1.9437 7.7831];
M2=[-1.1394 -1.9634 7.8151];
M3=[-1.0659 -2.1762 7.9544];
M4=[-1.1865 -1.9412 7.8401];
M5=[-1.2200 -1.9071 7.8394];
M6=[-1.1406 -2.1467 7.9997];
M7=[-1.3053 -1.8562 7.8739];
M8=[-1.2571 -1.9863 7.9557];
M9=[-1.2321 -2.0846 8.0291];
M10=[-1.2519 -2.1075 8.0718];
M11=[-1.3127 -2.0192 8.0443];
 
% Trajectory planning for the letter M
Mtraj1 = jtraj(M0,M1,t); 
Mtraj2 = jtraj(M1,M2,t); 
Mtraj3 = jtraj(M2,M3,t); 
Mtraj4 = jtraj(M3,M4,t); 
Mtraj5 = jtraj(M4,M5,t); 
Mtraj6 = jtraj(M5,M6,t); 
Mtraj7 = jtraj(M6,M7,t);
Mtraj8 = jtraj(M7,M8,t); 
Mtraj9 = jtraj(M8,M9,t); 
Mtraj10 = jtraj(M9,M10,t); 
Mtraj11 = jtraj(M10,M11,t);
 
hold on
 
% 1st segment
for i=1:1:length(t)
    Ibra=IMS.fkine(Mtraj1(i,:));
    M_letter(i,:)=transl(Ibra); 
    jta=M_letter; 
    plot2(jta(i,:),'y.')  
    IMS.plot(Mtraj1(i,:)) 
    plot2(M_letter,'g','linewidth',2)         
end
% 2nd segment
for i=1:1:length(t)
    Ibra2=IMS.fkine(Mtraj2(i,:));
    M_letter2(i,:)=transl(Ibra2);   
    jta2=M_letter2; 
    plot2(jta2(i,:),'y.')  
    IMS.plot(Mtraj2(i,:)) 
    plot2(M_letter2,'g','linewidth',2)         
end
% 3rd segment
for i=1:1:length(t)
    Ibra3=IMS.fkine(Mtraj3(i,:));
    M_letter3(i,:)=transl(Ibra3); 
    jta3=M_letter3; 
    plot2(jta3(i,:),'y.')   
    IMS.plot(Mtraj3(i,:)) 
    plot2(M_letter3,'g','linewidth',2)         
end
% 4th segment
for i=1:1:length(t)
    Ibra4=IMS.fkine(Mtraj4(i,:));
    M_letter4(i,:)=transl(Ibra4);   
    jta4=M_letter4; 
    plot2(jta4(i,:),'y.')   
    IMS.plot(Mtraj4(i,:)) 
    plot2(M_letter4,'g','linewidth',2)         
end
% 5th segment
for i=1:1:length(t)
    Ibra5=IMS.fkine(Mtraj5(i,:));
    M_letter5(i,:)=transl(Ibra5);  
    jta5=M_letter5; 
    plot2(jta5(i,:),'y.')   
    IMS.plot(Mtraj5(i,:))
    plot2(M_letter5,'g','linewidth',2)         
end
% 6th segment
for i=1:1:length(t)
    Ibra6=IMS.fkine(Mtraj6(i,:));
    M_letter6(i,:)=transl(Ibra6);  
    jta6=M_letter6; 
    plot2(jta6(i,:),'y.')   
    IMS.plot(Mtraj6(i,:)) 
    plot2(M_letter6,'g','linewidth',2)        
end
% 7th segment
for i=1:1:length(t)
    Ibra7=IMS.fkine(Mtraj7(i,:));
    M_letter7(i,:)=transl(Ibra7);  
    jta7=M_letter7; 
    plot2(jta7(i,:),'y.')   
    IMS.plot(Mtraj7(i,:)) 
    plot2(M_letter7,'g','linewidth',2)        
end
% 8th segment
for i=1:1:length(t)
    Ibra8=IMS.fkine(Mtraj8(i,:));
    M_letter8(i,:)=transl(Ibra8);  
    jta8=M_letter8; 
    plot2(jta8(i,:),'y.')   
    IMS.plot(Mtraj8(i,:)) 
    plot2(M_letter8,'g','linewidth',2)        
end
% 9th segment
for i=1:1:length(t)
    Ibra9=IMS.fkine(Mtraj9(i,:));
    M_letter9(i,:)=transl(Ibra9);  
    jta9=M_letter9; 
    plot2(jta9(i,:),'y.')   
    IMS.plot(Mtraj9(i,:)) 
    plot2(M_letter9,'g','linewidth',2)        
end
% 10th segment
for i=1:1:length(t)
    Ibra10=IMS.fkine(Mtraj10(i,:));
    M_letter10(i,:)=transl(Ibra10);  
    jta10=M_letter10; 
    plot2(jta10(i,:),'y.')   
    IMS.plot(Mtraj10(i,:)) 
    plot2(M_letter10,'g','linewidth',2)        
end
% 11th segment
for i=1:1:length(t)
    Ibra11=IMS.fkine(Mtraj11(i,:));
    M_letter11(i,:)=transl(Ibra11);  
    jta11=M_letter11; 
    plot2(jta11(i,:),'y.')   
    IMS.plot(Mtraj11(i,:)) 
    plot2(M_letter11,'g','linewidth',2)        
end
 
%% Ib
Ib0=[-1.3243 -1.9440 7.9807];
Ib1=[-1.3577 -1.8371 7.9072];
Ib2=[-1.4080 -1.7660 7.8864];
Ib3=[-1.4156 -1.7933 7.9213];
Ib4=[-1.3695 -2.0314 8.1133];
 
 
% Trajectory planning for the letter Ib
Ibtraj1 = jtraj(Ib0,Ib1,t); 
Ibtraj2 = jtraj(Ib1,Ib2,t); 
Ibtraj3 = jtraj(Ib2,Ib3,t); 
Ibtraj4 = jtraj(Ib3,Ib4,t); 
 
hold on
 
% 1st segment
for i=1:1:length(t)
    Ibra=IMS.fkine(Ibtraj1(i,:));
    Ib_letter(i,:)=transl(Ibra);               
    jta=Ib_letter;
    % Draw track points
    plot2(jta(i,:),'r.')   
    % Trajectory animation
    IMS.plot(Ibtraj1(i,:))
    % Draw a trajectory line
    plot2(Ib_letter,'b','linewidth',2)         
end
% 2nd segment
for i=1:1:length(t)
    Ibra2=IMS.fkine(Ibtraj2(i,:));
    Ib_letter2(i,:)=transl(Ibra2); 
    jta2=Ib_letter2; 
    plot2(jta2(i,:),'r.')   
    IMS.plot(Ibtraj2(i,:)) 
    plot2(Ib_letter2,'b','linewidth',2)        
end
% 3rd segment
for i=1:1:length(t)
    Ibra3=IMS.fkine(Ibtraj3(i,:));
    Ib_letter3(i,:)=transl(Ibra3);  
    jta3=Ib_letter3; 
    plot2(jta3(i,:),'r.')   
    IMS.plot(Ibtraj3(i,:)) 
    plot2(Ib_letter3,'b','linewidth',2)         
end
% 4th segment
for i=1:1:length(t)
    Ibra4=IMS.fkine(Ibtraj4(i,:));
    Ib_letter4(i,:)=transl(Ibra4);  
    jta4=Ib_letter4; 
    plot2(jta4(i,:),'r.')  
    IMS.plot(Ibtraj4(i,:)) 
    plot2(Ib_letter4,'b','linewidth',2)    
end
 
% Move back to origin
IMS.plot(jtraj(Ib4,[0 0 0],t))
