% Dallin and Bryan
%ME 537 Robotics Project

% inverse kinematics


clear
clc
%1
% Some of the code below is repeated for the two different methods shown
% (things like calculating pose error).
% This is not good coding practice and instead, repeated code should be
% changed to functions that can be called from anywhere.

%clearing workspace of variables
clear all;
close all;
clc;

%using custom defined robot from homework 2
Baxter;
% required joint angles to start from
qi1 = zeros(7,1);
qi2 = pi/2*ones(7,1);

% gain for damped pseudo-inverse
K_inv = 0.3*eye(6);

%for j_transpose
K_trans = 0.5*eye(3);

%place to store solution joint angles for two different methods
q_slns_pinv = [];
status_pinv = [];
q_slns_trans = [];
status_trans = [];
num_trials = 5;
% q_rand = [];
% for i = 1:num_trials
%     for j = 1:7
%         a = robot.qlim(j,1);
%         b = robot.qlim(j,2);
%         r = (b-a).*rand(1,1) + a;
%         q_rand(j,i) = [r];
%     end
conv = 0.0254;
T_des1 = ([-1 0 0 0*conv; 0 1 0 -18*conv; 0 0 -1 -18*conv+.08; 0 0 0 1]);
T_des2 = ([-1 0 0 -2*conv; 0 1 0 -18*conv; 0 0 -1 -18*conv; 0 0 0 1]); %pick up
T_des3 = ([-1 0 0 15*conv; 0 1 0 -10*conv; 0 0 -1 8*conv; 0 0 0 1]);
T_des4 = ([-1 0 0 30*conv; 0 1 0 2*conv; 0 0 -1 -5*conv; 0 0 0 1]); %drop off
T_des5 = ([-1 0 0 28*conv; 0 1 0 -7*conv; 0 0 -1 -5*conv+0.06; 0 0 0 1]);
T_desired(:,:,1) = T_des1;
T_desired(:,:,2) = T_des2;
T_desired(:,:,3) = T_des3;
T_desired(:,:,4) = T_des4;
T_desired(:,:,5) = T_des5;
i = 1;
% while i < (num_trials+1)
% T_desired(:,:,i) = q_rand(:,i);
% i=i+1;
% end

%start with pseudo-inverse method
qi = qi1; % if want to run from all joint angles being pi/2, just update this.
q = qi;
for i=1:1:num_trials
    
    %calculating the desired pose and the current pose
    T_des = T_desired(:,:,i);%robot.fkine(random_joints(:,i)).T;
    %     T_des = robot.fkine(q_rand(:,i)).T;
    T_cur = robot.fkine(qi).T;
    
    %initializing q
    
    
    counter = 0;
    % every time through loop, we check position error and if we have run
    % for 1000 iterations yet or not.
    while (norm(T_des- T_cur) > 0.0001) & (counter < 3000)
        % update pose and jacobian for current q's
        T_cur = robot.fkine(q).T;
        J = robot.jacob0(q);
        %J = [J(:, 1:3), J(:, 5:7)];
        %calculate error in pose and transform back to base frame.
        delta = tr2delta(T_cur, T_des);
        R2base = T_cur(1:3,1:3);
        delta_base = [R2base, zeros(3,3); zeros(3,3), R2base]*delta;
        %Debug for just position
        %         J = [J(1:3,:)];
        %         delta_base = [delta_base(1:3,:)];
        
        %perform the pseudo-inverse method
        qd = J'*inv(J*J'+0.01^2*eye(6))*K_inv*delta_base;
        %qd = [qd(1:3, :); 0; qd(4:6)];
        q = q + qd;
        %test if at joint limits
        for i=1:7
            if q(i)<robot.qlim(i,1)
                for j = 1:7
                    a = robot.qlim(j,1);
                    b = robot.qlim(j,2);
                    r = (b-a).*rand(1,1) + a;
                    q_randLimit(j) = [r];
                end
                q(i) = q_randLimit(i);
            end
            if q(i)>robot.qlim(i,2)
                for j = 1:7
                    a = robot.qlim(j,1);
                    b = robot.qlim(j,2);
                    r = (b-a).*rand(1,1) + a;
                    q_randLimit(j) = [r];
                end
                
                q(i) = q_randLimit(i);
            end
        end
        %       norm(T_des(1:3,4)- T_cur(1:3,4))
        norm(T_des- T_cur)
        counter = counter + 1;
    end
    
    if counter >= 3000
        status_pinv = [status_pinv, 0]
    else
        status_pinv = [status_pinv, 1]
    end
    
    %storing results for reference
    q_slns_pinv = [q_slns_pinv, q];
end


%when completed we can generate smooth joint trajectories between the start
% and end configurations for control
num_steps = 50

%this is a series of joint angle values defined by a number of steps it
%should take to reach the goal.

%we can do the same thing, but parameterized by time as well.
time = 0:0.01:1;
q_traj_time = mtraj(@lspb, qi', q_slns_pinv(:,1)', time);
%figure()
%plot(time, q_traj_time);
figure(1)
robot.plot(q_slns_pinv(:,5)')

filename = 'joint.mat';

save(filename, 'q_slns_pinv')